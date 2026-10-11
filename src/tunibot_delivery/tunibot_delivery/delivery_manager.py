"""Delivery task manager for the TuniBot waiter robot (Phase 2).

Receives food/beverage orders, queues them and drives the robot through
  dock -> kitchen pass (pickup) -> table (serve) -> next order ... -> dock
with Nav2. Phase 2 additions:

  * energy-aware mission planning: before an order starts, the energy for
    current -> pickup -> table -> dock is estimated from real Nav2 path lengths
    and the consumption per metre learned by the battery simulator. If it does
    not fit (plus a reserve), the robot charges first, only as long as needed.
  * return-to-dock before the battery dies: while driving, the energy needed to
    reach the dock from the current pose is re-planned every few seconds; the
    mission is interrupted (order re-queued at the front) as soon as
    remaining - needed < reserve. A critical SoC threshold is a hard safety net.
  * navigation failure handling: several approach poses per place, retries with
    costmap clearing, BLOCKED detection from Nav2 feedback, failed orders are
    reported without stalling the queue, ERROR state after repeated failures.
  * robust docking: Nav2 to the pre-dock pose, heading alignment, straight
    reverse until the charging contacts report power, retry if missed. The final
    distance to the pad is measured with the lidar (wall behind the robot, reference
    learned at start-up on the pad), not with AMCL.
  * localization watchdog, emergency stop / resume, VIP priority, event log.

Interfaces
  sub  /delivery/order    std_msgs/String  '{"table": "table_3", "items": "2 coffees", "priority": "vip"}' or 'table_3'
  sub  /delivery/command  std_msgs/String  dock | cancel | clear | estop | resume
  sub  /battery_state     sensor_msgs/BatteryState  (from battery_simulator)
  sub  /battery/info      std_msgs/String  JSON (learned Wh/m, idle power, ...)
  sub  /people            std_msgs/String  JSON (people_tracker, optional)
  sub  /scan              sensor_msgs/LaserScan  (final docking distance)
  pub  /delivery/status   std_msgs/String  JSON snapshot for the UI
  pub  /delivery/events   std_msgs/String  JSON, one message per event
"""
import itertools
import json
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

DOCKED, UNDOCKING, IDLE, TO_PICKUP, LOADING, TO_TABLE, SERVING = (
    'DOCKED', 'UNDOCKING', 'IDLE', 'TO_PICKUP', 'LOADING', 'TO_TABLE', 'SERVING')
TO_DOCK, DOCKING, CHARGING, ESTOP, ERROR, STARTING = (
    'RETURNING_TO_DOCK', 'DOCKING', 'CHARGING', 'EMERGENCY_STOP', 'ERROR', 'STARTING')


class Abort(Exception):
    """Raised inside the mission thread to unwind the current task."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rear_wall_distance(ranges, angle_min, angle_inc, range_min, range_max, lo=-2.54, hi=-2.18):
    """Perpendicular distance (m) from the lidar to the wall behind the robot, seen by the
    rear-right beams (the lidar is blind in a +-35 deg cone straight behind).

    A line x = m y + c is fitted to the hits in the lidar frame: points within 15 cm of
    their median depth first (a chair leg or a bag is far off it), least squares, then
    once more without points > 4 cm off the line. The fitted line, not a projection of
    the ranges, gives the distance, so a small heading error does not bias it. None
    (caller falls back to AMCL) if fewer than 8 beams or less than 75 % of them lie on
    that line - e.g. a person right behind."""
    pts = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        if lo <= a <= hi and range_min < r < range_max and math.isfinite(r):
            pts.append((r * math.cos(a), r * math.sin(a)))
    total = len(pts)
    if total < 8:
        return None
    x_med = sorted(p[0] for p in pts)[total // 2]
    pts = [p for p in pts if abs(p[0] - x_med) < 0.15]
    for _ in range(2):
        if len(pts) < 8 or len(pts) < 0.75 * total:
            return None
        n = len(pts)
        my = sum(p[1] for p in pts) / n
        mx = sum(p[0] for p in pts) / n
        syy = sum((p[1] - my) ** 2 for p in pts)
        if syy < 1e-6:
            return None
        m = sum((p[1] - my) * (p[0] - mx) for p in pts) / syy
        c = mx - m * my
        pts = [p for p in pts if abs(p[0] - (m * p[1] + c)) / math.sqrt(1 + m * m) < 0.04]
    if len(pts) < 0.75 * total or c >= 0.0:
        return None                     # not behind the robot
    return -c / math.sqrt(1.0 + m * m)


class DeliveryManager(Node):

    def __init__(self):
        super().__init__('delivery_manager')
        self.declare_parameter('tables', ['table_1'])
        self.tables = list(self.get_parameter('tables').value)
        self.places = {}
        for name in ['dock', 'pre_dock', 'pickup'] + self.tables:
            self.declare_parameter(name, [0.0, 0.0, 0.0])
            flat = [float(v) for v in self.get_parameter(name).value]
            self.places[name] = [flat[i:i + 3] for i in range(0, len(flat) - 2, 3)]
        defaults = {
            'load_time_sec': 5.0, 'serve_time_sec': 8.0, 'idle_dock_timeout_sec': 20.0,
            'battery_critical': 10.0,      # % : abort everything and go to the dock
            'battery_low': 25.0,           # % : no new order below this
            'battery_resume': 60.0,        # % : charging target when no order waits
            'energy_reserve': 12.0,        # % of capacity that must remain at the dock
            'energy_safety_factor': 1.3,   # margin on the path-length energy estimate
            'energy_check_period_sec': 3.0,
            'nav_attempts_per_pose': 3,
            'max_consecutive_failures': 3,
            'blocked_after_sec': 5.0,
            'path_wait_sec': 45.0,         # wait this long for a blocked route to clear (people)
            'cruise_speed': 0.35,          # m/s, used for the time part of energy estimates
            'pickup_depart_yaw': -1.5708,  # after loading, turn to face the dining room
            'pickup_staging_dist': 0.6,    # Nav2 stops this far back from a serving window
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}

        self.lock = threading.RLock()
        self.queue = deque()
        self.history = deque(maxlen=30)
        self.events = deque(maxlen=60)
        self.ids = itertools.count(1)
        self.current = None
        self.state = STARTING
        self.nav_status = ''
        self.docked = True
        self.cancel_requested = False
        self.dock_requested = False
        self.estop = False
        self.error = None              # latched until the operator sends "resume"
        self.consecutive_failures = 0
        self.energy = {}
        self.battery = {'soc': 100.0, 'charging': False, 'on_dock': True, 'capacity_wh': 259.0,
                        'wh_per_m': 1.1, 'idle_wh_per_s': 0.2, 'depleted': False}
        self.people = {'count': 0, 'nearest': None}
        self.odom = None
        # lidar docking reference: distance to the wall behind the robot, learned at start-up
        # while the robot still stands exactly on the charging pad where it was installed
        self.scan = None
        self.dock_ref = None
        self.dock_ref_samples = []
        self.left_dock_once = False

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(String, '/delivery/order', self.on_order, 10)
        self.create_subscription(String, '/delivery/command', self.on_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery_state, 10)
        self.create_subscription(String, '/battery/info', self.on_battery_info, 10)
        self.create_subscription(String, '/people', self.on_people, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.global_costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'global_costmap', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)
        self.event_pub = self.create_publisher(String, '/delivery/events', 10)
        # what the robot says (a real robot feeds this to text-to-speech + its screen)
        self.say_pub = self.create_publisher(String, '/tunibot/say', 10)
        self.last_say = {}
        self.speech = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.5, self.publish_status)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.last_reseed = 0.0
        self.create_timer(1.0, self.localization_supervisor)
        self.get_logger().info('Delivery manager ready. Tables: %s' % ', '.join(self.tables))

    # ------------------------------------------------------------------ utils
    def now(self):
        """Simulation time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds):
        end = self.now() + seconds
        while self.now() < end and rclpy.ok():
            time.sleep(0.05)

    def event(self, kind, text, **data):
        ev = {'t': round(self.now(), 1), 'wall': time.time(), 'kind': kind, 'text': text}
        ev.update(data)
        with self.lock:
            self.events.appendleft(ev)
        self.event_pub.publish(String(data=json.dumps(ev)))
        # (rclpy forbids changing the severity of a given logging call site)
        if kind in ('battery', 'failure', 'error', 'estop'):
            self.get_logger().warn('[%s] %s' % (kind, text))
        else:
            self.get_logger().info('[%s] %s' % (kind, text))

    def say(self, text, key=None, min_interval=8.0):
        """Speak to the people around (rate-limited per message kind)."""
        key = key or text
        now = self.now()
        if now - self.last_say.get(key, -1e9) < min_interval:
            return
        self.last_say[key] = now
        self.speech = {'t': round(now, 1), 'text': text}
        self.say_pub.publish(String(data=text))
        self.event('hri', 'Robot says: "%s"' % text)

    def pose_occupied(self, x, y, radius=0.25):
        """True if the global costmap has an obstacle (lethal or inscribed) within
        `radius` of (x, y): the goal itself is blocked (trolley, chairs, a standing person)."""
        m = self.global_costmap
        if m is None:
            return False
        res, w, h = m.info.resolution, m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        r = int(radius / res) + 1
        ci, cj = int((y - oy) / res), int((x - ox) / res)
        for i in range(max(0, ci - r), min(h, ci + r + 1)):
            for j in range(max(0, cj - r), min(w, cj + r + 1)):
                if (i - ci) ** 2 + (j - cj) ** 2 <= r * r and m.data[i * w + j] >= 99:
                    return True
        return False

    def rear_wall(self):
        s = self.scan
        if s is None:
            return None
        return rear_wall_distance(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)

    def on_scan(self, msg):
        self.scan = msg
        if self.dock_ref is None and not self.left_dock_once and self.docked:
            d = self.rear_wall()
            if d is not None:
                self.dock_ref_samples.append(d)
            if len(self.dock_ref_samples) >= 20:
                v = sorted(self.dock_ref_samples)
                self.dock_ref = v[len(v) // 2]
                self.event('dock', 'Docking reference learned from the lidar: wall %.3f m behind the lidar'
                           % self.dock_ref)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def localization_supervisor(self):
        """AMCL watchdog (1 Hz, independent of the mission).

        AMCL (Humble) occasionally stops refreshing map->odom; every TF lookup then
        fails with 'transform too old' and the robot is stuck. When map->odom is stale,
        re-seed AMCL with the best current estimate: last map->odom composed with the
        live odometry, stamped with the odometry time so that AMCL can use it."""
        try:
            m2o = self.tf.lookup_transform('map', 'odom', Time())
            o2b = self.tf.lookup_transform('odom', 'base_footprint', Time())
        except TransformException:
            return
        now = self.now()
        age = now - Time.from_msg(m2o.header.stamp).nanoseconds * 1e-9
        if age < 2.5 or now - self.last_reseed < 5.0:
            return
        self.last_reseed = now
        a, b = m2o.transform, o2b.transform
        ya, yb = yaw_of(a.rotation), yaw_of(b.rotation)
        x = a.translation.x + math.cos(ya) * b.translation.x - math.sin(ya) * b.translation.y
        y = a.translation.y + math.sin(ya) * b.translation.x + math.cos(ya) * b.translation.y
        yaw = ya + yb
        ps = PoseWithCovarianceStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = o2b.header.stamp
        ps.pose.pose.position.x, ps.pose.pose.position.y = x, y
        ps.pose.pose.orientation.z, ps.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ps.pose.covariance[0] = ps.pose.covariance[7] = 0.02
        ps.pose.covariance[35] = 0.02
        self.initialpose_pub.publish(ps)
        self.event('recovery', 'Localization watchdog: map->odom stale for %.0f s, AMCL re-seeded at '
                   '(%.2f, %.2f)' % (age, x, y))

    @property
    def soc(self):
        return float(self.battery.get('soc', 100.0))

    # ------------------------------------------------------------------ inputs
    def on_order(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError
        except ValueError:
            data = {'table': msg.data.strip()}
        table = str(data.get('table', ''))
        if table not in self.tables:
            self.event('order', 'Rejected order for unknown table "%s"' % table)
            return
        vip = str(data.get('priority', '')).lower() == 'vip'
        order = {'id': next(self.ids), 'table': table, 'items': str(data.get('items', '')),
                 'priority': 'vip' if vip else 'normal', 'status': 'queued',
                 'created': time.time(), 'attempts': 0, 'notes': []}
        with self.lock:
            if vip:  # VIP orders go before every normal order
                idx = sum(1 for o in self.queue if o['priority'] == 'vip')
                self.queue.insert(idx, order)
            else:
                self.queue.append(order)
        self.event('order', 'Order #%d queued -> %s %s(%s)' % (
            order['id'], table, '[VIP] ' if vip else '', order['items']), order_id=order['id'])

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        with self.lock:
            if cmd == 'dock':
                self.dock_requested = True
            elif cmd == 'cancel':
                self.cancel_requested = True
            elif cmd == 'clear':
                for o in self.queue:
                    o['status'] = 'cancelled'
                    self.history.appendleft(o)
                self.queue.clear()
            elif cmd == 'estop':
                self.estop = True
            elif cmd == 'resume':
                self.estop = False
                self.error = None
                self.consecutive_failures = 0
            else:
                self.get_logger().warn('Unknown command "%s"' % cmd)
                return
        self.event('command', 'Operator command: %s' % cmd)

    def on_odom(self, msg):
        self.odom = msg

    def on_battery_state(self, msg):
        with self.lock:
            self.battery['soc'] = msg.percentage * 100.0
            self.battery['voltage'] = round(msg.voltage, 2)
            self.battery['current'] = round(msg.current, 2)
            self.battery['charging'] = msg.power_supply_status in (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING, BatteryState.POWER_SUPPLY_STATUS_FULL)
            self.battery['depleted'] = msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD

    def on_battery_info(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.battery.update({k: info[k] for k in (
                'capacity_wh', 'wh_per_m', 'wh_per_m_effective', 'idle_wh_per_s', 'power_w', 'power_avg_w', 'on_dock',
                'time_to_empty_min', 'time_acceleration', 'energy_used_wh', 'distance_m',
                'charger_on', 'cycles', 'soh') if k in info})

    def on_people(self, msg):
        try:
            self.people = json.loads(msg.data)
        except ValueError:
            pass

    # ------------------------------------------------------------------ status
    def set_state(self, state, nav_status=''):
        with self.lock:
            changed = state != self.state
            self.state = state
            self.nav_status = nav_status
        if changed:
            self.get_logger().info('State -> %s' % state)

    def publish_status(self):
        with self.lock:
            status = {
                'state': self.state, 'nav_status': self.nav_status, 'docked': self.docked,
                'battery': {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.battery.items()},
                'energy': self.energy, 'current_order': self.current, 'queue': list(self.queue),
                'history': list(self.history), 'events': list(self.events)[:25], 'tables': self.tables,
                'speech': self.speech,
                'estop': self.estop, 'error': self.error, 'consecutive_failures': self.consecutive_failures,
                'people': self.people, 'pose': self.robot_pose(), 'sim_time': round(self.now(), 1),
                'places': {k: v[0] for k, v in self.places.items()},
                'thresholds': {k: self.cfg[k] for k in ('battery_critical', 'battery_low',
                                                        'battery_resume', 'energy_reserve')},
            }
        self.status_pub.publish(String(data=json.dumps(status)))


class Mission:
    """Mission state machine, run in its own thread (BasicNavigator blocks)."""

    def __init__(self, mgr: DeliveryManager):
        self.mgr = mgr
        self.nav = BasicNavigator('delivery_navigator')
        self.planner = BasicNavigator('delivery_energy_planner')   # separate node: path queries
        self.planner.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)  # one query every few s
        self.last_energy_check = 0.0

    # ------------------------------------------------------------ helpers
    def pose_msg(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.nav.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def path_length(self, start, goal):
        """Length of the Nav2 global plan from start (None = robot) to goal pose list entry."""
        g = self.pose_msg(*goal)
        try:
            if start is None:
                path = self.planner.getPath(g, g, use_start=False)
            else:
                path = self.planner.getPath(self.pose_msg(*start), g, use_start=True)
        except Exception:  # noqa: BLE001 - planner unavailable
            path = None
        if path is None or len(path.poses) < 2:
            # fall back to 1.5 x the straight line (conservative)
            s = start or self.mgr.robot_pose() or (0.0, 0.0, 0.0)
            return 1.5 * math.hypot(goal[0] - s[0], goal[1] - s[1]), False
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])), True

    def energy_for(self, length_m, extra_wait_s=0.0):
        """Wh needed for a path of length_m (+ waiting time), with the safety factor."""
        b, c = self.mgr.battery, self.mgr.cfg
        # effective Wh/m learned over whole missions (stops included), never below the
        # pure driving figure
        per_m = max(float(b.get('wh_per_m', 1.1)), float(b.get('wh_per_m_effective', 0.0)))
        drive = length_m * per_m
        wait = extra_wait_s * float(b.get('idle_wh_per_s', 0.2))
        dock_manoeuvre = 1.5 * per_m
        return (drive + wait + dock_manoeuvre) * c['energy_safety_factor']

    def reserve_wh(self):
        return self.mgr.cfg['energy_reserve'] / 100.0 * float(self.mgr.battery.get('capacity_wh', 259.0))

    def available_wh(self):
        return self.mgr.soc / 100.0 * float(self.mgr.battery.get('capacity_wh', 259.0))

    def order_energy(self, order):
        """Energy (Wh) to do the whole order from where the robot is and get back to the dock."""
        m = self.mgr
        pickup, table, dock = m.places['pickup'][0], m.places[order['table']][0], m.places['pre_dock'][0]
        start = None if not m.docked else m.places['pre_dock'][0]
        l1, ok1 = self.path_length(start, pickup)
        l2, ok2 = self.path_length(pickup, table)
        l3, ok3 = self.path_length(table, dock)
        total = l1 + l2 + l3
        need = self.energy_for(total, m.cfg['load_time_sec'] + m.cfg['serve_time_sec'])
        return need, total, ok1 and ok2 and ok3

    def check_return_energy(self, force=False):
        """While away from the dock: abort if the energy to get back is running out."""
        m = self.mgr
        if m.soc <= m.cfg['battery_critical']:
            raise Abort('battery critical (%.0f%%)' % m.soc)
        t = m.now()
        if not force and t - self.last_energy_check < m.cfg['energy_check_period_sec']:
            return
        self.last_energy_check = t
        length, planned = self.path_length(None, m.places['pre_dock'][0])
        need = self.energy_for(length)
        margin = self.available_wh() - need - self.reserve_wh()
        with m.lock:
            m.energy = {'return_path_m': round(length, 1), 'return_wh': round(need, 1),
                        'available_wh': round(self.available_wh(), 1), 'reserve_wh': round(self.reserve_wh(), 1),
                        'margin_wh': round(margin, 1), 'planned': planned}
        if margin < 0.0:
            raise Abort('energy to reach the dock: %.1f Wh needed, %.1f Wh left above the reserve'
                        % (need, self.available_wh() - self.reserve_wh()))

    def check_interrupts(self, away=True):
        m = self.mgr
        if m.estop:
            raise Abort('emergency stop')
        if m.cancel_requested:
            raise Abort('cancelled by operator')
        if away:
            self.check_return_energy()

    # ------------------------------------------------------------ motion
    def hold(self, seconds):
        """Pause while actively braking (zero velocity at 10 Hz): any late velocity
        command from another source cannot move the robot during the pause."""
        end = self.mgr.now() + seconds
        while self.mgr.now() < end and rclpy.ok():
            self.mgr.cmd_pub.publish(Twist())
            time.sleep(0.1)

    def stop(self):
        """Zero velocity, sent a few times: the simulated base (gazebo diff drive) has no
        command timeout and keeps executing the last /cmd_vel it received."""
        for _ in range(3):
            self.mgr.cmd_pub.publish(Twist())
            time.sleep(0.03)

    def navigate(self, x, y, yaw, label, away=True):
        """One Nav2 NavigateToPose with monitoring. Returns True on success."""
        self.nav.goToPose(self.pose_msg(x, y, yaw))
        # give up on this goal if it takes far longer than the planned path justifies
        deadline = self.mgr.now() + 60.0 + 4.0 * getattr(self, 'last_path_length', 20.0)
        blocked, recoveries = False, 0
        anchor, anchor_t = self.mgr.robot_pose(), self.mgr.now()
        base_state = self.mgr.state
        try:
            while not self.nav.isTaskComplete():
                self.check_interrupts(away)
                if self.mgr.now() > deadline:
                    self.mgr.event('failure', 'Giving up on %s: no arrival within %.0f s' % (
                        label, 60.0 + 4.0 * getattr(self, 'last_path_length', 20.0)))
                    self.nav.cancelTask()
                    self.stop()
                    return False
                fb = self.nav.getFeedback()
                # blocked = the robot has not really moved (10 cm or 30 deg) for a while
                pose = self.mgr.robot_pose()
                if pose is not None and anchor is not None and (
                        math.hypot(pose[0] - anchor[0], pose[1] - anchor[1]) > 0.10 or
                        abs(math.remainder(pose[2] - anchor[2], 2 * math.pi)) > 0.5):
                    anchor, anchor_t = pose, self.mgr.now()
                    if blocked:
                        blocked = False
                        self.mgr.event('nav', 'Moving again towards %s' % label)
                elif anchor is None:
                    anchor, anchor_t = pose, self.mgr.now()
                elif not blocked and self.mgr.now() - anchor_t > self.mgr.cfg['blocked_after_sec']:
                    blocked = True
                    self.mgr.event('nav', 'Blocked on the way to %s (people around: %s) - waiting / replanning'
                                   % (label, self.mgr.people.get('count', 0)))
                if blocked:
                    near = self.mgr.people.get('nearest')
                    if near is not None and near < 1.8:
                        self.mgr.say('Excuse me, may I pass? Thank you!', key='excuse')
                if fb is not None:
                    if fb.number_of_recoveries > recoveries:
                        recoveries = fb.number_of_recoveries
                        self.mgr.event('recovery', 'Nav2 recovery #%d on the way to %s' % (recoveries, label))
                    self.mgr.set_state(base_state, 'blocked' if blocked else
                                       'moving (%.1f m left)' % fb.distance_remaining)
                time.sleep(0.1)
        except Abort:
            self.nav.cancelTask()
            raise
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def wait_for_path(self, pose, label, away):
        """Reachability check with the global planner before committing to a goal.

        A route blocked by people usually clears within seconds: re-check (clearing
        stale obstacles) for up to path_wait_sec, then give up on this approach.
        """
        m = self.mgr
        deadline = m.now() + m.cfg['path_wait_sec']
        announced = False
        last_clear = m.now()
        while True:
            self.check_interrupts(away)
            length, ok = self.path_length(None, pose)
            goal_blocked = m.pose_occupied(pose[0], pose[1])
            ok = ok and not goal_blocked
            if ok:
                self.last_path_length = length
                if announced:
                    m.event('nav', 'Route to %s is free again' % label)
                return True
            if not announced:
                m.event('nav', 'No route to %s right now (route or goal blocked) - re-checking for %.0f s'
                        % (label, m.cfg['path_wait_sec']))
                announced = True
                m.set_state(m.state, 'waiting: route blocked')
            if m.now() > deadline:
                return False
            # Route blocked (e.g. a person in a doorway): every 10 s forget the costmap and
            # let the lidar re-mark what is really still there (1.5 s) - a person who left
            # is gone, a wall in view is back after one scan. Goal itself occupied (e.g. a
            # trolley on the approach pose): keep the memory, the alternate approach is next.
            if not goal_blocked and m.now() - last_clear > 10.0:
                last_clear = m.now()
                self.nav.clearAllCostmaps()
                self.hold(1.5)
            else:
                m.sleep(2.0)

    def go(self, name, away=True, label=None):
        """Navigate to a named place, trying each approach pose. Returns False if impossible."""
        label = label or name
        poses = self.mgr.places[name]
        # Serving windows are 0.5 m from the partition wall: Nav2 stops at a staging
        # pose in open floor, then the robot faces the window and creeps in straight.
        # (Humble's rotation shim turns in place without collision checking: a turn
        # close to a wall would sweep the robot's corner into it.)
        staging = self.mgr.cfg['pickup_staging_dist'] if name == 'pickup' else 0.0
        for i, (x, y, yaw) in enumerate(poses):
            sx, sy = x - staging * math.cos(yaw), y - staging * math.sin(yaw)
            for attempt in range(int(self.mgr.cfg['nav_attempts_per_pose'])):
                if not self.wait_for_path((sx, sy, yaw), label, away):
                    self.mgr.event('failure', 'Approach %d/%d of %s unreachable' % (i + 1, len(poses), label))
                    break
                self.face_route((sx, sy, yaw))
                if self.navigate(sx, sy, yaw, label, away):
                    self.rotate_to(yaw, tol=0.05)   # face the window / table
                    if staging:
                        self.goto_point(x, y, tol=0.04)
                        self.rotate_to(yaw, tol=0.05)
                    return True
                self.mgr.event('failure', 'Navigation to %s failed (approach %d/%d, attempt %d)' % (
                    label, i + 1, len(poses), attempt + 1))
                # (no costmap clearing: what the robot just saw - e.g. a trolley parked on
                # the approach pose - is exactly what the next reachability check needs;
                # a short pause lets people walking around move on)
                self.hold(3.0)
        return False

    def face_route(self, goal, lookahead=0.8, min_turn=1.0):
        """Before leaving a place, turn in place towards the first metre of the planned route.

        After serving, the robot faces the table; the way out is often behind it. MPPI
        sometimes does not settle into a 180 deg turn next to a table and oscillates on the
        spot (moving, so never 'blocked'). Turning first removes that. Only with room to
        turn: nothing lethal / inscribed within 45 cm (never inside the narrow corridor)."""
        m = self.mgr
        pose = m.robot_pose()
        if pose is None or m.pose_occupied(pose[0], pose[1], radius=0.45):
            return
        try:
            path = self.planner.getPath(self.pose_msg(*goal), self.pose_msg(*goal), use_start=False)
        except Exception:  # noqa: BLE001 - planner unavailable: let Nav2 handle the turn
            return
        if path is None or len(path.poses) < 2:
            return
        x0, y0 = path.poses[0].pose.position.x, path.poses[0].pose.position.y
        target = None
        for p in path.poses:
            if math.hypot(p.pose.position.x - x0, p.pose.position.y - y0) >= lookahead:
                target = (p.pose.position.x, p.pose.position.y)
                break
        if target is None:
            return
        yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
        if abs(math.remainder(yaw - pose[2], 2 * math.pi)) > min_turn:
            self.rotate_to(yaw, tol=0.15)

    def drive(self, distance, speed, until=None, timeout=None):
        """Odometry closed-loop straight motion (negative = reverse). Stops early if until()."""
        m = self.mgr
        while m.odom is None:
            time.sleep(0.1)
        start = m.odom.pose.pose.position
        cmd = Twist()
        cmd.linear.x = math.copysign(speed, distance)
        deadline = m.now() + (timeout or abs(distance) / speed * 2.5 + 3.0)
        while m.now() < deadline and not m.estop:
            cur = m.odom.pose.pose.position
            if math.hypot(cur.x - start.x, cur.y - start.y) >= abs(distance) or (until and until()):
                break
            m.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.stop()

    def rotate_to(self, yaw_target, tol=0.03, timeout=15.0):
        m = self.mgr
        end = m.now() + timeout
        cmd = Twist()
        while m.now() < end and not m.estop:
            pose = m.robot_pose()
            if pose is None:
                break
            err = math.remainder(yaw_target - pose[2], 2 * math.pi)
            if abs(err) < tol:
                break
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * err))
            if abs(cmd.angular.z) < 0.12:
                cmd.angular.z = math.copysign(0.12, err)
            m.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.stop()

    def undock(self):
        m = self.mgr
        if m.docked:
            m.set_state(UNDOCKING)
            m.left_dock_once = True
            self.drive(0.6, 0.15)
            with m.lock:
                m.docked = False
            m.event('dock', 'Undocked')

    def goto_point(self, tx, ty, tol=0.03, timeout=25.0):
        """Short, precise closed-loop move to (tx, ty) on the localized pose: turn to
        face the point, then drive with heading correction (used for docking)."""
        m = self.mgr
        end = m.now() + timeout
        cmd = Twist()
        while m.now() < end and not m.estop:
            pose = m.robot_pose()
            if pose is None:
                break
            dx, dy = tx - pose[0], ty - pose[1]
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            err = math.remainder(math.atan2(dy, dx) - pose[2], 2 * math.pi)
            if abs(err) > 0.35:
                cmd.linear.x, cmd.angular.z = 0.0, math.copysign(min(0.5, max(0.15, abs(err))), err)
            else:
                cmd.linear.x = min(0.12, max(0.04, 0.6 * dist))
                cmd.angular.z = max(-0.4, min(0.4, 1.5 * err))
            m.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.stop()

    def reverse_onto_dock(self, dx, dy, dyaw, speed=0.08, timeout=40.0):
        """Straight reverse along the dock axis, holding the heading and correcting
        the lateral offset, until the robot reaches the charging pad."""
        m = self.mgr
        end = m.now() + timeout
        cmd = Twist()
        c, s = math.cos(dyaw), math.sin(dyaw)
        p0 = m.robot_pose()
        max_travel = (0.0 if p0 is None else (p0[0] - dx) * c + (p0[1] - dy) * s) + 0.15
        start = m.odom.pose.pose.position
        while m.now() < end and not m.estop:
            pose = m.robot_pose()
            if pose is None:
                break
            cur = m.odom.pose.pose.position
            if math.hypot(cur.x - start.x, cur.y - start.y) > max_travel:
                break                   # odometry cap: never drive into the station
            along = (pose[0] - dx) * c + (pose[1] - dy) * s       # > 0: still in front of the pad
            lateral = -(pose[0] - dx) * s + (pose[1] - dy) * c
            # along the dock axis the lidar beats AMCL: the wall behind is in the lidar's
            # blind cone for AMCL too, so the localized pose can be several cm off here
            # (only when both agree within 12 cm: a person or a chair in the rear beams can
            # look like a wall right behind the robot)
            wall = m.rear_wall() if m.dock_ref is not None else None
            if wall is not None and abs((wall - m.dock_ref) - along) < 0.12:
                along = wall - m.dock_ref
            if along < 0.015:
                break
            # reversing: a small positive yaw moves the robot to its right, so the
            # desired heading leans towards the axis by an amount ~ lateral offset
            # (the lean fades out over the last 30 cm so the robot arrives straight)
            desired = dyaw + max(-0.2, min(0.2, 1.2 * lateral)) * min(1.0, along / 0.3)
            heading_err = math.remainder(desired - pose[2], 2 * math.pi)
            cmd.linear.x = -min(speed, max(0.03, along))
            cmd.angular.z = max(-0.3, min(0.3, 2.0 * heading_err))
            m.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.stop()

    def dock(self, reason):
        """Return to the dock and get onto the charging contacts. True when charging."""
        m = self.mgr
        m.set_state(TO_DOCK)
        m.event('dock', 'Returning to dock: %s' % reason)
        dx, dy, dyaw = m.places['dock'][0]
        px, py, _ = m.places['pre_dock'][0]
        for attempt in range(3):
            if not self.go('pre_dock', away=False, label='the dock'):
                continue
            m.set_state(DOCKING)
            # Nav2 stops within 25 cm of the pre-dock pose: fine alignment on the axis
            self.goto_point(px, py)
            self.rotate_to(dyaw, tol=0.03)
            pose = m.robot_pose()
            lat = 0.0 if pose is None else -(pose[0] - dx) * math.sin(dyaw) + (pose[1] - dy) * math.cos(dyaw)
            if abs(lat) > 0.06:
                m.event('dock', 'Docking: %.2f m off the axis, aligning again' % lat)
                self.drive(0.3, 0.1)
                continue
            self.reverse_onto_dock(dx, dy, dyaw)
            self.rotate_to(dyaw, tol=0.03)  # final heading touch-up on the pad
            self.hold(1.5)              # charger contact check, robot braked
            if m.battery.get('on_dock', False) or m.battery.get('charging', False):
                with m.lock:
                    m.docked = True
                    m.dock_requested = False
                m.set_state(DOCKED)
                pose = m.robot_pose() or (dx, dy, dyaw)
                wall = m.rear_wall() if m.dock_ref is not None else None
                lat = -(pose[0] - dx) * math.sin(dyaw) + (pose[1] - dy) * math.cos(dyaw)
                along = (pose[0] - dx) * math.cos(dyaw) + (pose[1] - dy) * math.sin(dyaw)
                lidar = wall is not None and abs((wall - m.dock_ref) - along) < 0.12
                if lidar:
                    along = wall - m.dock_ref
                m.event('dock', 'Docked, charging contacts OK (%.0f%%), pad offset %.1f cm (%s)' % (
                    m.soc, 100 * math.hypot(along, lat), 'lidar' if lidar else 'AMCL'))
                return True
            m.event('failure', 'Docking attempt %d: no charging contact, retrying' % (attempt + 1))
            self.drive(0.6, 0.12)
        m.error = 'docking failed'
        m.set_state(ERROR, 'docking failed - send "resume"')
        m.event('error', 'Docking failed 3 times - operator needed')
        return False

    # ------------------------------------------------------------ tasks
    def deliver(self, order):
        m = self.mgr
        order['status'] = 'in_progress'
        order['attempts'] += 1
        order['started'] = time.time()
        with m.lock:
            m.current = order
        try:
            m.set_state(TO_PICKUP)
            if not self.go('pickup', label='the kitchen pass'):
                raise Abort('kitchen pass unreachable')
            m.set_state(LOADING)
            self.wait_checked(m.cfg['load_time_sec'])
            m.set_state(TO_TABLE)
            # deliberate departure: back out of the window to the staging pose, then
            # turn towards the dining room before handing over to Nav2
            self.wait_rear_clear(m.cfg['pickup_staging_dist'] + 0.5)
            self.drive(-m.cfg['pickup_staging_dist'], 0.1)
            self.rotate_to(m.cfg['pickup_depart_yaw'], tol=0.1)
            if not self.go(order['table']):
                raise Abort('%s unreachable' % order['table'])
            m.set_state(SERVING)
            m.say('Hello! Your order has arrived%s. Please take it from the tray.'
                  % (' (%s)' % order['items'] if order.get('items') else ''), key='serve', min_interval=0.0)
            self.wait_checked(m.cfg['serve_time_sec'])
            order['status'] = 'delivered'
            m.consecutive_failures = 0
            m.event('delivery', 'Order #%d delivered to %s' % (order['id'], order['table']),
                    order_id=order['id'])
        except Abort as a:
            order['notes'].append(a.reason)
            if a.reason.startswith(('battery', 'energy')):
                order['status'] = 'queued'
                with m.lock:
                    m.queue.appendleft(order)       # not lost: first in line after charging
                m.event('battery', 'Order #%d interrupted (%s); re-queued, going to charge'
                        % (order['id'], a.reason))
                raise
            if a.reason in ('emergency stop',):
                order['status'] = 'queued'
                with m.lock:
                    m.queue.appendleft(order)
                raise
            order['status'] = 'cancelled' if 'cancel' in a.reason else 'failed'
            if order['status'] == 'failed':
                m.consecutive_failures += 1
            m.event('failure', 'Order #%d %s: %s' % (order['id'], order['status'], a.reason),
                    order_id=order['id'])
        finally:
            with m.lock:
                if order['status'] != 'queued':
                    order['finished'] = time.time()
                    m.history.appendleft(order)
                m.current = None
                m.cancel_requested = False

    def wait_rear_clear(self, dist, timeout=20.0):
        """The lidar does not see behind the robot (290 deg): before backing out, wait
        until no tracked person is within `dist` behind it."""
        m = self.mgr
        end = m.now() + timeout
        announced = False
        while m.now() < end:
            pose = m.robot_pose()
            behind = []
            for t in m.people.get('tracks', []) if pose else []:
                dx, dy = t['x'] - pose[0], t['y'] - pose[1]
                along = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
                if along < 0.0 and math.hypot(dx, dy) < dist:
                    behind.append(t)
            if not behind:
                return
            if not announced:
                m.event('nav', 'Person behind the robot - waiting before backing out')
                announced = True
            self.check_interrupts()
            time.sleep(0.2)

    def wait_checked(self, seconds):
        """Wait (loading / serving) holding the robot still, with interrupt checks."""
        end = self.mgr.now() + seconds
        while self.mgr.now() < end:
            self.check_interrupts()
            self.mgr.cmd_pub.publish(Twist())   # brake: hold position while staff/customers interact
            time.sleep(0.2)

    def handle_estop(self):
        m = self.mgr
        m.set_state(ESTOP)
        m.event('estop', 'EMERGENCY STOP - motors held at zero until "resume"')
        self.nav.cancelTask()
        while m.estop and rclpy.ok():
            m.cmd_pub.publish(Twist())
            time.sleep(0.1)
        m.event('command', 'Resumed after emergency stop')

    def can_start(self, order):
        """Energy check before leaving with an order. Returns (ok, need_soc)."""
        m = self.mgr
        need, length, planned = self.order_energy(order)
        need_soc = (need + self.reserve_wh()) / float(m.battery.get('capacity_wh', 259.0)) * 100.0
        with m.lock:
            m.energy = {'order_id': order['id'], 'order_path_m': round(length, 1), 'order_wh': round(need, 1),
                        'need_soc': round(need_soc, 1), 'planned': planned}
        ok = m.soc >= max(need_soc, m.cfg['battery_low'])
        return ok, max(need_soc, m.cfg['battery_low'])

    def charge_until(self, target_soc, reason):
        m = self.mgr
        m.set_state(CHARGING, 'to %.0f%%' % target_soc)
        m.event('battery', 'Charging to %.0f%% (%s), now %.0f%%' % (target_soc, reason, m.soc))
        no_current_since, fault = None, False
        while rclpy.ok() and m.soc < target_soc and not m.estop and not m.cancel_requested:
            m.cmd_pub.publish(Twist())          # parked on the charger: brake
            # charger fault: on the pad (contacts OK) but no charging current
            if m.battery.get('on_dock', False) and not m.battery.get('charging', False):
                no_current_since = no_current_since or m.now()
                if not fault and m.now() - no_current_since > 10.0:
                    fault = True
                    m.set_state(CHARGING, 'CHARGER FAULT - no current')
                    m.event('error', 'Charger fault: on the pad but no charging current for 10 s - '
                                     'staff notified, orders wait until it is fixed')
            else:
                no_current_since = None
                if fault and m.battery.get('charging', False):
                    fault = False
                    m.set_state(CHARGING, 'to %.0f%%' % target_soc)
                    m.event('battery', 'Charger OK again, charging resumed (%.0f%%)' % m.soc)
            if not m.battery.get('charging', False) and not m.battery.get('on_dock', False):
                m.event('error', 'Not receiving charge on the dock - re-docking')
                with m.lock:
                    m.docked = False
                self.drive(0.5, 0.12)
                self.dock('charger contact lost')
            time.sleep(0.5)
        m.cancel_requested = False
        m.set_state(DOCKED)

    def wait_node_active(self, node):
        """Wait until a lifecycle node is ACTIVE. Unlike BasicNavigator.waitUntilNav2Active
        (one service call without timeout - hangs forever if the FastDDS response is lost),
        every query uses a fresh client and a 3 s timeout."""
        from lifecycle_msgs.srv import GetState
        while rclpy.ok():
            cli = self.nav.create_client(GetState, '/%s/get_state' % node)
            try:
                if cli.wait_for_service(timeout_sec=3.0):
                    fut = cli.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=3.0)
                    if fut.done() and fut.result() is not None and fut.result().current_state.label == 'active':
                        return
            finally:
                self.nav.destroy_client(cli)
            time.sleep(1.0)

    # ------------------------------------------------------------ main loop
    def run(self):
        m = self.mgr
        # AMCL initialises itself on the dock (set_initial_pose in nav2_params.yaml).
        # Publishing another initial pose here raced with TF at startup ("Failed to
        # transform initial pose in time") and BasicNavigator then waited forever
        # for /amcl_pose: wait for the navigator and for the localized TF instead.
        self.wait_node_active('bt_navigator')
        self.wait_node_active('planner_server')
        while rclpy.ok() and m.robot_pose() is None:
            time.sleep(0.5)
        m.set_state(DOCKED)
        m.event('system', 'Nav2 active, robot on the dock')
        idle_since = m.now()
        while rclpy.ok():
            try:
                idle_since = self.step(idle_since)
            except Abort as a:
                if a.reason == 'emergency stop':
                    self.handle_estop()
                elif a.reason.startswith(('battery', 'energy')):
                    self.dock(a.reason)
                else:
                    m.event('failure', 'Aborted: %s' % a.reason)
                idle_since = m.now()
            time.sleep(0.2)

    def step(self, idle_since):
        m = self.mgr
        if m.estop:
            raise Abort('emergency stop')
        with m.lock:
            order = m.queue[0] if m.queue else None
            docked = m.docked
        if m.error:
            if m.state != ERROR:
                m.set_state(ERROR, '%s - send "resume"' % m.error)
            return idle_since
        if m.state == ERROR:                     # operator sent "resume"
            m.set_state(DOCKED if docked else IDLE)
            return m.now()
        if m.consecutive_failures >= m.cfg['max_consecutive_failures']:
            m.event('error', '%d consecutive failed deliveries - returning to dock, operator needed'
                    % m.consecutive_failures)
            if not docked:
                self.dock('too many failures')
            m.error = '%d failed deliveries' % m.consecutive_failures
            return idle_since
        if not docked:
            if m.soc <= m.cfg['battery_critical']:
                raise Abort('battery critical (%.0f%%)' % m.soc)
            self.check_return_energy()
        if m.dock_requested and not docked:
            self.dock('operator request')
            return m.now()
        if order is not None:
            ok, need_soc = self.can_start(order)
            if not ok and need_soc > 100.0 and docked and m.soc >= 98.0:
                # even a full battery is not enough: reject instead of charging forever
                with m.lock:
                    if m.queue and m.queue[0] is order:
                        m.queue.popleft()
                    order['status'] = 'failed'
                    order['notes'].append('beyond battery range (needs %.0f%%)' % need_soc)
                    order['finished'] = time.time()
                    m.history.appendleft(order)
                m.event('failure', 'Order #%d rejected: needs %.0f%% of the battery, more than a full charge'
                        % (order['id'], need_soc), order_id=order['id'])
                return m.now()
            if not ok:
                if docked:
                    self.charge_until(min(100.0, need_soc * 1.15 + 5.0),
                                      'order #%d needs %.0f%%' % (order['id'], need_soc))
                else:
                    raise Abort('energy: order #%d needs %.0f%%, battery %.0f%%' % (order['id'], need_soc, m.soc))
                return m.now()
            with m.lock:
                if m.queue and m.queue[0] is order:
                    m.queue.popleft()
            self.undock()
            self.deliver(order)
            if m.state not in (DOCKED, ERROR):
                m.set_state(IDLE)
            return m.now()
        if docked:
            m.cmd_pub.publish(Twist())          # parked on the charger: brake
            if m.soc < m.cfg['battery_resume'] and m.battery.get('charging', False):
                m.set_state(CHARGING, 'to %.0f%%' % m.cfg['battery_resume'])
            elif m.state != DOCKED:
                m.set_state(DOCKED)
            return idle_since
        if m.soc < m.cfg['battery_low']:
            self.dock('battery low (%.0f%%)' % m.soc)
            return m.now()
        if m.now() - idle_since > m.cfg['idle_dock_timeout_sec']:
            self.dock('idle for %.0fs' % m.cfg['idle_dock_timeout_sec'])
            return m.now()
        return idle_since


def main():
    rclpy.init()
    mgr = DeliveryManager()
    executor = MultiThreadedExecutor()
    executor.add_node(mgr)
    threading.Thread(target=Mission(mgr).run, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mgr.cmd_pub.publish(Twist())
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
