"""Delivery task manager for the TuniBot waiter robot.

Receives food/beverage orders, queues them, and drives the robot through
  dock -> bar (pickup) -> table (serve) -> next order ... -> dock
using Nav2 (nav2_simple_commander). When idle for a while, or when the simulated
battery is low, it returns to the docking station: Nav2 brings it to a pre-dock
pose, then a straight odometry-controlled reverse puts it on the charging pad.

Interfaces
  sub  /delivery/order    std_msgs/String  '{"table": "table_3", "items": "2 coffees"}' or 'table_3'
  sub  /delivery/command  std_msgs/String  'dock' | 'cancel' | 'clear'
  pub  /delivery/status   std_msgs/String  JSON snapshot (state, queue, battery, history)
  pub  /battery_state     sensor_msgs/BatteryState
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
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

DOCKED, UNDOCKING, IDLE, TO_PICKUP, LOADING, TO_TABLE, SERVING, TO_DOCK, DOCKING, CHARGING = (
    'DOCKED', 'UNDOCKING', 'IDLE', 'TO_PICKUP', 'LOADING', 'TO_TABLE', 'SERVING',
    'RETURNING_TO_DOCK', 'DOCKING', 'CHARGING')


class DeliveryManager(Node):

    def __init__(self):
        super().__init__('delivery_manager')
        self.declare_parameter('tables', ['table_1'])
        self.tables = list(self.get_parameter('tables').value)
        self.places = {}
        for name in ['dock', 'pre_dock', 'pickup'] + self.tables:
            self.declare_parameter(name, [0.0, 0.0, 0.0])
            self.places[name] = list(self.get_parameter(name).value)
        for name, default in [('load_time_sec', 5.0), ('serve_time_sec', 8.0),
                              ('idle_dock_timeout_sec', 15.0), ('battery_drain_per_sec', 0.15),
                              ('battery_charge_per_sec', 1.0), ('battery_low_threshold', 25.0)]:
            self.declare_parameter(name, default)
        p = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        self.load_time, self.serve_time = p('load_time_sec'), p('serve_time_sec')
        self.idle_timeout = p('idle_dock_timeout_sec')
        self.drain, self.charge = p('battery_drain_per_sec'), p('battery_charge_per_sec')
        self.low_battery = p('battery_low_threshold')

        self.lock = threading.Lock()
        self.queue = deque()
        self.history = deque(maxlen=20)
        self.ids = itertools.count(1)
        self.current = None
        self.state = DOCKED
        self.battery = 100.0
        self.docked = True
        self.cancel_requested = False
        self.dock_requested = False
        self.odom = None

        self.create_subscription(String, '/delivery/order', self.on_order, 10)
        self.create_subscription(String, '/delivery/command', self.on_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_tick = time.monotonic()
        self.create_timer(0.5, self.tick)
        self.get_logger().info('Delivery manager ready. Tables: %s' % ', '.join(self.tables))

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
            self.get_logger().warn('Rejected order for unknown table "%s"' % table)
            return
        order = {'id': next(self.ids), 'table': table, 'items': str(data.get('items', '')),
                 'status': 'queued', 'created': time.time()}
        with self.lock:
            self.queue.append(order)
        self.get_logger().info('Order #%d queued -> %s (%s)' % (order['id'], table, order['items']))

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
        self.get_logger().info('Command: %s' % cmd)

    def on_odom(self, msg):
        self.odom = msg.pose.pose

    # --------------------------------------------------------- battery/status
    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        with self.lock:
            if self.docked:
                self.battery = min(100.0, self.battery + self.charge * dt)
            else:
                self.battery = max(0.0, self.battery - self.drain * dt)
            status = {
                'state': self.state, 'battery': round(self.battery, 1), 'docked': self.docked,
                'current_order': self.current, 'queue': list(self.queue),
                'history': list(self.history), 'tables': self.tables,
            }
        self.status_pub.publish(String(data=json.dumps(status)))
        b = BatteryState()
        b.percentage = self.battery / 100.0
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.docked
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        b.present = True
        self.battery_pub.publish(b)

    def set_state(self, state):
        with self.lock:
            self.state = state
        self.get_logger().info('State -> %s' % state)


class Mission:
    """State machine, run in its own thread because BasicNavigator blocks."""

    def __init__(self, mgr: DeliveryManager):
        self.mgr = mgr
        self.nav = BasicNavigator()

    def pose(self, name):
        x, y, yaw = self.mgr.places[name]
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.nav.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def refresh_localization(self):
        """Re-seed AMCL from the current odometry estimate.

        Workaround for an observed nav2_amcl behavior on this machine where its
        map->odom broadcast can silently stop a few minutes into a run (TF
        lookups then fail with "transform too old"/extrapolation errors even
        though /scan, /odom and /clock keep publishing normally). Republishing
        /initialpose reliably restarts the broadcast. odom and world share the
        same orientation/scale here, so the map-frame estimate is just the
        odom position shifted by the dock's world offset (+9.0, 0.0).
        """
        odom = self.mgr.odom
        if odom is None:
            return
        x = odom.position.x + 9.0
        y = odom.position.y
        ps = PoseWithCovarianceStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.nav.get_clock().now().to_msg()
        ps.pose.pose.position.x, ps.pose.pose.position.y = x, y
        ps.pose.pose.orientation = odom.orientation
        ps.pose.covariance[0] = ps.pose.covariance[7] = 0.1
        ps.pose.covariance[35] = 0.1
        self.nav.initial_pose_pub.publish(ps)

    def go(self, name, retries=2):
        """Navigate to a named place. Returns True on success."""
        for attempt in range(retries + 1):
            if attempt > 0:
                self.refresh_localization()
                time.sleep(1.0)
            self.nav.goToPose(self.pose(name))
            while not self.nav.isTaskComplete():
                if self.mgr.cancel_requested:
                    self.nav.cancelTask()
                time.sleep(0.1)
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                return True
            if self.mgr.cancel_requested:
                return False
            self.mgr.get_logger().warn('Navigation to %s failed (attempt %d)' % (name, attempt + 1))
            self.nav.clearAllCostmaps()
        return False

    def drive_straight(self, distance, speed):
        """Odometry-closed-loop straight motion (negative distance = reverse)."""
        while self.mgr.odom is None:
            time.sleep(0.1)
        start = self.mgr.odom.position
        cmd = Twist()
        cmd.linear.x = math.copysign(speed, distance)
        deadline = time.monotonic() + abs(distance) / speed * 3 + 5
        while time.monotonic() < deadline:
            cur = self.mgr.odom.position
            if math.hypot(cur.x - start.x, cur.y - start.y) >= abs(distance):
                break
            self.mgr.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.mgr.cmd_pub.publish(Twist())

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.mgr.cancel_requested:
            time.sleep(0.1)

    def undock(self):
        if self.mgr.docked:
            self.mgr.set_state(UNDOCKING)
            with self.mgr.lock:
                self.mgr.docked = False
            self.drive_straight(0.6, 0.1)

    def dock(self):
        self.mgr.set_state(TO_DOCK)
        if not self.go('pre_dock', retries=2):
            self.mgr.set_state(IDLE)
            return
        self.mgr.set_state(DOCKING)
        dx = self.mgr.places['pre_dock'][0] - self.mgr.places['dock'][0]
        dy = self.mgr.places['pre_dock'][1] - self.mgr.places['dock'][1]
        self.drive_straight(-math.hypot(dx, dy), 0.08)
        with self.mgr.lock:
            self.mgr.docked = True
            self.mgr.dock_requested = False
        self.mgr.set_state(DOCKED)

    def deliver(self, order):
        m = self.mgr
        order['status'] = 'in_progress'
        with m.lock:
            m.current = order
        steps = [(TO_PICKUP, lambda: self.go('pickup')),
                 (LOADING, lambda: self.wait(m.load_time) or True),
                 (TO_TABLE, lambda: self.go(order['table'])),
                 (SERVING, lambda: self.wait(m.serve_time) or True)]
        ok = True
        for state, step in steps:
            m.set_state(state)
            if m.cancel_requested or not step():
                ok = False
                break
        order['status'] = 'delivered' if ok else ('cancelled' if m.cancel_requested else 'failed')
        order['finished'] = time.time()
        with m.lock:
            m.history.appendleft(order)
            m.current = None
            m.cancel_requested = False
        m.get_logger().info('Order #%d %s' % (order['id'], order['status']))

    def run(self):
        m = self.mgr
        self.nav.setInitialPose(self.pose('dock'))
        self.nav.waitUntilNav2Active()
        m.set_state(DOCKED)
        idle_since = time.monotonic()
        while rclpy.ok():
            with m.lock:
                order = m.queue[0] if m.queue else None
                battery, docked = m.battery, m.docked
            if m.dock_requested and not docked:
                self.dock()
                idle_since = time.monotonic()
            elif order and battery < m.low_battery:
                if docked:
                    if m.state != CHARGING:
                        m.set_state(CHARGING)
                else:
                    m.get_logger().warn('Battery low (%.0f%%): charging before next order' % battery)
                    self.dock()
            elif order:
                with m.lock:
                    m.queue.popleft()
                self.undock()
                self.deliver(order)
                if m.state != DOCKED:
                    m.set_state(IDLE)
                idle_since = time.monotonic()
            elif not docked and time.monotonic() - idle_since > m.idle_timeout:
                self.dock()
            elif docked and m.state == CHARGING:
                m.set_state(DOCKED)
            m.dock_requested = m.dock_requested and not m.docked
            time.sleep(0.2)


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
