"""Automated Phase 2 test scenarios with ground-truth metrics.

    ros2 run tunibot_delivery scenario_runner --ros-args -p scenario:=tc01 -p out_dir:=/tmp/results

Run on top of sim.launch.py + navigation.launch.py + delivery.launch.py
(scripts/run_tests.sh starts a fresh stack for every scenario).

Ground truth comes from Gazebo (/gazebo/model_states), never from the robot's
own estimate, so the metrics measure what really happened:
  * min_clearance_m: smallest distance between the robot footprint (0.54 x 0.50 m
    rectangle at its true pose) and any person body (0.22 m radius cylinder);
    <= 0 means a collision with a person
  * contacts: bumper contacts of the robot with anything (walls, furniture, people)
  * delivery outcome, durations, path length, battery curve, state/event timeline

Output per scenario: <out_dir>/<id>.json (summary + pass/fail), <id>.csv
(10 Hz time series), <id>_events.json and <id>.png (trajectory over the map,
people tracks, battery curve).
"""
import csv
import json
import math
import os
import threading
import time

import rclpy
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

DOCK_WORLD = (-9.0, 0.0)          # map = world - DOCK_WORLD
HALF_L, HALF_W = 0.27, 0.25       # robot footprint half sizes
BODY_R = 0.22


def rect_circle_distance(rx, ry, ryaw, cx, cy, r):
    """Distance from an oriented rectangle (robot) to a circle (person body)."""
    c, s = math.cos(ryaw), math.sin(ryaw)
    lx = c * (cx - rx) + s * (cy - ry)
    ly = -s * (cx - rx) + c * (cy - ry)
    dx = max(abs(lx) - HALF_L, 0.0)
    dy = max(abs(ly) - HALF_W, 0.0)
    return math.hypot(dx, dy) - r


def person_sdf(name, waypoints, speed=0.8, dwell=1.0, start_delay=0.0, max_yield=6.0, ping_pong=True):
    """An actor + collision body, same model as the world's people (see build_world.py)."""
    wps = ''.join('<waypoint>%.2f %.2f</waypoint>' % w for w in waypoints)
    x0, y0 = waypoints[0]
    body = f"""<?xml version='1.0'?><sdf version='1.7'><model name='{name}'>
<pose>{x0} {y0} 0 0 0 0</pose><link name='body'><gravity>0</gravity><kinematic>1</kinematic>
<inertial><mass>70</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='body_collision'><pose>0 0 0.85 0 0 0</pose><geometry><cylinder><radius>{BODY_R}</radius><length>1.7</length></cylinder></geometry></collision>
</link></model></sdf>"""
    actor = f"""<?xml version='1.0'?><sdf version='1.7'><actor name='{name}_actor'>
<pose>{x0} {y0} 1.0 1.5708 0 0</pose>
<skin><filename>walk.dae</filename><scale>1.0</scale></skin>
<animation name='walking'><filename>walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
<plugin name='{name}_walker' filename='libtunibot_human_actor.so'>{wps}<speed>{speed}</speed><dwell>{dwell}</dwell>
<start_delay>{start_delay}</start_delay><body>{name}</body><robot>waiter_robot</robot><yield_distance>0.9</yield_distance>
<max_yield_time>{max_yield}</max_yield_time><ping_pong>{'true' if ping_pong else 'false'}</ping_pong></plugin>
</actor></sdf>"""
    return body, actor


def box_sdf(name, x, y, sx, sy, sz, yaw=0.0):
    geo = '<geometry><box><size>%.2f %.2f %.2f</size></box></geometry>' % (sx, sy, sz)
    return f"""<?xml version='1.0'?><sdf version='1.7'><model name='{name}'><static>1</static>
<pose>{x} {y} {sz / 2} 0 0 {yaw}</pose><link name='link'><collision name='c'>{geo}</collision>
<visual name='v'>{geo}<material><script><uri>file://media/materials/scripts/gazebo.material</uri>
<name>Gazebo/Orange</name></script></material></visual></link></model></sdf>"""


class Runner(Node):

    def __init__(self):
        super().__init__('scenario_runner')
        self.declare_parameter('scenario', 'tc01')
        self.declare_parameter('out_dir', os.path.expanduser('~/tunibot_test_results'))
        self.scenario = self.get_parameter('scenario').value
        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.status, self.battery, self.people_est = {}, {}, {}
        self.truth = {}
        self.events = []
        self.samples = []
        self.contacts = []
        self.min_clear = float('inf')
        self.min_clear_info = None
        self.min_soc = 100.0
        self.path_len = 0.0
        self.last_xy = None
        self.lock = threading.Lock()

        self.order_pub = self.create_publisher(String, '/delivery/order', 10)
        self.cmd_pub = self.create_publisher(String, '/delivery/command', 10)
        self.soc_pub = self.create_publisher(Float32, '/battery/set_soc', 10)
        self.charger_pub = self.create_publisher(Bool, '/battery/charger', 10)
        self.create_subscription(String, '/delivery/status', self.on_status, 10)
        self.create_subscription(String, '/delivery/events', self.on_event, 50)
        self.create_subscription(String, '/battery/info', lambda m: setattr(self, 'battery', json.loads(m.data)), 10)
        self.create_subscription(String, '/people', lambda m: setattr(self, 'people_est', json.loads(m.data)), 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.create_subscription(ContactsState, '/bumper', self.on_bumper, 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.create_timer(0.1, self.sample)
        self.t0 = None

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_event(self, msg):
        ev = json.loads(msg.data)
        ev['t_test'] = round(self.now() - (self.t0 or self.now()), 1)
        with self.lock:
            self.events.append(ev)
        self.get_logger().info('EVENT %s: %s' % (ev['kind'], ev['text']))

    def on_models(self, msg):
        truth = {}
        for name, p in zip(msg.name, msg.pose):
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            truth[name] = (p.position.x, p.position.y, yaw)
        self.truth = truth

    def on_bumper(self, msg):
        for st in msg.states:
            other = st.collision2_name if 'waiter_robot' in st.collision1_name else st.collision1_name
            if other.startswith('ground_plane'):
                continue
            with self.lock:
                if not self.contacts or self.now() - self.contacts[-1]['t'] > 1.0 or \
                        self.contacts[-1]['with'] != other:
                    self.contacts.append({'t': self.now(), 't_test': round(self.now() - (self.t0 or self.now()), 1),
                                          'with': other, 'state': self.status.get('state')})
                    self.get_logger().error('CONTACT with %s' % other)

    def people_truth(self):
        return {n: p for n, p in self.truth.items()
                if (n.startswith('human_') or n.startswith('tc_person')) and not n.endswith('_actor')}

    def sample(self):
        if self.t0 is None or 'waiter_robot' not in self.truth:
            return
        rx, ry, ryaw = self.truth['waiter_robot']
        clear, who = float('inf'), None
        for n, (px, py, _) in self.people_truth().items():
            d = rect_circle_distance(rx, ry, ryaw, px, py, BODY_R)
            if d < clear:
                clear, who = d, n
        soc = self.battery.get('soc', float('nan'))
        if self.last_xy is not None:
            self.path_len += math.hypot(rx - self.last_xy[0], ry - self.last_xy[1])
        self.last_xy = (rx, ry)
        moving = self.battery.get('power_w', 0) is not None
        row = {'t': round(self.now() - self.t0, 2), 'x': round(rx - DOCK_WORLD[0], 3),
               'y': round(ry - DOCK_WORLD[1], 3), 'yaw': round(ryaw, 3),
               'state': self.status.get('state', ''), 'nav': self.status.get('nav_status', ''),
               'soc': soc, 'power_w': self.battery.get('power_w'),
               'clearance': round(clear, 3) if clear < 99 else None, 'nearest_person': who,
               'people_tracked': self.people_est.get('count', 0)}
        for n, (px, py, _) in self.people_truth().items():
            row['p_' + n] = '%.2f;%.2f' % (px - DOCK_WORLD[0], py - DOCK_WORLD[1])
        del moving
        with self.lock:
            self.samples.append(row)
            if clear < self.min_clear:
                self.min_clear = clear
                self.min_clear_info = {'t': row['t'], 'person': who, 'state': row['state'],
                                       'robot': (row['x'], row['y'])}
            if isinstance(soc, (int, float)) and not math.isnan(soc):
                self.min_soc = min(self.min_soc, soc)

    # ---------------------------------------------------------------- actions
    def order(self, table, items='test', vip=False):
        self.order_pub.publish(String(data=json.dumps(
            {'table': table, 'items': items, 'priority': 'vip' if vip else 'normal'})))

    def command(self, cmd):
        self.cmd_pub.publish(String(data=cmd))

    def set_soc(self, soc):
        self.soc_pub.publish(Float32(data=float(soc)))

    def charger(self, on):
        self.charger_pub.publish(Bool(data=bool(on)))

    def spawn(self, name, xml):
        self.spawn_cli.wait_for_service(timeout_sec=10.0)
        req = SpawnEntity.Request()
        req.name, req.xml = name, xml
        fut = self.spawn_cli.call_async(req)
        self.wait_until(lambda: fut.done(), 15.0)
        ok = fut.done() and fut.result().success
        self.get_logger().info('spawn %s: %s' % (name, 'ok' if ok else 'FAILED'))
        return ok

    def spawn_person(self, name, waypoints, **kw):
        body, actor = person_sdf(name, [(x + DOCK_WORLD[0], y + DOCK_WORLD[1]) for x, y in waypoints], **kw)
        return self.spawn(name, body) and self.spawn(name + '_actor', actor)

    def spawn_box(self, name, x, y, sx, sy, sz=1.0, yaw=0.0):
        """x, y in the map frame."""
        return self.spawn(name, box_sdf(name, x + DOCK_WORLD[0], y + DOCK_WORLD[1], sx, sy, sz, yaw))

    def delete(self, name):
        req = DeleteEntity.Request()
        req.name = name
        fut = self.delete_cli.call_async(req)
        self.wait_until(lambda: fut.done(), 10.0)

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.1)
        return False

    def wait_sim(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            time.sleep(0.05)

    def wait_state(self, states, timeout):
        states = (states,) if isinstance(states, str) else tuple(states)
        return self.wait_until(lambda: self.status.get('state') in states, timeout)

    def robot_map(self):
        if 'waiter_robot' not in self.truth:
            return None
        x, y, yaw = self.truth['waiter_robot']
        return x - DOCK_WORLD[0], y - DOCK_WORLD[1], yaw

    def history(self):
        return {o['id']: o for o in self.status.get('history', [])}

    def wait_orders_done(self, n, timeout):
        """Wait until n orders are finished (delivered/failed/cancelled) and the queue is empty."""
        return self.wait_until(lambda: len(self.status.get('history', [])) >= n and
                               not self.status.get('queue') and self.status.get('current_order') is None,
                               timeout)

    # ---------------------------------------------------------------- report
    def save(self, result):
        sid = self.scenario
        with self.lock:
            samples, events, contacts = list(self.samples), list(self.events), list(self.contacts)
        hist = self.status.get('history', [])
        result.update({
            'scenario': sid, 'duration_s': samples[-1]['t'] if samples else 0,
            'min_clearance_m': round(self.min_clear, 3) if self.min_clear < 99 else None,
            'min_clearance_at': self.min_clear_info,
            'person_collisions': sum(1 for c in contacts if 'human' in c['with'] or 'tc_person' in c['with']),
            'contacts': contacts, 'min_soc': round(self.min_soc, 1),
            'final_soc': self.battery.get('soc'), 'path_length_m': round(self.path_len, 1),
            'orders': [{k: o.get(k) for k in ('id', 'table', 'status', 'attempts', 'notes', 'priority')}
                       for o in hist],
            'final_state': self.status.get('state'),
            'learned_wh_per_m': self.battery.get('wh_per_m'),
        })
        result['passed'] = bool(result.get('passed')) and result['person_collisions'] == 0 and not contacts
        with open(os.path.join(self.out_dir, sid + '.json'), 'w') as f:
            json.dump(result, f, indent=2)
        with open(os.path.join(self.out_dir, sid + '_events.json'), 'w') as f:
            json.dump(events, f, indent=1)
        if samples:
            keys = sorted({k for s in samples for k in s}, key=lambda k: (k.startswith('p_'), k))
            with open(os.path.join(self.out_dir, sid + '.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(samples)
        try:
            from tunibot_delivery.test_plots import plot_scenario
            plot_scenario(self.out_dir, sid, result, samples, events)
        except Exception as ex:  # noqa: BLE001 - plotting must never fail a test
            self.get_logger().warn('plot failed: %s' % ex)
        self.get_logger().info('RESULT %s: %s' % (sid, 'PASS' if result['passed'] else 'FAIL'))
        self.get_logger().info(json.dumps({k: result[k] for k in (
            'passed', 'min_clearance_m', 'person_collisions', 'min_soc', 'duration_s')}))


# ====================================================================== scenarios
# Each scenario returns a dict with at least 'title', 'passed', 'criteria', 'measured'.

def delivered(r, oid):
    return r.history().get(oid, {}).get('status') == 'delivered'


def tc01(r):
    """Delivery through the dining room with the 4 ambient walking people."""
    r.order('table_6', 'TC01: 2 coffees')
    done = r.wait_orders_done(1, 600) and r.wait_state('DOCKED', 400)
    ok = done and delivered(r, 1)
    return {'title': 'Delivery with people walking around (ambient crowd)', 'passed': ok,
            'criteria': 'order delivered, robot back on the dock, 0 collisions, min clearance > 0.10 m',
            'measured': {'delivered': ok}, 'clearance_threshold': 0.10}


def tc02(r):
    """A person walks head-on towards the robot in the dining room."""
    r.order('table_5', 'TC02: 1 tea')
    r.wait_state('TO_TABLE', 300)
    # robot leaves the kitchen door (map ~(5, 3)) heading south: spawn a person ahead walking north at it
    r.wait_until(lambda: (r.robot_map() or (0, 9))[1] < 2.4, 120)
    x, y, _ = r.robot_map()
    r.spawn_person('tc_person_headon', [(x + 0.4, y - 4.5), (x + 0.4, y + 0.5), (x - 3.0, y + 0.5)],
                   speed=0.9, dwell=30.0, ping_pong=False)
    done = r.wait_orders_done(1, 500)
    ok = done and delivered(r, 1)
    return {'title': 'Person walking head-on towards the robot', 'passed': ok,
            'criteria': 'robot slows/stops or swerves, order delivered, 0 collisions, clearance > 0.10 m',
            'measured': {'delivered': ok}, 'clearance_threshold': 0.10}


def tc03(r):
    """A person crosses right in front of the moving robot."""
    r.order('table_4', 'TC03: 3 waters')
    r.wait_state('TO_TABLE', 300)
    r.wait_until(lambda: (r.robot_map() or (0, 9))[1] < 1.8, 120)
    x, y, _ = r.robot_map()
    # crossing from the robot's left to its right ~1.8 m ahead (robot goes roughly south)
    r.spawn_person('tc_person_cross', [(x + 2.8, y - 1.8), (x - 3.5, y - 1.8)], speed=1.0, dwell=60.0,
                   ping_pong=False)
    done = r.wait_orders_done(1, 500)
    ok = done and delivered(r, 1)
    return {'title': 'Person crossing just in front of the robot', 'passed': ok,
            'criteria': 'robot yields (stops/slows) and continues, order delivered, 0 collisions',
            'measured': {'delivered': ok}, 'clearance_threshold': 0.10}


def tc04(r):
    """A person stands in the kitchen doorway for 25 s, then walks away."""
    r.order('table_7', 'TC04: 1 juice')
    r.wait_state(('UNDOCKING', 'TO_PICKUP'), 120)
    t_spawn = r.now()
    # person standing in door D1 (map (5.0, 3.0)), leaves after 25 s towards the dining room
    r.spawn_person('tc_person_door', [(5.0, 3.0), (2.0, 1.6)], speed=0.7, dwell=100.0,
                   start_delay=t_spawn + 25.0, ping_pong=False)
    blocked = r.wait_until(lambda: any('Blocked' in e['text'] or 'recovery' == e['kind'] for e in r.events), 120)
    done = r.wait_orders_done(1, 600)
    ok = done and delivered(r, 1)
    return {'title': 'Doorway blocked by a standing person (25 s)', 'passed': ok,
            'criteria': 'robot detects the blockage, waits/recovers without colliding, goes through once '
                        'the person leaves, order delivered',
            'measured': {'delivered': ok, 'blocked_detected': blocked}, 'clearance_threshold': 0.05}


def tc05(r):
    """VIP lounge delivery: 3 doorways (0.9 m), 1.05 m corridor and an L-turn."""
    r.order('table_8', 'TC05: VIP champagne', vip=True)
    done = r.wait_orders_done(1, 700) and r.wait_state('DOCKED', 500)
    ok = done and delivered(r, 1)
    return {'title': 'VIP lounge: doorways + narrow corridor + L-turn', 'passed': ok,
            'criteria': 'order to table_8 delivered through D2, the 1.05 m corridor, the L-turn and D3, '
                        'robot back on the dock, 0 contacts',
            'measured': {'delivered': ok}, 'clearance_threshold': 0.05}


def tc06(r):
    """The primary approach pose of table_5 is blocked by a trolley: alternate approach used."""
    r.spawn_box('tc_trolley', 9.40, -3.77, 0.7, 0.5, 0.9)
    r.order('table_5', 'TC06: 2 sodas')
    done = r.wait_orders_done(1, 700)
    ok = done and delivered(r, 1)
    alt = any('approach 1/2' in e['text'] for e in r.events)
    return {'title': 'Table approach blocked by a trolley -> alternate approach pose', 'passed': ok and alt,
            'criteria': 'first approach fails, robot uses the second approach pose, order delivered',
            'measured': {'delivered': ok, 'first_approach_failed': alt}, 'clearance_threshold': 0.05}


def tc07(r):
    """The service corridor is fully blocked: VIP order fails gracefully, next order still served."""
    r.spawn_box('tc_corridor_block', 16.70, 6.0, 1.0, 0.6, 1.2)
    r.order('table_9', 'TC07: blocked VIP order', vip=False)
    r.order('table_7', 'TC07: next order')
    done = r.wait_orders_done(2, 900) and r.wait_state('DOCKED', 400)
    h = r.history()
    ok = done and h.get(1, {}).get('status') == 'failed' and h.get(2, {}).get('status') == 'delivered'
    return {'title': 'Unreachable table (corridor blocked): graceful failure, queue continues', 'passed': ok,
            'criteria': 'order 1 reported failed (unreachable) after the recoveries, order 2 delivered, '
                        'robot returns to dock (no deadlock)',
            'measured': {'order1': h.get(1, {}).get('status'), 'order2': h.get(2, {}).get('status'),
                         'notes': h.get(1, {}).get('notes')}, 'clearance_threshold': 0.05}


def tc08(r):
    """Battery too low for the order at request time: charge first, only as needed."""
    r.set_soc(18.0)
    r.wait_sim(2.0)
    r.order('table_3', 'TC08: low battery at order time')
    charged = r.wait_state('CHARGING', 60)
    soc_at_departure = None
    if r.wait_state('UNDOCKING', 600):
        soc_at_departure = r.battery.get('soc')
    done = r.wait_orders_done(1, 600)
    ok = charged and done and delivered(r, 1)
    return {'title': 'Low battery when the order arrives -> charge first', 'passed': ok and r.min_soc > 0,
            'criteria': 'robot does not leave at 18 %, charges until the energy needed for the whole '
                        'order + reserve is available, then delivers; SoC never reaches 0',
            'measured': {'charged_first': charged, 'soc_at_departure': soc_at_departure,
                         'need_soc': r.status.get('energy', {}).get('need_soc')},
            'clearance_threshold': 0.05}


def tc09(r):
    """Battery drains during the mission: return to dock before it dies, order resumed after charging."""
    r.order('table_9', 'TC09: VIP delivery, battery failing')
    r.wait_state('TO_TABLE', 400)
    r.wait_sim(5.0)
    r.set_soc(16.0)                    # sudden drop (old cells / heavy load)
    returned = r.wait_until(lambda: any(e['kind'] == 'battery' and 'interrupted' in e['text']
                                        for e in r.events), 120)
    docked = r.wait_state(('DOCKED', 'CHARGING'), 400)
    soc_at_dock = r.battery.get('soc')
    done = r.wait_orders_done(1, 900)
    ok = returned and docked and done and delivered(r, 1)
    return {'title': 'Battery runs low mid-mission -> return to dock in time, resume later',
            'passed': ok and r.min_soc > 0,
            'criteria': 'energy monitor interrupts the mission, robot docks with SoC > 0 (reserve kept), '
                        'charges, then the re-queued order is delivered',
            'measured': {'interrupted': returned, 'docked': docked, 'soc_at_dock': soc_at_dock,
                         'delivered_after_charge': ok}, 'clearance_threshold': 0.05}


def tc10(r):
    """Endurance: 8 orders back to back, battery cycling handled autonomously."""
    tables = ['table_1', 'table_4', 'table_8', 'table_3', 'table_6', 'table_2', 'table_9', 'table_5']
    r.set_soc(55.0)
    for i, t in enumerate(tables):
        r.order(t, 'TC10 order %d' % (i + 1))
    done = r.wait_orders_done(len(tables), 3600)
    delivered_n = sum(1 for o in r.history().values() if o['status'] == 'delivered')
    charges = sum(1 for e in r.events if e['kind'] == 'battery' and 'Charging' in e['text'])
    ok = done and delivered_n == len(tables)
    return {'title': 'Endurance: 8 orders, battery cycles handled autonomously', 'passed': ok and r.min_soc > 0,
            'criteria': 'all 8 orders delivered, robot charges by itself when needed, SoC never 0',
            'measured': {'delivered': delivered_n, 'of': len(tables), 'charging_sessions': charges},
            'clearance_threshold': 0.05}


def tc11(r):
    """Emergency stop while driving, then resume."""
    r.order('table_2', 'TC11: e-stop test')
    r.wait_state('TO_TABLE', 300)
    r.wait_sim(4.0)
    r.command('estop')
    t_cmd = r.now()
    stopped = r.wait_until(lambda: r.status.get('state') == 'EMERGENCY_STOP', 10)
    r.wait_sim(2.0)
    rows = [s for s in r.samples if s['t'] >= t_cmd - r.t0 + 1.0]
    still = len(rows) > 3 and math.hypot(rows[-1]['x'] - rows[0]['x'], rows[-1]['y'] - rows[0]['y']) < 0.05
    r.wait_sim(5.0)
    r.command('resume')
    done = r.wait_orders_done(1, 600)
    ok = stopped and still and done and delivered(r, 1)
    return {'title': 'Emergency stop and resume', 'passed': ok,
            'criteria': 'robot stops within 1 s and stays still, resumes and delivers the re-queued order',
            'measured': {'estop_state': stopped, 'stationary_during_estop': still, 'delivered': ok},
            'clearance_threshold': 0.05}


def tc12(r):
    """Docking precision and charger-contact check over 3 docking cycles."""
    errors = []
    for i, t in enumerate(['table_7', 'table_4', 'table_1']):
        r.order(t, 'TC12 docking %d' % (i + 1))
        r.wait_until(lambda: len(r.status.get('history', [])) >= i + 1, 600)
        r.command('dock')
        r.wait_until(lambda: r.status.get('state') == 'DOCKED' and r.status.get('docked'), 400)
        r.wait_sim(3.0)
        x, y, yaw = r.robot_map()
        errors.append({'dx': round(x, 3), 'dy': round(y, 3), 'dyaw_deg': round(math.degrees(yaw), 1),
                       'charging': r.battery.get('charging') or r.battery.get('on_dock')})
    ok = all(abs(e['dx']) < 0.06 and abs(e['dy']) < 0.06 and abs(e['dyaw_deg']) < 5 and e['charging']
             for e in errors)
    return {'title': 'Docking precision (3 cycles) and charger contact', 'passed': ok,
            'criteria': 'final pose within 6 cm / 5 deg of the charging pad, charging confirmed each time',
            'measured': {'dockings': errors}, 'clearance_threshold': 0.05}


SCENARIOS = {f.__name__: f for f in (tc01, tc02, tc03, tc04, tc05, tc06, tc07, tc08, tc09, tc10, tc11, tc12)}


def main():
    rclpy.init()
    r = Runner()
    ex = MultiThreadedExecutor()
    ex.add_node(r)
    threading.Thread(target=ex.spin, daemon=True).start()
    fn = SCENARIOS[r.scenario]
    r.get_logger().info('Waiting for the delivery manager...')
    r.wait_until(lambda: r.status.get('state') == 'DOCKED' and 'waiter_robot' in r.truth, 300)
    r.wait_sim(3.0)
    r.t0 = r.now()
    r.get_logger().info('=== %s: %s' % (r.scenario, fn.__doc__.strip()))
    try:
        result = fn(r)
    except Exception as ex:  # noqa: BLE001
        result = {'title': fn.__doc__.strip(), 'passed': False, 'error': repr(ex)}
    result.setdefault('description', fn.__doc__.strip())
    thr = result.get('clearance_threshold', 0.05)
    if r.min_clear < 99 and r.min_clear < thr:
        result['passed'] = False
        result['failure'] = 'min clearance %.3f m below %.2f m' % (r.min_clear, thr)
    r.save(result)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
