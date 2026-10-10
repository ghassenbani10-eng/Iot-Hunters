"""Battery simulator for the TuniBot waiter robot.

A physics-based energy model, integrated on simulation time:

  P_total = P_idle                                  electronics, lidar, computer, screen
          + (m g C_rr |v| + max(0, m a v)) / eta     rolling resistance + acceleration
          + k_turn |w|                              in-place / curve turning losses
          + P_drive (if moving)                      motor controllers enabled

  m = robot mass + payload (payload is added while a tray is loaded, i.e. while
      the delivery manager is in TO_TABLE / SERVING).

The pack is a 7S Li-ion (25.9 V nominal). Terminal voltage = OCV(SoC) - I R_int,
current I = P / V. Charging only happens on the charging pad (the robot pose is
within `dock_tolerance` of the dock pose, i.e. the charging contacts touch)
following a CC-CV profile: constant power up to 80 % SoC, then tapering.

`time_acceleration` multiplies energy flows so a whole battery cycle fits in a
demo (default 25x: a full charge lasts ~20 min of driving instead of ~8 h).
It is reported in every message and in the UI so nothing is hidden.

The node also learns the robot's real consumption per metre while driving
(exponential moving average of measured energy / travelled distance). The
delivery manager uses it to decide, from Nav2 path lengths, whether there is
enough energy to finish a task and still reach the dock.

Interfaces
  sub  /odom                   nav_msgs/Odometry       velocity, distance
  sub  /delivery/status        std_msgs/String (JSON)  payload on board?
  sub  /battery/set_soc        std_msgs/Float32        test hook: force SoC (0-100)
  sub  /battery/charger        std_msgs/Bool           test hook: charger power on/off
  pub  /battery_state          sensor_msgs/BatteryState
  pub  /battery/info           std_msgs/String (JSON)  power, energy/m, time-to-empty ...
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

# Open-circuit voltage of one Li-ion cell vs state of charge (typical NMC curve).
OCV_TABLE = [(0.00, 3.00), (0.05, 3.30), (0.10, 3.45), (0.20, 3.55), (0.30, 3.62),
             (0.40, 3.68), (0.50, 3.74), (0.60, 3.81), (0.70, 3.89), (0.80, 3.97),
             (0.90, 4.06), (1.00, 4.18)]
G = 9.81


def ocv_cell(soc):
    soc = min(1.0, max(0.0, soc))
    for (s0, v0), (s1, v1) in zip(OCV_TABLE, OCV_TABLE[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return OCV_TABLE[-1][1]


class BatterySimulator(Node):

    def __init__(self):
        super().__init__('battery_simulator')
        params = {
            'capacity_ah': 10.0, 'cells_in_series': 7, 'internal_resistance': 0.08,
            'initial_soc': 100.0, 'time_acceleration': 25.0,
            'idle_power_w': 28.0, 'drive_power_w': 12.0, 'mass_kg': 40.0, 'payload_kg': 4.0,
            'rolling_resistance': 0.03, 'drivetrain_efficiency': 0.65, 'turn_power_w_per_rad_s': 18.0,
            'charge_power_w': 240.0, 'cv_start_soc': 80.0,
            'dock_x': 0.0, 'dock_y': 0.0, 'dock_tolerance': 0.15,
            'global_frame': 'map', 'base_frame': 'base_footprint', 'rate_hz': 10.0,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.capacity_ah = float(g('capacity_ah'))
        self.cells = int(g('cells_in_series'))
        self.r_int = float(g('internal_resistance'))
        self.accel = float(g('time_acceleration'))
        self.p_idle, self.p_drive = float(g('idle_power_w')), float(g('drive_power_w'))
        self.mass, self.payload_mass = float(g('mass_kg')), float(g('payload_kg'))
        self.c_rr, self.eta = float(g('rolling_resistance')), float(g('drivetrain_efficiency'))
        self.k_turn = float(g('turn_power_w_per_rad_s'))
        self.p_charge, self.cv_soc = float(g('charge_power_w')), float(g('cv_start_soc')) / 100.0
        self.dock = (float(g('dock_x')), float(g('dock_y')))
        self.dock_tol = float(g('dock_tolerance'))
        self.global_frame, self.base_frame = g('global_frame'), g('base_frame')

        self.nominal_v = 3.7 * self.cells
        self.capacity_wh = self.capacity_ah * self.nominal_v
        self.soc = float(g('initial_soc')) / 100.0
        self.v, self.w, self.prev_v = 0.0, 0.0, 0.0
        self.payload = False
        self.charger_on = True
        self.on_dock = False
        self.power = 0.0              # W, + discharging, - charging (real-time, not accelerated)
        self.power_avg = self.p_idle
        self.energy_used_wh = 0.0     # accelerated, since start
        self.distance_m = 0.0
        # learned consumption while driving (Wh per metre, accelerated). Prior:
        # idle + drive power at 0.35 m/s plus traction.
        self.wh_per_m = (self.p_idle + self.p_drive + self._traction(0.35, 0.0) + 8.0) \
            / 0.35 / 3600.0 * self.accel
        self.depleted = False
        self.last = None

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/delivery/status', self.on_status, 10)
        self.create_subscription(Float32, '/battery/set_soc', self.on_set_soc, 10)
        self.create_subscription(Bool, '/battery/charger', self.on_charger, 10)
        self.state_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.info_pub = self.create_publisher(String, '/battery/info', 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.step)
        self.create_timer(0.5, self.publish)
        self.get_logger().info('Battery: %.0f Wh (%dS %.0f Ah), time acceleration x%.0f' % (
            self.capacity_wh, self.cells, self.capacity_ah, self.accel))

    # ------------------------------------------------------------ inputs
    def on_odom(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def on_status(self, msg):
        try:
            self.payload = json.loads(msg.data).get('state') in ('TO_TABLE', 'SERVING')
        except ValueError:
            pass

    def on_set_soc(self, msg):
        self.soc = min(1.0, max(0.0, msg.data / 100.0))
        self.depleted = self.soc <= 0.0
        self.get_logger().warn('Test hook: SoC forced to %.1f %%' % (self.soc * 100))

    def on_charger(self, msg):
        self.charger_on = bool(msg.data)
        self.get_logger().warn('Test hook: charger %s' % ('ON' if self.charger_on else 'OFF'))

    # ------------------------------------------------------------ model
    def _traction(self, v, a):
        m = self.mass + (self.payload_mass if self.payload else 0.0)
        return (m * G * self.c_rr * abs(v) + max(0.0, m * a * v)) / self.eta

    def _check_dock(self):
        try:
            t = self.tf.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return self.on_dock
        dx = t.transform.translation.x - self.dock[0]
        dy = t.transform.translation.y - self.dock[1]
        return math.hypot(dx, dy) < self.dock_tol and abs(self.v) < 0.05

    def step(self):
        now = self.get_clock().now()
        if self.last is None:
            self.last = now
            return
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if dt <= 0.0 or dt > 1.0:
            return
        a = (self.v - self.prev_v) / dt
        self.prev_v = self.v
        self.on_dock = self._check_dock()
        moving = abs(self.v) > 0.01 or abs(self.w) > 0.02
        p = self.p_idle + (self.p_drive if moving else 0.0) + self._traction(self.v, a) + self.k_turn * abs(self.w)

        charging = self.on_dock and self.charger_on and self.soc < 1.0
        if charging:
            cc = self.p_charge if self.soc < self.cv_soc else \
                self.p_charge * max(0.05, (1.0 - self.soc) / (1.0 - self.cv_soc))
            p -= cc
        self.power = p
        self.power_avg += (p - self.power_avg) * min(1.0, dt / 10.0)
        e_wh = p * dt / 3600.0 * self.accel
        self.soc = min(1.0, max(0.0, self.soc - e_wh / self.capacity_wh))
        if e_wh > 0:
            self.energy_used_wh += e_wh
        # Learn Wh/m while cruising (ignore near-standstill samples).
        ds = abs(self.v) * dt
        self.distance_m += ds
        if abs(self.v) > 0.1 and not charging:
            sample = e_wh / ds
            self.wh_per_m += (sample - self.wh_per_m) * min(1.0, ds / 5.0)
        if self.soc <= 0.0 and not self.depleted:
            self.depleted = True
            self.get_logger().error('BATTERY DEPLETED: the robot would stop here.')
        elif self.soc > 0.02:
            self.depleted = False

    def publish(self):
        ocv = ocv_cell(self.soc) * self.cells
        current = self.power / max(ocv, 1.0)          # A, + discharging
        voltage = ocv - current * self.r_int
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = voltage
        b.current = -current                           # REP: negative while discharging
        b.charge = self.soc * self.capacity_ah
        b.capacity = self.capacity_ah
        b.design_capacity = self.capacity_ah
        b.percentage = self.soc
        b.temperature = float('nan')
        if self.on_dock and self.charger_on:
            b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if self.soc >= 0.999
                                     else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        b.power_supply_health = (BatteryState.POWER_SUPPLY_HEALTH_DEAD if self.depleted
                                 else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        b.location = 'tunibot_base'
        self.state_pub.publish(b)

        drain_w = max(self.power_avg, 1.0) * self.accel   # Wh per real hour at this pace
        info = {
            'soc': round(self.soc * 100, 2), 'voltage': round(voltage, 2), 'current': round(current, 2),
            'power_w': round(self.power, 1), 'power_avg_w': round(self.power_avg, 1),
            'capacity_wh': round(self.capacity_wh, 1), 'energy_left_wh': round(self.soc * self.capacity_wh, 2),
            'wh_per_m': round(self.wh_per_m, 4), 'idle_wh_per_s': round(self.p_idle * self.accel / 3600, 5),
            'time_to_empty_min': round(self.soc * self.capacity_wh / drain_w * 60, 1) if self.power_avg > 0 else None,
            'on_dock': self.on_dock, 'charging': self.on_dock and self.charger_on and self.soc < 1.0,
            'charger_on': self.charger_on, 'payload': self.payload, 'depleted': self.depleted,
            'distance_m': round(self.distance_m, 1), 'energy_used_wh': round(self.energy_used_wh, 2),
            'time_acceleration': self.accel,
        }
        self.info_pub.publish(String(data=json.dumps(info)))


def main():
    rclpy.init()
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
