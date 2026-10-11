"""People (dynamic obstacle) tracker, motion predictor and social speed governor.

Pipeline, on every lidar scan:
  1. scan points -> map frame; points closer than `static_margin` to an occupied
     cell of the static map are background (walls, furniture) and dropped;
  2. the remaining points are clustered (euclidean, `cluster_gap`); clusters of
     person size become detections (centroid pushed back by ~ the body radius,
     because the lidar only sees the near side of a person);
  3. a multi-target tracker (constant-velocity Kalman filter per track,
     nearest-neighbour gating) estimates position and velocity; a track counts
     as a person once it has walked 0.6 m (furniture never does);
  4. outputs:
     /people                 std_msgs/String JSON: tracks, count, nearest distance
     /people/cost_mask       nav_msgs/OccupancyGrid: cost mask of where each moving
                             person will be in the next `horizon` seconds (cost fading
                             with time). It is consumed by a Nav2 KeepoutFilter
                             ("people_filter") in both costmaps, so the planner routes
                             *behind* walking people and the controller slows down /
                             stops before a predicted crossing. Each new mask replaces
                             the previous one: no ghost obstacles.
                             Around every confirmed person the mask also holds a
                             personal-space cost: an asymmetric Gaussian (Kirby et al.
                             2009, "COMPANION"; the layered social costmap of Lu et al.
                             2014) elongated in the walking direction, capped below
                             lethal so that it shapes the path but never blocks it.
     /people/filter_info     nav2_msgs/CostmapFilterInfo for that filter
     /people/markers         visualization_msgs/MarkerArray (RViz)
     /speed_limit            nav2_msgs/SpeedLimit: social speed governor = min of
                               - people proximity (slow near people, crawl very close)
                               - narrow-passage zones (doorways, corridors)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav2_msgs.msg import CostmapFilterInfo, SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class Track:
    _next = 1

    def __init__(self, z, t):
        self.id = Track._next
        Track._next += 1
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([0.05, 0.05, 1.0, 1.0])
        self.hits, self.last_seen, self.t = 1, t, t
        self.origin = (z[0], z[1])
        self.walked = False      # becomes a person once it has really moved

    def predict(self, t, q=1.5):
        dt = max(0.0, t - self.t)
        self.t = t
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q

    def update(self, z, t, r=0.04 ** 2 * 4):
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        S = H @ self.P @ H.T + np.eye(2) * r
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z) - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        self.hits += 1
        self.last_seen = t
        if math.hypot(self.x[0] - self.origin[0], self.x[1] - self.origin[1]) > 0.6:
            self.walked = True

    @property
    def speed(self):
        return math.hypot(self.x[2], self.x[3])


class PeopleTracker(Node):

    def __init__(self):
        super().__init__('people_tracker')
        params = {
            'static_margin': 0.3, 'cluster_gap': 0.25, 'min_points': 3, 'max_cluster_size': 0.9,
            'body_radius': 0.22, 'gate': 0.8, 'confirm_hits': 3, 'track_timeout': 1.0,
            'max_range': 8.0, 'horizon': 2.0, 'horizon_step': 0.25, 'moving_speed': 0.25,
            'mask_resolution': 0.1, 'mask_publish_hz': 5.0,
            # personal space (OccupancyGrid units 0..100, 100 = lethal)
            'proxemics_amplitude': 60.0, 'proxemics_sigma_side': 0.45, 'proxemics_sigma_rear': 0.35,
            'proxemics_sigma_front': 0.5, 'proxemics_front_per_speed': 0.8,
            'near_distance': 1.2, 'near_speed_pct': 45.0, 'close_distance': 2.2, 'close_speed_pct': 70.0,
            'zone_speed_pct': 60.0,
            # narrow-passage zones, map frame, flat list of [xmin, xmax, ymin, ymax] * n
            'slow_zones': [0.0, 0.0, 0.0, 0.0],
            'global_frame': 'map', 'robot_frame': 'base_footprint',
        }
        for k, v in params.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in params}
        z = [float(v) for v in self.p['slow_zones']]
        self.zones = [z[i:i + 4] for i in range(0, len(z) - 3, 4) if z[i + 1] > z[i]]

        self.static = None       # (dist_to_occupied grid, origin x, origin y, res)
        self.tracks = []
        self.tf = Buffer()
        # TF is received by a helper node spun in its own thread: on_scan waits for the
        # transform at the scan time, which could never arrive if the listener shared
        # this node's (single-threaded) executor.
        self.tf_node = rclpy.create_node('people_tracker_tf', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.tf_listener = TransformListener(self.tf, self.tf_node, spin_thread=True)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.people_pub = self.create_publisher(String, '/people', 10)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/people/cost_mask', LATCHED)
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/people/filter_info', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/people/markers', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.last_mask = 0.0
        self.last_mask_digest = None
        self.last_speed = None
        self.last_speed_pub = 0.0
        self.create_timer(1.0 / float(self.p['mask_publish_hz']), self.publish_outputs)
        info = CostmapFilterInfo()
        info.header.frame_id = self.p['global_frame']
        info.type = 0                       # keepout-type filter (graded cost)
        info.filter_mask_topic = '/people/cost_mask'
        info.base, info.multiplier = 0.0, 1.0
        self.info_pub.publish(info)
        self.get_logger().info('People tracker ready, %d slow zones' % len(self.zones))

    # ------------------------------------------------------------- inputs
    def on_map(self, msg):
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        occ = np.array(msg.data, dtype=np.int16).reshape(h, w) > 50
        dist = distance_transform_edt(~occ) * res
        self.static = (dist, msg.info.origin.position.x, msg.info.origin.position.y, res)
        self.mask_info = msg.info
        self.get_logger().info('Static map received (%dx%d)' % (w, h))

    def is_static(self, x, y):
        dist, ox, oy, res = self.static
        i = np.floor((y - oy) / res).astype(int)
        j = np.floor((x - ox) / res).astype(int)
        inside = (i >= 0) & (i < dist.shape[0]) & (j >= 0) & (j < dist.shape[1])
        out = np.ones_like(x, dtype=bool)
        out[inside] = dist[i[inside], j[inside]] < self.p['static_margin']
        return out

    def on_scan(self, scan):
        if self.static is None:
            return
        try:
            tf = self.tf.lookup_transform(self.p['global_frame'], scan.header.frame_id,
                                          Time.from_msg(scan.header.stamp), Duration(seconds=0.2))
        except TransformException:
            return
        t = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, self.p['max_range']))
        r, a = r[ok], a[ok]
        tr, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sx, sy = tr.x, tr.y
        px = sx + r * np.cos(a + yaw)
        py = sy + r * np.sin(a + yaw)
        dyn = ~self.is_static(px, py)
        detections = self.cluster(px[dyn], py[dyn], sx, sy)
        # a person's centre cannot be inside ~0.45 m of the robot centre (body radius
        # 0.22 + robot half-width 0.25): such clusters are artefacts, not people
        robot = self.robot_xy()
        if robot is not None:
            detections = [d for d in detections if math.hypot(d[0] - robot[0], d[1] - robot[1]) > 0.45]
        self.track(detections, t)

    def cluster(self, xs, ys, sx, sy):
        dets = []
        if len(xs) == 0:
            return dets
        gap = self.p['cluster_gap']
        start = 0
        for k in range(1, len(xs) + 1):
            if k == len(xs) or math.hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1]) > gap:
                cx, cy = xs[start:k], ys[start:k]
                size = math.hypot(cx.max() - cx.min(), cy.max() - cy.min())
                if k - start >= self.p['min_points'] and size < self.p['max_cluster_size']:
                    mx, my = cx.mean(), cy.mean()
                    d = math.hypot(mx - sx, my - sy)
                    push = self.p['body_radius'] * 0.6
                    dets.append((mx + (mx - sx) / d * push, my + (my - sy) / d * push))
                start = k
        return dets

    def track(self, dets, t):
        for tr in self.tracks:
            tr.predict(t)
        unused = list(range(len(dets)))
        pairs = sorted(((math.hypot(tr.x[0] - d[0], tr.x[1] - d[1]), ti, di)
                        for ti, tr in enumerate(self.tracks) for di, d in enumerate(dets)))
        used_t = set()
        for dist, ti, di in pairs:
            if dist > self.p['gate'] or ti in used_t or di not in unused:
                continue
            self.tracks[ti].update(dets[di], t)
            used_t.add(ti)
            unused.remove(di)
        for di in unused:
            self.tracks.append(Track(dets[di], t))
        self.tracks = [tr for tr in self.tracks if t - tr.last_seen < self.p['track_timeout']]
        self.now_t = t

    # ------------------------------------------------------------- outputs
    def confirmed(self):
        """Tracks that are people: seen several times and have walked at least 0.6 m.
        Clusters that never move (furniture edges, a trolley...) stay plain obstacles
        for the costmaps; they get no motion prediction and no social speed limit."""
        return [tr for tr in self.tracks if tr.hits >= self.p['confirm_hits'] and tr.walked]

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform(self.p['global_frame'], self.p['robot_frame'], Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def publish_outputs(self):
        if self.static is None:
            return
        people = self.confirmed()
        robot = self.robot_xy()
        nearest = None
        out = []
        for tr in people:
            d = math.hypot(tr.x[0] - robot[0], tr.x[1] - robot[1]) if robot else None
            if d is not None and (nearest is None or d < nearest):
                nearest = d
            out.append({'id': tr.id, 'x': round(tr.x[0], 2), 'y': round(tr.x[1], 2),
                        'vx': round(tr.x[2], 2), 'vy': round(tr.x[3], 2), 'speed': round(tr.speed, 2),
                        'distance': round(d, 2) if d is not None else None})
        self.people_pub.publish(String(data=json.dumps(
            {'count': len(out), 'nearest': round(nearest, 2) if nearest is not None else None, 'tracks': out})))
        self.publish_mask(people, robot)
        self.publish_markers(people)
        self.publish_speed(nearest, robot)

    def publish_mask(self, people, robot):
        info = self.mask_info
        res = self.p['mask_resolution']
        scale = info.resolution / res
        w, h = int(info.width * scale), int(info.height * scale)
        ox, oy = info.origin.position.x, info.origin.position.y
        mask = np.zeros((h, w), dtype=np.int8)
        yy, xx = np.mgrid[0:h, 0:w]
        cx_all = ox + (xx + 0.5) * res
        cy_all = oy + (yy + 0.5) * res

        def stamp(x, y, radius, cost):
            i0, i1 = int((y - radius - oy) / res), int((y + radius - oy) / res) + 1
            j0, j1 = int((x - radius - ox) / res), int((x + radius - ox) / res) + 1
            i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, h), min(j1, w)
            if i0 >= i1 or j0 >= j1:
                return
            sub_x, sub_y = cx_all[i0:i1, j0:j1], cy_all[i0:i1, j0:j1]
            inside = (sub_x - x) ** 2 + (sub_y - y) ** 2 <= radius ** 2
            if robot is not None:   # never paint over the robot itself
                inside &= (sub_x - robot[0]) ** 2 + (sub_y - robot[1]) ** 2 > 0.45 ** 2
            region = mask[i0:i1, j0:j1]
            region[inside] = np.maximum(region[inside], cost)

        def personal_space(tr):
            """Asymmetric Gaussian: sigma_front grows with walking speed, sigma_rear is
            short, sigma_side constant. Standing people get a round one."""
            x, y, vx, vy = tr.x
            v = tr.speed
            heading = math.atan2(vy, vx) if v >= self.p['moving_speed'] else 0.0
            s_side = self.p['proxemics_sigma_side']
            s_front = (self.p['proxemics_sigma_front'] + self.p['proxemics_front_per_speed'] * v
                       if v >= self.p['moving_speed'] else s_side)
            s_rear = self.p['proxemics_sigma_rear'] if v >= self.p['moving_speed'] else s_side
            r = 3.0 * max(s_front, s_side)
            i0, i1 = max(int((y - r - oy) / res), 0), min(int((y + r - oy) / res) + 1, h)
            j0, j1 = max(int((x - r - ox) / res), 0), min(int((x + r - ox) / res) + 1, w)
            if i0 >= i1 or j0 >= j1:
                return
            dx, dy = cx_all[i0:i1, j0:j1] - x, cy_all[i0:i1, j0:j1] - y
            c, s_ = math.cos(heading), math.sin(heading)
            along, across = c * dx + s_ * dy, -s_ * dx + c * dy
            s_al = np.where(along >= 0.0, s_front, s_rear)
            cost = self.p['proxemics_amplitude'] * np.exp(-0.5 * ((along / s_al) ** 2 + (across / s_side) ** 2))
            cost = np.where(cost >= 3.0, np.minimum(cost, 99.0), 0.0).astype(np.int8)
            if robot is not None:
                cost[(cx_all[i0:i1, j0:j1] - robot[0]) ** 2 + (cy_all[i0:i1, j0:j1] - robot[1]) ** 2 <= 0.45 ** 2] = 0
            region = mask[i0:i1, j0:j1]
            np.maximum(region, cost, out=region)

        for tr in people:
            personal_space(tr)

        steps = int(self.p['horizon'] / self.p['horizon_step'])
        for tr in people:
            if tr.speed < self.p['moving_speed']:
                continue
            for k in range(1, steps + 1):
                t = k * self.p['horizon_step']
                frac = t / self.p['horizon']
                stamp(tr.x[0] + tr.x[2] * t, tr.x[1] + tr.x[3] * t, 0.32, int(round(95 - 55 * frac)))
        # republish only when the mask changes (or every 2 s): each new mask makes
        # both costmaps re-process their filter
        digest = hash(mask.tobytes())
        now = self.get_clock().now().nanoseconds * 1e-9
        if digest == self.last_mask_digest and now - self.last_mask < 2.0:
            return
        self.last_mask_digest, self.last_mask = digest, now
        msg = OccupancyGrid()
        msg.header.frame_id = self.p['global_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width, msg.info.height = w, h
        msg.info.origin = info.origin
        msg.data = mask.flatten().tolist()
        self.mask_pub.publish(msg)

    def publish_markers(self, people):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for tr in people:
            m = Marker()
            m.header.frame_id, m.header.stamp = self.p['global_frame'], stamp
            m.ns, m.id, m.type = 'people', tr.id, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tr.x[0], tr.x[1], 0.85
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.44
            m.scale.z = 1.7
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.1, 0.6
            arr.markers.append(m)
            if tr.speed >= self.p['moving_speed']:
                v = Marker()
                v.header = m.header
                v.ns, v.id, v.type = 'velocity', tr.id, Marker.ARROW
                v.points = [Point(x=tr.x[0], y=tr.x[1], z=0.1),
                            Point(x=tr.x[0] + tr.x[2] * self.p['horizon'],
                                  y=tr.x[1] + tr.x[3] * self.p['horizon'], z=0.1)]
                v.scale.x, v.scale.y, v.scale.z = 0.06, 0.15, 0.15
                v.color.r, v.color.g, v.color.b, v.color.a = 1.0, 0.0, 0.0, 0.9
                arr.markers.append(v)
        self.marker_pub.publish(arr)

    def publish_speed(self, nearest, robot):
        pct = 100.0
        reason = 'free'
        if nearest is not None and nearest < self.p['near_distance']:
            pct, reason = self.p['near_speed_pct'], 'person within %.1f m' % nearest
        elif nearest is not None and nearest < self.p['close_distance']:
            pct, reason = self.p['close_speed_pct'], 'person within %.1f m' % nearest
        if robot is not None:
            for x0, x1, y0, y1 in self.zones:
                if x0 <= robot[0] <= x1 and y0 <= robot[1] <= y1 and self.p['zone_speed_pct'] < pct:
                    pct, reason = self.p['zone_speed_pct'], 'narrow passage'
        now = self.get_clock().now().nanoseconds * 1e-9
        if pct != self.last_speed or now - self.last_speed_pub > 1.0:
            msg = SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p['global_frame']
            msg.percentage = True
            msg.speed_limit = 0.0 if pct >= 100.0 else float(pct)   # 0 = no limit
            self.speed_pub.publish(msg)
            if pct != self.last_speed:
                self.get_logger().info('Speed limit %s%% (%s)' % ('100' if pct >= 100 else '%.0f' % pct, reason))
            self.last_speed, self.last_speed_pub = pct, now


def main():
    rclpy.init()
    node = PeopleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
