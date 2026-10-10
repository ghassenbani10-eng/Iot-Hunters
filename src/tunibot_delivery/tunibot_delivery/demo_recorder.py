"""Records a demo video (MP4) of the running simulation, no screen recorder needed.

    ros2 run tunibot_delivery demo_recorder --ros-args -p out:=~/tunibot_demo.mp4 -p duration:=180.0

It spawns two Gazebo cameras:
  * a chase camera that follows the robot (moved every frame),
  * an overhead camera above the dining room,
and composes, at 10 fps, a 1600x900 frame:
  +-------------------------------+---------------+
  |                               |  overhead     |
  |  chase camera (Gazebo)        |  camera       |
  |                               +---------------+
  |                               |  2D map: robot, path, tracked
  +-------------------------------+  people + predicted motion
  |  state · battery · people · last events                    |
  +-------------------------------------------------------------+
Stops after `duration` seconds (wall clock), or when the delivery manager is
back in DOCKED after having been busy (stop_when_docked).
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tunibot_delivery.test_plots import load_map

DOCK_WORLD = (-9.0, 2.2)          # map = world - DOCK_WORLD
W, H = 1600, 900
STATE_COLORS = {'DOCKED': (60, 160, 60), 'CHARGING': (90, 200, 90), 'TO_PICKUP': (230, 140, 40),
                'LOADING': (240, 200, 120), 'TO_TABLE': (170, 60, 140), 'SERVING': (210, 150, 210),
                'RETURNING_TO_DOCK': (40, 140, 250), 'DOCKING': (120, 200, 255), 'IDLE': (160, 160, 160),
                'EMERGENCY_STOP': (40, 40, 220), 'ERROR': (30, 30, 200)}


def camera_sdf(name, w, h, fov):
    return f"""<?xml version='1.0'?><sdf version='1.7'><model name='{name}'><static>1</static>
<link name='link'><sensor type='camera' name='cam'><update_rate>10</update_rate>
<camera><horizontal_fov>{fov}</horizontal_fov><image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
<clip><near>0.1</near><far>60</far></clip></camera>
<plugin name='{name}_plugin' filename='libgazebo_ros_camera.so'><ros><namespace>/{name}</namespace></ros>
<camera_name>cam</camera_name><frame_name>{name}</frame_name></plugin></sensor></link></model></sdf>"""


def quat(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class DemoRecorder(Node):

    def __init__(self):
        super().__init__('demo_recorder')
        self.declare_parameter('out', os.path.expanduser('~/tunibot_demo.mp4'))
        self.declare_parameter('duration', 240.0)
        self.declare_parameter('stop_when_docked', True)
        self.declare_parameter('title', 'TuniBot - dynamic obstacle avoidance')
        self.out = os.path.expanduser(self.get_parameter('out').value)
        self.duration = float(self.get_parameter('duration').value)
        self.stop_when_docked = bool(self.get_parameter('stop_when_docked').value)
        self.title = self.get_parameter('title').value
        self.bridge = CvBridge()
        self.img = {}
        self.status, self.people, self.plan, self.truth = {}, {}, [], {}
        for cam in ('demo_cam_chase', 'demo_cam_top'):
            self.create_subscription(Image, '/%s/cam/image_raw' % cam,
                                     lambda m, c=cam: self.img.__setitem__(c, m), 2)
        self.create_subscription(String, '/delivery/status', lambda m: setattr(self, 'status', json.loads(m.data)), 5)
        self.create_subscription(String, '/people', lambda m: setattr(self, 'people', json.loads(m.data)), 5)
        self.create_subscription(Path, '/plan', lambda m: setattr(
            self, 'plan', [(p.pose.position.x, p.pose.position.y) for p in m.poses]), 2)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 5)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        try:
            self.map_img, self.map_ext = load_map()
        except (OSError, KeyError, ValueError):
            self.map_img = None

    def on_models(self, msg):
        t = {}
        for n, p in zip(msg.name, msg.pose):
            q = p.orientation
            t[n] = (p.position.x, p.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.truth = t

    # ------------------------------------------------------------ cameras
    def spawn_cameras(self):
        self.spawn_cli.wait_for_service(timeout_sec=20.0)
        for name, w, h, fov, pose in (('demo_cam_chase', 1100, 620, 1.3, (-9.0, 2.2, 3.0, 0.0, 0.6, 0.0)),
                                      ('demo_cam_top', 500, 360, 1.5, (-0.5, -3.6, 15.0, 0.0, 1.5707, 1.5708))):
            req = SpawnEntity.Request()
            req.name, req.xml = name, camera_sdf(name, w, h, fov)
            req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = pose[:3]
            qx, qy, qz, qw = quat(*pose[3:])
            req.initial_pose.orientation.x, req.initial_pose.orientation.y = qx, qy
            req.initial_pose.orientation.z, req.initial_pose.orientation.w = qz, qw
            fut = self.spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            self.get_logger().info('camera %s: %s' % (name, fut.result().status_message if fut.result() else 'timeout'))

    def move_chase(self):
        rp = self.truth.get('waiter_robot')
        if rp is None or not self.set_cli.service_is_ready():
            return
        x, y, yaw = rp
        back, up = 3.2, 2.6                       # behind and above the robot, looking at it
        cx, cy = x - back * math.cos(yaw), y - back * math.sin(yaw)
        st = EntityState()
        st.name = 'demo_cam_chase'
        st.pose.position.x, st.pose.position.y, st.pose.position.z = cx, cy, up
        q = quat(0.0, math.atan2(up - 0.6, back), yaw)
        st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = q
        st.reference_frame = 'world'
        self.set_cli.call_async(SetEntityState.Request(state=st))

    # ------------------------------------------------------------ drawing
    def cam_frame(self, name, w, h):
        m = self.img.get(name)
        if m is None:
            f = np.full((h, w, 3), 40, np.uint8)
            cv2.putText(f, 'waiting for %s' % name, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            return f
        f = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        return cv2.resize(f, (w, h))

    def map_panel(self, w, h):
        f = np.full((h, w, 3), 245, np.uint8)
        if self.map_img is None:
            return f
        ox, x1, oy, y1 = self.map_ext
        # crop around the dining room + lounge (map frame)
        vx0, vx1, vy0, vy1 = -1.8, 18.8, -12.2, 8.6
        s = min(w / (vx1 - vx0), h / (vy1 - vy0))
        def P(x, y):  # noqa: E306
            return int((x - vx0) * s), int(h - (y - vy0) * s)
        res = (x1 - ox) / self.map_img.shape[1]
        mh, mw = self.map_img.shape
        j0, j1 = int((vx0 - ox) / res), int((vx1 - ox) / res)
        i0, i1 = int(mh - (vy1 - oy) / res), int(mh - (vy0 - oy) / res)
        crop = np.full((i1 - i0, j1 - j0), 205, np.uint8)
        si0, si1, sj0, sj1 = max(i0, 0), min(i1, mh), max(j0, 0), min(j1, mw)
        crop[si0 - i0:si1 - i0, sj0 - j0:sj1 - j0] = self.map_img[si0:si1, sj0:sj1]
        crop = cv2.resize(crop, (int((vx1 - vx0) * s), int((vy1 - vy0) * s)), interpolation=cv2.INTER_NEAREST)
        f[:crop.shape[0], :crop.shape[1]] = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        if len(self.plan) > 1:
            pts = np.array([P(x, y) for x, y in self.plan], np.int32)
            cv2.polylines(f, [pts], False, (200, 80, 200), 2)
        for t in self.people.get('tracks', []):
            c = P(t['x'], t['y'])
            cv2.circle(f, c, max(3, int(0.25 * s)), (30, 110, 240), -1)
            if t.get('speed', 0) > 0.25:
                cv2.arrowedLine(f, c, P(t['x'] + 2 * t['vx'], t['y'] + 2 * t['vy']), (30, 60, 220), 2, tipLength=0.3)
        pose = self.status.get('pose')
        if pose:
            x, y, yaw = pose
            c, sn = math.cos(yaw), math.sin(yaw)
            fp = [(0.27, 0.25), (0.27, -0.25), (-0.27, -0.25), (-0.27, 0.25)]
            pts = np.array([P(x + c * a - sn * b, y + sn * a + c * b) for a, b in fp], np.int32)
            cv2.fillPoly(f, [pts], (200, 90, 30))
            cv2.arrowedLine(f, P(x, y), P(x + 0.6 * c, y + 0.6 * sn), (255, 255, 255), 2, tipLength=0.4)
        cv2.putText(f, 'live map: robot (blue), path, people + 2 s prediction', (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1)
        return f

    def info_bar(self, w, h, t):
        f = np.full((h, w, 3), 30, np.uint8)
        st = self.status
        state = st.get('state', '-')
        col = STATE_COLORS.get(state, (200, 200, 200))
        cv2.putText(f, self.title, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(f, 't = %5.1f s' % t, (w - 160, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.rectangle(f, (14, 44), (300, 80), col, -1)
        cv2.putText(f, state.replace('_', ' '), (22, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2)
        cv2.putText(f, st.get('nav_status', '')[:40], (312, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        b = st.get('battery', {})
        soc = float(b.get('soc', 0) or 0)
        bx = 760
        cv2.rectangle(f, (bx, 48), (bx + 200, 76), (90, 90, 90), 1)
        cv2.rectangle(f, (bx + 1, 49), (bx + 1 + int(198 * soc / 100), 75),
                      (60, 180, 60) if soc > 25 else (40, 160, 230) if soc > 10 else (40, 40, 220), -1)
        cv2.putText(f, 'battery %.0f%%%s' % (soc, '  charging' if b.get('charging') else ''), (bx + 210, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        ppl = self.people
        cv2.putText(f, 'people tracked: %s%s' % (ppl.get('count', 0), '  nearest %.1f m' % ppl['nearest']
                                                 if ppl.get('nearest') is not None else ''),
                    (bx + 470, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        cur = st.get('current_order')
        if cur:
            cv2.putText(f, 'order #%s -> %s %s' % (cur['id'], cur['table'], cur.get('items', '')[:40]), (14, 108),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        for i, e in enumerate((st.get('events') or [])[:3]):
            cv2.putText(f, '%6.0fs  %s' % (e['t'], e['text'][:95]), (560, 100 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (180, 220, 255) if e['kind'] in ('nav', 'recovery') else (200, 200, 200), 1)
        return f

    def compose(self, t):
        frame = np.zeros((H, W, 3), np.uint8)
        frame[0:620, 0:1100] = self.cam_frame('demo_cam_chase', 1100, 620)
        frame[0:360, 1100:1600] = self.cam_frame('demo_cam_top', 500, 360)
        frame[360:740, 1100:1600] = self.map_panel(500, 380)
        frame[620:740, 0:1100] = 30
        frame[740:900, :] = self.info_bar(W, 160, t)
        cv2.putText(frame, 'chase camera (Gazebo)', (10, 610), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, 'overhead camera', (1110, 352), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return frame


def main():
    rclpy.init()
    node = DemoRecorder()
    node.spawn_cameras()
    writer = cv2.VideoWriter(node.out, cv2.VideoWriter_fourcc(*'mp4v'), 10, (W, H))
    t0 = time.time()
    next_frame = t0
    busy = False
    frames = 0
    try:
        while rclpy.ok() and time.time() - t0 < node.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.time() >= next_frame:
                next_frame += 0.1
                node.move_chase()
                writer.write(node.compose(time.time() - t0))
                frames += 1
            state = node.status.get('state')
            busy = busy or state not in (None, 'DOCKED', 'CHARGING', 'STARTING')
            if node.stop_when_docked and busy and state == 'DOCKED':
                for _ in range(30):     # 3 s of the robot on its dock
                    rclpy.spin_once(node, timeout_sec=0.1)
                    writer.write(node.compose(time.time() - t0))
                break
    finally:
        writer.release()
        node.get_logger().info('wrote %s (%d frames)' % (node.out, frames))
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
