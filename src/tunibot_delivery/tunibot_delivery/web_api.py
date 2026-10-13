"""REST backend for the delivery UI (standard library only, no extra deps).

  GET  /                 -> web/index.html (customer order page + admin status)
  GET  /api/status       -> last /delivery/status JSON (state, battery, energy, queue, events...)
  GET  /api/battery      -> last /battery/info JSON (power, Wh/m, time to empty...)
  GET  /api/people       -> last /people JSON (tracked people around the robot)
  GET  /api/map          -> static map (PGM pixels, base64) + resolution/origin for the live map view
  POST /api/orders       {"table": "table_2", "items": "1 tea", "priority": "vip"}  -> /delivery/order
  POST /api/command      {"command": "dock" | "cancel" | "clear" | "estop" | "resume" | "restart_nav"} -> /delivery/command
  POST /api/battery      {"soc": 20} and/or {"charger": false}   (test hooks for the low-battery scenarios)
"""
import base64
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

COMMANDS = {'dock', 'cancel', 'clear', 'estop', 'resume', 'restart_nav'}


class WebApi(Node):

    def __init__(self):
        super().__init__('web_api')
        self.declare_parameter('port', 8080)
        self.status = {'state': 'UNKNOWN'}
        self.order_pub = self.create_publisher(String, '/delivery/order', 10)
        self.cmd_pub = self.create_publisher(String, '/delivery/command', 10)
        self.battery = {}
        self.people = {}
        self.soc_pub = self.create_publisher(Float32, '/battery/set_soc', 10)
        self.charger_pub = self.create_publisher(Bool, '/battery/charger', 10)
        self.create_subscription(String, '/delivery/status', self.on_status, 10)
        self.create_subscription(String, '/battery/info', lambda m: setattr(self, 'battery', json.loads(m.data)), 10)
        self.create_subscription(String, '/people', lambda m: setattr(self, 'people', json.loads(m.data)), 10)

    def on_status(self, msg):
        self.status = json.loads(msg.data)


def load_map(maps_dir):
    """Static map for the UI: {'width', 'height', 'resolution', 'origin', 'pixels' (base64, row 0 = top)}."""
    meta = {}
    with open(os.path.join(maps_dir, 'restaurant_map.yaml')) as f:
        for line in f:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    with open(os.path.join(maps_dir, meta['image']), 'rb') as f:
        raw = f.read()
    fields, i = [], 0
    while len(fields) < 4:            # P5 header: magic, width, height, maxval (comments allowed)
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        fields.append(raw[i:j])
        i = j
    w, h = int(fields[1]), int(fields[2])
    return {'width': w, 'height': h, 'resolution': float(meta['resolution']),
            'origin': [float(v) for v in meta['origin'].strip('[]').split(',')[:2]],
            'pixels': base64.b64encode(raw[i + 1:i + 1 + w * h]).decode()}


def make_handler(node: WebApi, index_path: str, map_json):

    class Handler(BaseHTTPRequestHandler):

        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)

        def _json(self):
            n = int(self.headers.get('Content-Length', 0))
            try:
                return json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                return None

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                with open(index_path, 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            elif self.path == '/api/status':
                self._send(200, node.status)
            elif self.path == '/api/battery':
                self._send(200, node.battery)
            elif self.path == '/api/people':
                self._send(200, node.people)
            elif self.path == '/api/map':
                self._send(200 if map_json else 404, map_json or {'error': 'no map'})
            else:
                self._send(404, {'error': 'not found'})

        def do_POST(self):
            body = self._json()
            if not isinstance(body, dict):
                return self._send(400, {'error': 'invalid JSON'})
            if self.path == '/api/orders':
                table = body.get('table')
                if table not in node.status.get('tables', []):
                    return self._send(400, {'error': 'unknown table %r' % table})
                node.order_pub.publish(String(data=json.dumps(
                    {'table': table, 'items': str(body.get('items', '')),
                     'priority': 'vip' if body.get('priority') == 'vip' else 'normal'})))
                return self._send(202, {'accepted': True})
            if self.path == '/api/battery':
                if 'soc' in body:
                    try:
                        soc = float(body['soc'])
                    except (TypeError, ValueError):
                        return self._send(400, {'error': 'soc must be a number'})
                    node.soc_pub.publish(Float32(data=max(0.0, min(100.0, soc))))
                if 'charger' in body:
                    node.charger_pub.publish(Bool(data=bool(body['charger'])))
                return self._send(202, {'accepted': True})
            if self.path == '/api/command':
                cmd = body.get('command')
                if cmd not in COMMANDS:
                    return self._send(400, {'error': 'command must be one of %s' % sorted(COMMANDS)})
                node.cmd_pub.publish(String(data=cmd))
                return self._send(202, {'accepted': True})
            self._send(404, {'error': 'not found'})

        def log_message(self, fmt, *args):
            node.get_logger().debug(fmt % args)

    return Handler


def main():
    rclpy.init()
    node = WebApi()
    port = int(node.get_parameter('port').value)
    share = get_package_share_directory('tunibot_delivery')
    index = os.path.join(share, 'web', 'index.html')
    try:
        map_json = load_map(os.path.join(share, 'maps'))
    except (OSError, KeyError, ValueError) as ex:
        node.get_logger().warn('No map for the UI: %s' % ex)
        map_json = None
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, index, map_json))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info('Delivery UI on http://localhost:%d' % port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
