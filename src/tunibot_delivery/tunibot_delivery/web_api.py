"""REST backend for the delivery UI (standard library only, no extra deps).

  GET  /                 -> web/index.html (customer order page + admin status)
  GET  /api/status       -> last /delivery/status JSON
  POST /api/orders       {"table": "table_2", "items": "1 tea"}  -> /delivery/order
  POST /api/command      {"command": "dock" | "cancel" | "clear"} -> /delivery/command
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

COMMANDS = {'dock', 'cancel', 'clear'}


class WebApi(Node):

    def __init__(self):
        super().__init__('web_api')
        self.declare_parameter('port', 8080)
        self.status = {'state': 'UNKNOWN'}
        self.order_pub = self.create_publisher(String, '/delivery/order', 10)
        self.cmd_pub = self.create_publisher(String, '/delivery/command', 10)
        self.create_subscription(String, '/delivery/status', self.on_status, 10)

    def on_status(self, msg):
        self.status = json.loads(msg.data)


def make_handler(node: WebApi, index_path: str):

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
                    {'table': table, 'items': str(body.get('items', ''))})))
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
    index = os.path.join(get_package_share_directory('tunibot_delivery'), 'web', 'index.html')
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, index))
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
