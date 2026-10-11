"""Lifecycle watchdog for the Nav2 bringup.

With FastDDS (ROS 2 Humble) a lifecycle `change_state` request sent while
discovery is still in progress can lose its response ("failed to send response
to /<node>/change_state (timeout)"). The Nav2 lifecycle manager then waits
forever and the navigation stack never becomes active (observed in ~1 out of 5
launches). This node checks, after `start_delay` seconds and then periodically
until everything is active, the state of every managed node and drives the
stuck ones to ACTIVE itself (configure, activate) with fresh service clients,
timeouts and retries. It exits once all nodes are active.
"""
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleWatchdog(Node):

    def __init__(self):
        super().__init__('lifecycle_watchdog')
        self.declare_parameter('node_names', [''])
        self.declare_parameter('start_delay', 20.0)
        self.declare_parameter('period', 5.0)
        self.declare_parameter('give_up_after', 180.0)
        self.nodes = [n for n in self.get_parameter('node_names').value if n]
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.period = float(self.get_parameter('period').value)
        self.give_up = float(self.get_parameter('give_up_after').value)

    def call(self, srv_type, name, request, timeout=3.0):
        cli = self.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(request)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result() if fut.done() else None
        finally:
            self.destroy_client(cli)

    def state(self, node):
        res = self.call(GetState, '/%s/get_state' % node, GetState.Request())
        return None if res is None else res.current_state.id

    def transition(self, node, tid):
        req = ChangeState.Request()
        req.transition.id = tid
        res = self.call(ChangeState, '/%s/change_state' % node, req, timeout=10.0)
        return res is not None and res.success

    def run(self):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < self.start_delay:
            rclpy.spin_once(self, timeout_sec=0.2)
        while rclpy.ok() and time.monotonic() - t0 < self.give_up:
            all_active = True
            # the order of node_names matters (map_server before amcl, ... bt_navigator last)
            for node in self.nodes:
                st = self.state(node)
                if st == State.PRIMARY_STATE_ACTIVE:
                    continue
                all_active = False
                if st == State.PRIMARY_STATE_UNCONFIGURED:
                    self.get_logger().warn('%s stuck UNCONFIGURED: configuring it' % node)
                    if not self.transition(node, Transition.TRANSITION_CONFIGURE):
                        break
                    st = self.state(node)
                if st == State.PRIMARY_STATE_INACTIVE:
                    self.get_logger().warn('%s stuck INACTIVE: activating it' % node)
                    if not self.transition(node, Transition.TRANSITION_ACTIVATE):
                        break
                elif st is None:
                    self.get_logger().info('%s: no state yet' % node)
                    break
            if all_active:
                self.get_logger().info('all %d Nav2 nodes active' % len(self.nodes))
                return
            t = time.monotonic()
            while rclpy.ok() and time.monotonic() - t < self.period:
                rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().error('Nav2 nodes still not all active after %.0f s' % self.give_up)


def main():
    rclpy.init()
    node = LifecycleWatchdog()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
