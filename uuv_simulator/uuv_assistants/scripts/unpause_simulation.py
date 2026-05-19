#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time


class UnpauseSimulation(Node):

    def __init__(self):
        super().__init__('unpause_simulation')

        self.declare_parameter('timeout', 0.0)
        self.timeout = self.get_parameter('timeout').value

        self.client = self.create_client(Empty, '/gazebo/unpause_physics')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/unpause_physics...")

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.check)

        self.triggered = False

    def check(self):
        if self.triggered:
            return

        if time.time() - self.start_time < self.timeout:
            return

        req = Empty.Request()
        future = self.client.call_async(req)

        self.get_logger().info("Unpausing simulation...")
        rclpy.spin_until_future_complete(self, future)

        self.triggered = True
        self.get_logger().info("Simulation unpaused")


def main():
    rclpy.init()
    node = UnpauseSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
