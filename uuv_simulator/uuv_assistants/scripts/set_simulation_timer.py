#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class SimulationTimer(Node):

    def __init__(self):
        super().__init__('set_simulation_timer')

        self.declare_parameter('timeout', 0.0)
        self.timeout = self.get_parameter('timeout').value

        if self.timeout <= 0:
            self.get_logger().error("timeout must be > 0")
            rclpy.shutdown()
            return

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f"Simulation timer started: {self.timeout}s")

        self.timer = self.create_timer(0.1, self.check)

    def check(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.start_time >= self.timeout:
            self.get_logger().info("Timeout reached → stopping simulation")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SimulationTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
