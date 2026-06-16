#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PoseGtToOdom(Node):
    def __init__(self):
        super().__init__('pose_gt_to_odom')
        self.declare_parameter('input_topic', '/rexrov2/pose_gt')
        self.declare_parameter('output_topic', '/rexrov2/odom')
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self.publisher.publish, 10)


def main():
    rclpy.init()
    node = PoseGtToOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
