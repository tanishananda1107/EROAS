#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import csv
import math


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (radians)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseToCSV(Node):
    def __init__(self):
        super().__init__('pose_to_csv')

        self.h_value = None

        self.file = open('/home/user/pose_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['Timestamp', 'Position_X', 'Position_Y', 'Position_Z', 'Yaw', 'H'])

        self.pose_subscriber = self.create_subscription(
            Odometry,
            '/rexrov2/pose_gt',
            self.pose_callback,
            10
        )
        self.h_subscriber = self.create_subscription(
            Float64,
            '/rexrov2/current_h',
            self.h_callback,
            10
        )
        self.get_logger().info("PoseToCSV node started. Writing to /home/user/pose_data.csv")

    def pose_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        h = self.h_value if self.h_value is not None else 'N/A'

        self.csv_writer.writerow([
            timestamp,
            position.x,
            position.y,
            position.z,
            yaw,
            h
        ])

    def h_callback(self, msg):
        self.h_value = msg.data

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("CSV file closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseToCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
