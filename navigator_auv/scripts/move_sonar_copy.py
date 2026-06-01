#!/usr/bin/env python3
"""
move_sonar_copy.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
Sinusoidal sonar sweep version.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MoveSonar(Node):
    def __init__(self):
        super().__init__('move_sonar')

        self.pub = self.create_publisher(
            Float64, '/rexrov2/sonar_joint_position_controller/command', 10)

        self.amplitude = 0.77
        self.frequency = 0.05

        # 10 Hz publish timer
        self.timer = self.create_timer(0.1, self.publish_position)

    def publish_position(self):
        # Use wall-clock time consistent with ROS 2
        current_time = self.get_clock().now().nanoseconds / 1e9
        position = self.amplitude * math.sin(
            2 * math.pi * self.frequency * current_time)
        msg = Float64()
        msg.data = position
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveSonar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
