#!/usr/bin/env python3
"""
euler_calculator.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - rospy → rclpy / Node
  - tf.transformations → scipy.spatial.transform.Rotation
  - Global mutable pub reference → proper Node member
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation


TARGET_X = 50.0
TARGET_Y = 72.0


class MoveRobotToTarget(Node):
    def __init__(self):
        super().__init__('move_robot_to_target')

        self.pub = self.create_publisher(Float64, '/rexrov2/global_angle', 10)

        self.create_subscription(
            Odometry,
            '/rexrov2/pose_gt',
            self.pose_callback,
            10)

        self.get_logger().info('Move Robot to Target Node started')

    # ------------------------------------------------------------------
    def pose_callback(self, pose_msg: Odometry):
        pose = pose_msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        orientation = pose.orientation

        rot = Rotation.from_quat([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        _, _, yaw = rot.as_euler('xyz', degrees=False)

        angle_to_target = math.atan2(TARGET_Y - y, TARGET_X - x)
        angular_error = angle_to_target - yaw
        # Normalise to [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        msg = Float64()
        msg.data = angular_error
        self.pub.publish(msg)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
