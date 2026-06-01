#!/usr/bin/env python3
"""
euler.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - tf.transformations → transforms3d (pip install transforms3d)
    or use scipy.spatial.transform.Rotation (zero extra deps)
  - rospy → rclpy / Node
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Use scipy for quaternion → Euler conversion (no tf dependency needed in ROS 2)
from scipy.spatial.transform import Rotation


class PoseEulerCalculator(Node):
    def __init__(self):
        super().__init__('pose_euler_calculator')

        self.create_subscription(
            Odometry,
            '/rexrov2/pose_gt',
            self.callback,
            10)

        self.get_logger().info('Pose Euler Calculator Node started')

    # ------------------------------------------------------------------
    def callback(self, pose_msg: Odometry):
        quat = pose_msg.pose.pose.orientation
        # scipy expects [x, y, z, w]
        rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
        self.get_logger().info(
            f'Roll: {roll:.6f}, Pitch: {pitch:.6f}, Yaw: {yaw:.6f}')


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PoseEulerCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
