#!/usr/bin/env python3
"""
freq_changer.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - rospy → rclpy / Node
  - Subscriber + manual publish loop → subscriber callback + wall timer
  - rospy.Rate → self.create_timer (7 Hz publish rate)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelForwarderNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_forwarder_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 1)

        self.create_subscription(
            Twist,
            '/rexrov2/cmd_vel_1',
            self.cmd_vel_heading_callback,
            10)

        self.latest_cmd_vel = Twist()

        # Publish at ~7 Hz (matches original publish_rate)
        self.timer = self.create_timer(1.0 / 7.0, self.publish_cmd_vel)

        self.get_logger().info('CmdVel Forwarder Node started')

    # ------------------------------------------------------------------
    def cmd_vel_heading_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    # ------------------------------------------------------------------
    def publish_cmd_vel(self):
        self.cmd_vel_pub.publish(self.latest_cmd_vel)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelForwarderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
