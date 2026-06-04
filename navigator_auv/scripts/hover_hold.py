#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HoverHold(Node):
    def __init__(self):
        super().__init__('hover_hold')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('cmd_vel_topic', '/rexrov2/cmd_vel')
        self.declare_parameter('target_depth', -60.0)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('max_vertical_speed', 1.2)
        self.declare_parameter('publish_rate', 20.0)

        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.max_vertical_speed = float(
            self.get_parameter('max_vertical_speed').value)
        self.current_z = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, pose_topic, self.pose_cb, 10)
        self.create_timer(1.0 / publish_rate, self.control_cb)

    def pose_cb(self, msg):
        self.current_z = msg.pose.pose.position.z

    def control_cb(self):
        if self.current_z is None:
            return

        error = self.target_depth - self.current_z
        vertical_speed = self.depth_kp * error
        vertical_speed = max(
            -self.max_vertical_speed,
            min(self.max_vertical_speed, vertical_speed))

        cmd = Twist()
        cmd.linear.z = vertical_speed
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = HoverHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
