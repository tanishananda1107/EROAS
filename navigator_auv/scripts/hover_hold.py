#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, limit):
    return max(-limit, min(limit, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HoverHold(Node):
    def __init__(self):
        super().__init__('hover_hold')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('cmd_vel_topic', '/rexrov2/cmd_vel')
        self.declare_parameter('target_depth', -60.0)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('max_vertical_speed', 1.2)
        self.declare_parameter('hold_xy', True)
        self.declare_parameter('hold_yaw', True)
        self.declare_parameter('use_initial_xy', True)
        self.declare_parameter('use_initial_yaw', True)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('xy_kp', 0.45)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('max_horizontal_speed', 0.6)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('publish_rate', 20.0)

        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.max_vertical_speed = float(
            self.get_parameter('max_vertical_speed').value)
        self.hold_xy = bool(self.get_parameter('hold_xy').value)
        self.hold_yaw = bool(self.get_parameter('hold_yaw').value)
        self.use_initial_xy = bool(
            self.get_parameter('use_initial_xy').value)
        self.use_initial_yaw = bool(
            self.get_parameter('use_initial_yaw').value)
        self.xy_kp = float(self.get_parameter('xy_kp').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_horizontal_speed = float(
            self.get_parameter('max_horizontal_speed').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value)

        self.target_x = None if self.use_initial_xy else float(
            self.get_parameter('target_x').value)
        self.target_y = None if self.use_initial_xy else float(
            self.get_parameter('target_y').value)
        self.target_yaw = None if self.use_initial_yaw else float(
            self.get_parameter('target_yaw').value)

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, pose_topic, self.pose_cb, 10)
        self.create_timer(1.0 / publish_rate, self.control_cb)

    def pose_cb(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_z = pose.position.z
        self.current_yaw = yaw_from_quaternion(pose.orientation)

        if self.hold_xy and self.target_x is None:
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.get_logger().info(
                f'Hover hold XY target set to '
                f'x={self.target_x:.2f}, y={self.target_y:.2f}')

        if self.hold_yaw and self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(
                f'Hover hold yaw target set to {self.target_yaw:.2f} rad')

    def control_cb(self):
        if self.current_z is None:
            return

        error = self.target_depth - self.current_z
        cmd = Twist()
        cmd.linear.z = clamp(
            self.depth_kp * error, self.max_vertical_speed)

        if self.hold_xy and self.target_x is not None:
            error_x = self.target_x - self.current_x
            error_y = self.target_y - self.current_y
            vx_world = clamp(
                self.xy_kp * error_x, self.max_horizontal_speed)
            vy_world = clamp(
                self.xy_kp * error_y, self.max_horizontal_speed)

            # Gazebo VelocityControl consumes Twist in the vehicle frame.
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            cmd.linear.x = cos_yaw * vx_world + sin_yaw * vy_world
            cmd.linear.y = -sin_yaw * vx_world + cos_yaw * vy_world

        if self.hold_yaw and self.target_yaw is not None:
            yaw_error = wrap_pi(self.target_yaw - self.current_yaw)
            cmd.angular.z = clamp(
                self.yaw_kp * yaw_error, self.max_yaw_rate)

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
