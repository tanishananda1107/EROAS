#!/usr/bin/env python3

from __future__ import annotations

import os

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class VehicleTeleop(Node):

    def __init__(self):

        super().__init__('vehicle_teleop')

        self._axes = {
            'x': 4,
            'y': 3,
            'z': 1,
            'roll': 2,
            'pitch': 5,
            'yaw': 0
        }

        self._axes_gain = {
            'x': 3.0,
            'y': 3.0,
            'z': 0.5,
            'roll': 0.5,
            'pitch': 0.5,
            'yaw': 0.5
        }

        self.declare_parameter('deadzone', 0.5)
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('home_button', 7)
        self.declare_parameter('type', 'twist')

        self._deadzone = self.get_parameter(
            'deadzone').value

        self._deadman_button = self.get_parameter(
            'deadman_button').value

        self._home_button = self.get_parameter(
            'home_button').value

        self._msg_type = self.get_parameter(
            'type').value

        if self._msg_type == 'twist':
            self._pub = self.create_publisher(
                Twist,
                'output',
                10
            )
        else:
            self._pub = self.create_publisher(
                Accel,
                'output',
                10
            )

        self._home_pub = self.create_publisher(
            Bool,
            'home_pressed',
            10
        )

        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def parse_joy(self, joy=None):

        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()

        if joy is None:
            cmd.linear = Vector3()
            cmd.angular = Vector3()
            return cmd

        l = Vector3()
        a = Vector3()

        if abs(joy.axes[self._axes['x']]) > self._deadzone:
            l.x = self._axes_gain['x'] * joy.axes[self._axes['x']]

        if abs(joy.axes[self._axes['y']]) > self._deadzone:
            l.y = self._axes_gain['y'] * joy.axes[self._axes['y']]

        if abs(joy.axes[self._axes['z']]) > self._deadzone:
            l.z = self._axes_gain['z'] * joy.axes[self._axes['z']]

        if abs(joy.axes[self._axes['roll']]) > self._deadzone:
            a.x = self._axes_gain['roll'] * joy.axes[self._axes['roll']]

        if abs(joy.axes[self._axes['pitch']]) > self._deadzone:
            a.y = self._axes_gain['pitch'] * joy.axes[self._axes['pitch']]

        if abs(joy.axes[self._axes['yaw']]) > self._deadzone:
            a.z = self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

        cmd.linear = l
        cmd.angular = a

        return cmd

    def joy_callback(self, joy):

        try:

            if self._deadman_button != -1:

                if joy.buttons[self._deadman_button] == 1:
                    cmd = self.parse_joy(joy)
                else:
                    cmd = self.parse_joy()

            else:
                cmd = self.parse_joy(joy)

            self._pub.publish(cmd)

            home_msg = Bool()
            home_msg.data = bool(
                joy.buttons[self._home_button]
            )

            self._home_pub.publish(home_msg)

        except Exception as e:
            self.get_logger().error(
                f'Joystick parsing error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = VehicleTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
