#!/usr/bin/env python3

from __future__ import annotations

import os
import sys
import select
import tty
import termios
import time

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Vector3


class KeyBoardVehicleTeleop(Node):

    def __init__(self):

        super().__init__('vehicle_keyboard_teleop')

        self.settings = termios.tcgetattr(sys.stdin)

        self.speed = 1

        self.l = Vector3()
        self.a = Vector3()

        self.linear_increment = 0.05
        self.linear_limit = 1.0

        self.angular_increment = 0.05
        self.angular_limit = 0.5

        self.declare_parameter('type', 'twist')

        self._msg_type = self.get_parameter(
            'type').get_parameter_value().string_value

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

        self.timer = self.create_timer(
            0.02,
            self.loop
        )

        print("""
Control Your Vehicle!

W/S : X
A/D : Y
X/Z : Z

Q/E : Yaw
I/K : Pitch
J/L : Roll

1/2 : Speed

CTRL+C to quit
""")

    def get_key(self):

        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.1
        )

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key

    def speed_windup(
        self,
        speed,
        increment,
        limit,
        reverse
    ):

        if reverse:
            speed -= increment * self.speed

            if speed < -limit * self.speed:
                speed = -limit * self.speed
        else:
            speed += increment * self.speed

            if speed > limit * self.speed:
                speed = limit * self.speed

        return speed

    def loop(self):

        key = self.get_key()

        if key == '1':
            self.speed = 1

        if key == '2':
            self.speed = 2

        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()

        if key != '':

            if key == 'w':
                self.l.x = self.speed_windup(
                    self.l.x,
                    self.linear_increment,
                    self.linear_limit,
                    False
                )

            if key == 's':
                self.l.x = self.speed_windup(
                    self.l.x,
                    self.linear_increment,
                    self.linear_limit,
                    True
                )

            if key == 'a':
                self.l.y = self.speed_windup(
                    self.l.y,
                    self.linear_increment,
                    self.linear_limit,
                    False
                )

            if key == 'd':
                self.l.y = self.speed_windup(
                    self.l.y,
                    self.linear_increment,
                    self.linear_limit,
                    True
                )

            if key == 'x':
                self.l.z = self.speed_windup(
                    self.l.z,
                    self.linear_increment,
                    self.linear_limit * 0.5,
                    False
                )

            if key == 'z':
                self.l.z = self.speed_windup(
                    self.l.z,
                    self.linear_increment,
                    self.linear_limit * 0.5,
                    True
                )

            if key == 'j':
                self.a.x = self.speed_windup(
                    self.a.x,
                    self.angular_increment,
                    self.angular_limit,
                    True
                )

            if key == 'l':
                self.a.x = self.speed_windup(
                    self.a.x,
                    self.angular_increment,
                    self.angular_limit,
                    False
                )

            if key == 'i':
                self.a.y = self.speed_windup(
                    self.a.y,
                    self.angular_increment,
                    self.angular_limit,
                    False
                )

            if key == 'k':
                self.a.y = self.speed_windup(
                    self.a.y,
                    self.angular_increment,
                    self.angular_limit,
                    True
                )

            if key == 'q':
                self.a.z = self.speed_windup(
                    self.a.z,
                    self.angular_increment,
                    self.angular_limit,
                    False
                )

            if key == 'e':
                self.a.z = self.speed_windup(
                    self.a.z,
                    self.angular_increment,
                    self.angular_limit,
                    True
                )

        else:
            self.l = Vector3()
            self.a = Vector3()

        cmd.linear = self.l
        cmd.angular = self.a

        self._pub.publish(cmd)


def main(args=None):

    time.sleep(2)

    rclpy.init(args=args)

    node = KeyBoardVehicleTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        node.settings
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
