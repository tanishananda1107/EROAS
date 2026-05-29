#!/usr/bin/env python3

# Copyright (c) 2016 The UUV Simulator Authors.
# ROS2 + Gazebo Harmonic migration

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class FinnedUUVControllerNode(Node):

    def __init__(self):
        super().__init__('finned_uuv_teleop')

        self.get_logger().info('Initializing FinnedUUVControllerNode')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('n_fins', 4)

        self.declare_parameter('gain_roll', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('gain_pitch', [1.0, 1.0, -1.0, -1.0])
        self.declare_parameter('gain_yaw', [-1.0, 1.0, 1.0, -1.0])

        self.declare_parameter('axis_thruster', 1)
        self.declare_parameter('axis_roll', 0)
        self.declare_parameter('axis_pitch', 4)
        self.declare_parameter('axis_yaw', 3)

        self.declare_parameter('thruster_joy_gain', 1.0)
        self.declare_parameter('max_thrust', 200.0)

        self.declare_parameter('thruster_topic', 'thrusters/0/input')

        self.declare_parameter('fin_topic_prefix', 'fins/')
        self.declare_parameter('fin_topic_suffix', '/input')

        # -----------------------------
        # Read parameters
        # -----------------------------
        self._n_fins = self.get_parameter(
            'n_fins').get_parameter_value().integer_value

        gain_roll = self.get_parameter(
            'gain_roll').get_parameter_value().double_array_value

        gain_pitch = self.get_parameter(
            'gain_pitch').get_parameter_value().double_array_value

        gain_yaw = self.get_parameter(
            'gain_yaw').get_parameter_value().double_array_value

        self._thruster_joy_gain = self.get_parameter(
            'thruster_joy_gain').get_parameter_value().double_value

        self._max_thrust = self.get_parameter(
            'max_thrust').get_parameter_value().double_value

        self._joy_axis = {
            'axis_thruster':
                self.get_parameter('axis_thruster')
                .get_parameter_value().integer_value,

            'axis_roll':
                self.get_parameter('axis_roll')
                .get_parameter_value().integer_value,

            'axis_pitch':
                self.get_parameter('axis_pitch')
                .get_parameter_value().integer_value,

            'axis_yaw':
                self.get_parameter('axis_yaw')
                .get_parameter_value().integer_value
        }

        if (
            len(gain_roll) != self._n_fins or
            len(gain_pitch) != self._n_fins or
            len(gain_yaw) != self._n_fins
        ):
            raise RuntimeError(
                'Gain vector sizes must match n_fins'
            )

        self._rpy_to_fins = np.vstack(
            (gain_roll, gain_pitch, gain_yaw)
        ).T

        # -----------------------------
        # Publishers
        # -----------------------------
        self._pub_cmd = []

        fin_topic_prefix = self.get_parameter(
            'fin_topic_prefix').get_parameter_value().string_value

        fin_topic_suffix = self.get_parameter(
            'fin_topic_suffix').get_parameter_value().string_value

        for i in range(self._n_fins):
            topic = f'{fin_topic_prefix}{i}{fin_topic_suffix}'

            pub = self.create_publisher(
                Float64,
                topic,
                10
            )

            self._pub_cmd.append(pub)

        thruster_topic = self.get_parameter(
            'thruster_topic').get_parameter_value().string_value

        self._thruster_pub = self.create_publisher(
            Float64,
            thruster_topic,
            10
        )

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('Finned teleop node started')

    def joy_callback(self, msg: Joy):

        try:
            thrust_axis = msg.axes[self._joy_axis['axis_thruster']]

            thrust = max(0.0, thrust_axis) * \
                self._max_thrust * \
                self._thruster_joy_gain

            cmd_roll = msg.axes[self._joy_axis['axis_roll']]
            cmd_pitch = msg.axes[self._joy_axis['axis_pitch']]
            cmd_yaw = msg.axes[self._joy_axis['axis_yaw']]

            deadzone = 0.2

            if abs(cmd_roll) < deadzone:
                cmd_roll = 0.0

            if abs(cmd_pitch) < deadzone:
                cmd_pitch = 0.0

            if abs(cmd_yaw) < deadzone:
                cmd_yaw = 0.0

            rpy = np.array([
                cmd_roll,
                cmd_pitch,
                cmd_yaw
            ])

            fins = self._rpy_to_fins.dot(rpy)

            # Publish thruster
            thrust_msg = Float64()
            thrust_msg.data = thrust

            self._thruster_pub.publish(thrust_msg)

            # Publish fins
            for i in range(self._n_fins):
                fin_msg = Float64()
                fin_msg.data = float(fins[i])

                self._pub_cmd[i].publish(fin_msg)

        except Exception as e:
            self.get_logger().error(
                f'Joystick parsing error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = FinnedUUVControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
