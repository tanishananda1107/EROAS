#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Accel, Vector3
from sensor_msgs.msg import Joy


class VehicleTeleop(Node):
    def __init__(self):
        # Load the mapping for each input
        self._axes = dict(x=4, y=3, z=1,
                          roll=2, pitch=5, yaw=0,
                          xfast=-1, yfast=-1, zfast=-1,
                          rollfast=-1, pitchfast=-1, yawfast=-1)
        # Load the gain for each joystick axis input
        # (default values for the XBox 360 controller)
        self._axes_gain = dict(x=3, y=3, z=0.5,
                               roll=0.5, pitch=0.5, yaw=0.5,
                               xfast=6, yfast=6, zfast=1,
                               rollfast=2, pitchfast=2, yawfast=2)

        # Load parameters
        self._mapping = self.declare_parameter('mapping', {}).value
        if self._mapping:
            for tag in self._axes:
                if tag not in self._mapping:
                    self.get_logger().info(f'Tag not found in axes mapping, tag={tag}')
                else:
                    if 'axis' in self._mapping[tag]:
                        self._axes[tag] = self._mapping[tag]['axis']
                    if 'gain' in self._mapping[tag]:
                        self._axes_gain[tag] = self._mapping[tag]['gain']

        # Dead zone: Force values close to 0 to 0
        # (Recommended for imprecise controllers)
        self._deadzone = float(self.declare_parameter('deadzone', 0.5).value)

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = int(self.declare_parameter('deadman_button', -1).value)

        # If these buttons are pressed, the arm will not move
        self._exclusion_buttons = self.declare_parameter('exclusion_buttons', []).value
        if isinstance(self._exclusion_buttons, (float, int)):
            self._exclusion_buttons = [int(self._exclusion_buttons)]
        elif isinstance(self._exclusion_buttons, list):
            for n in self._exclusion_buttons:
                if not isinstance(n, (float, int)):
                    raise Exception(
                        'Exclusion buttons must be an integer index to '
                        'the joystick button')

        # Default for the start button of the XBox 360 controller
        self._home_button = int(self.declare_parameter('home_button', 7).value)

        self._msg_type = self.declare_parameter('type', 'twist').value
        if self._msg_type not in ['twist', 'accel']:
            raise Exception('Teleoperation output must be either twist or accel')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        if self._msg_type == 'twist':
            self._output_pub = self.create_publisher(Twist, 'output', qos_profile)
        else:
            self._output_pub = self.create_publisher(Accel, 'output', qos_profile)

        self._home_pressed_pub = self.create_publisher(
            Bool, 'home_pressed', qos_profile)

        # Joystick topic subscriber
        self._joy_sub = self.create_subscription(
            Joy, 'joy', self._joy_callback, qos_profile)

    def _parse_joy(self, joy=None):
        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()
        if joy is not None:
            # Linear velocities:
            l = Vector3(0, 0, 0)

            if self._axes['x'] > -1 and abs(joy.axes[self._axes['x']]) > self._deadzone:
                l.x += self._axes_gain['x'] * joy.axes[self._axes['x']]

            if self._axes['y'] > -1 and abs(joy.axes[self._axes['y']]) > self._deadzone:
                l.y += self._axes_gain['y'] * joy.axes[self._axes['y']]

            if self._axes['z'] > -1 and abs(joy.axes[self._axes['z']]) > self._deadzone:
                l.z += self._axes_gain['z'] * joy.axes[self._axes['z']]

            if self._axes['xfast'] > -1 and abs(joy.axes[self._axes['xfast']]) > self._deadzone:
                l.x += self._axes_gain['xfast'] * joy.axes[self._axes['xfast']]

            if self._axes['yfast'] > -1 and abs(joy.axes[self._axes['yfast']]) > self._deadzone:
                l.y += self._axes_gain['yfast'] * joy.axes[self._axes['yfast']]

            if self._axes['zfast'] > -1 and abs(joy.axes[self._axes['zfast']]) > self._deadzone:
                l.z += self._axes_gain['zfast'] * joy.axes[self._axes['zfast']]

            # Angular velocities:
            a = Vector3(0, 0, 0)

            if self._axes['roll'] > -1 and abs(joy.axes[self._axes['roll']]) > self._deadzone:
                a.x += self._axes_gain['roll'] * joy.axes[self._axes['roll']]

            if self._axes['rollfast'] > -1 and abs(joy.axes[self._axes['rollfast']]) > self._deadzone:
                a.x += self._axes_gain['rollfast'] * joy.axes[self._axes['rollfast']]

            if self._axes['pitch'] > -1 and abs(joy.axes[self._axes['pitch']]) > self._deadzone:
                a.y += self._axes_gain['pitch'] * joy.axes[self._axes['pitch']]

            if self._axes['pitchfast'] > -1 and abs(joy.axes[self._axes['pitchfast']]) > self._deadzone:
                a.y += self._axes_gain['pitchfast'] * joy.axes[self._axes['pitchfast']]

            if self._axes['yaw'] > -1 and abs(joy.axes[self._axes['yaw']]) > self._deadzone:
                a.z += self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

            if self._axes['yawfast'] > -1 and abs(joy.axes[self._axes['yawfast']]) > self._deadzone:
                a.z += self._axes_gain['yawfast'] * joy.axes[self._axes['yawfast']]

            cmd.linear = l
            cmd.angular = a
        else:
            cmd.linear = Vector3(0, 0, 0)
            cmd.angular = Vector3(0, 0, 0)
        return cmd

    def _joy_callback(self, joy):
        # If any exclusion buttons are pressed, do nothing
        try:
            for n in self._exclusion_buttons:
                if joy.buttons[n] == 1:
                    cmd = self._parse_joy()
                    self._output_pub.publish(cmd)
                    return

            if self._deadman_button != -1:
                if joy.buttons[self._deadman_button] == 1:
                    cmd = self._parse_joy(joy)
                else:
                    cmd = self._parse_joy()
            else:
                cmd = self._parse_joy(joy)
            self._output_pub.publish(cmd)
            self._home_pressed_pub.publish(
                Bool(bool(joy.buttons[self._home_button])))
        except Exception as e:
            print('Error occurred while parsing joystick input,'
                  ' check if the joy_id corresponds to the joystick ' 
                  'being used. message={}'.format(e))

def main(args=None):
    rclpy.init(args=args)

    node_name = 'vehicle_teleop'
    print(f'Starting [{node_name}] node')

    node = VehicleTeleop()

    rclpy.spin(node)

    node.destroy_node()
    print(f'Shutting down [{node_name}] node')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
