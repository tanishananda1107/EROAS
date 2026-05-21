#!/usr/bin/env python3
"""
ROS2 conversion of unpause_simulation.launch

ROS1 original:
  <arg name="timeout" default="0"/>
  <node name="unpause_simulation"
        pkg="uuv_assistants"
        type="unpause_simulation.py"
        output="screen">
    <rosparam subst_value="true">
      timeout: $(arg timeout)
    </rosparam>
  </node>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── mirrors <arg name="timeout" default="0"/> ─────────────────────────
        DeclareLaunchArgument('timeout', default_value='0',
                              description='Delay before unpausing simulation (seconds)'),

        # ── mirrors <node> + <rosparam> ───────────────────────────────────────
        Node(
            package='uuv_assistants',
            executable='unpause_simulation.py',   # = type= in ROS1
            name='unpause_simulation',
            output='screen',
            parameters=[{                          # = <rosparam>
                'timeout': LaunchConfiguration('timeout'),
            }],
        ),
    ])
