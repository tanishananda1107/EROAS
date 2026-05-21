#!/usr/bin/env python3
"""
ROS2 conversion of publish_vehicle_footprint.launch

ROS1 original:
  <arg name="uuv_name" />
  <arg name="scale_footprint" default="10"/>
  <arg name="scale_label" default="10"/>
  <arg name="label_x_offset" default="60"/>
  <arg name="odom_topic" default="pose_gt"/>
  <group ns="$(arg uuv_name)">
    <node name="publish_footprints" pkg="uuv_assistants" type="publish_vehicle_footprint.py">
      <remap from="odom" to="$(arg odom_topic)"/>
      <rosparam> scale_footprint, scale_label, label_x_offset </rosparam>
    </node>
  </group>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── mirrors every <arg> tag exactly ─────────────────────────────────
        DeclareLaunchArgument('uuv_name',
                              description='Namespace of the UUV (required, no default)'),
        DeclareLaunchArgument('scale_footprint',  default_value='10'),
        DeclareLaunchArgument('scale_label',      default_value='10'),
        DeclareLaunchArgument('label_x_offset',   default_value='60'),
        DeclareLaunchArgument('odom_topic',        default_value='pose_gt'),

        # ── mirrors <group ns> + <node> + <remap> + <rosparam> ──────────────
        Node(
            package='uuv_assistants',
            executable='publish_vehicle_footprint.py',   # = type= in ROS1
            name='publish_footprints',
            output='screen',
            namespace=LaunchConfiguration('uuv_name'),   # = <group ns="...">
            remappings=[
                ('odom', LaunchConfiguration('odom_topic')),  # = <remap>
            ],
            parameters=[{                                # = <rosparam>
                'scale_footprint': LaunchConfiguration('scale_footprint'),
                'scale_label':     LaunchConfiguration('scale_label'),
                'label_x_offset':  LaunchConfiguration('label_x_offset'),
            }],
        ),
    ])
