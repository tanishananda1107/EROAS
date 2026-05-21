#!/usr/bin/env python3
"""
Launch file to publish state and tf for in relation to the world frame.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('world_frame', default_value='world',
                             description='World frame ID'),
        DeclareLaunchArgument('child_frame_id', default_value='$(var namespace)/base_link',
                             description='Child frame ID'),
        DeclareLaunchArgument('odometry_topic', default_value='$(var namespace)/pose_gt',
                             description='Odometry topic'),

        Node(
            package='uuv_assistants',
            executable='uuv_message_to_tf',
            name='ground_truth_to_tf_$(var namespace)',
            output='screen',
            namespace='$(var namespace)',
            parameters=[{
                'odometry_topic': LaunchConfiguration('odometry_topic'),
                'frame_id': LaunchConfiguration('world_frame'),
                'stabilized_frame_id': '/$(var namespace)/base_stabilized',
                'footprint_frame_id': '/$(var namespace)/base_footprint',
                'child_frame_id': LaunchConfiguration('child_frame_id'),
            }]
        ),
    ])
