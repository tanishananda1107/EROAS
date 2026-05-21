#!/usr/bin/env python3
"""
Launch file to publish state and TF for in relation to the world frame.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace'),

        Node(
            package='uuv_assistants',
            executable='uuv_message_to_tf',
            name=f'ground_truth_to_tf_{LaunchConfiguration("namespace")}',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'odometry_topic': f'/{LaunchConfiguration("namespace")}/pose_gt',
                'frame_id': '/world',
                'stabilized_frame_id': f'/{LaunchConfiguration("namespace")}/base_stabilized',
                'footprint_frame_id': f'/{LaunchConfiguration("namespace")}/base_footprint',
                'child_frame_id': f'/{LaunchConfiguration("namespace")}/base_link',
            }]
        ),
    ])
