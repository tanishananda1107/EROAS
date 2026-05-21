#!/usr/bin/env python3
"""
Launch file for message_to_tf node in uuv_descriptions.
Publishes transforms from odometry messages.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='rexrov',
            description='Namespace for the node'
        ),
        Node(
            package='uuv_assistants',
            executable='uuv_message_to_tf',
            name='ground_truth_to_tf_' + LaunchConfiguration('namespace'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'odometry_topic': PythonExpression(['"/", LaunchConfiguration("namespace"), "/pose_gt"]]),
                'frame_id': '/world',
                'stabilized_frame_id': PythonExpression(['"/", LaunchConfiguration("namespace"), "/base_stabilized"]]),
                'footprint_frame_id': PythonExpression(['"/", LaunchConfiguration("namespace"), "/base_footprint"]]),
                'child_frame_id': PythonExpression(['"/", LaunchConfiguration("namespace"), "/base_link"]]),
            }]
        ),
    ])
