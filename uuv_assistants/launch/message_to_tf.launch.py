#!/usr/bin/env python3
"""
Launch file for message_to_tf node.
Publishes transforms from odometry messages.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for message_to_tf."""
    namespace = LaunchConfiguration('namespace').perform(context)
    world_frame = LaunchConfiguration('world_frame').perform(context)
    child_frame_id = LaunchConfiguration('child_frame_id').perform(context)
    odometry_topic = LaunchConfiguration('odometry_topic').perform(context)

    actions = []

    # Construct default values for child_frame_id and odometry_topic
    if child_frame_id == '':
        child_frame_id = '/' + namespace + '/base_link'
    if odometry_topic == '':
        odometry_topic = '/' + namespace + '/pose_gt'

    # Node for ground truth to TF
    actions.append(Node(
        package='uuv_assistants',
        executable='uuv_message_to_tf',
        name='ground_truth_to_tf_' + namespace,
        namespace=namespace,
        output='screen',
        parameters=[{
            'odometry_topic': odometry_topic,
            'frame_id': world_frame,
            'stabilized_frame_id': '/' + namespace + '/base_stabilized',
            'footprint_frame_id': '/' + namespace + '/base_footprint',
            'child_frame_id': child_frame_id,
        }]
    ))

    return actions


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='rexrov',
            description='Namespace for the node'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='World frame ID'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='',
            description='Child frame ID'
        ),
        DeclareLaunchArgument(
            'odometry_topic',
            default_value='',
            description='Odometry topic'
        ),
    ])
