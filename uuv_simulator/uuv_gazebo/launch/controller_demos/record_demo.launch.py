#!/usr/bin/env python3
"""
Launch file for recording simulation data.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration."""
    record = LaunchConfiguration('record').perform(context)
    bag_filename = LaunchConfiguration('bag_filename').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    actions = []

    # Only record if record is true
    if record == 'true':
        # Determine topic based on use_ned_frame
        if use_ned_frame == 'true':
            topics = [
                '/rexrov/dp_controller/trajectory',
                '/rexrov/dp_controller/reference',
                '/rexrov/pose_gt_ned',
                '/hydrodynamics/current_velocity',
                '/rexrov/thruster_manager/input',
                '/rexrov/wrench_perturbation',
                '/rexrov/thrusters/0/thrust',
                '/rexrov/thrusters/1/thrust',
                '/rexrov/thrusters/2/thrust',
                '/rexrov/thrusters/3/thrust',
                '/rexrov/thrusters/4/thrust',
                '/rexrov/thrusters/5/thrust',
                '/rexrov/thrusters/6/thrust',
                '/rexrov/thrusters/7/thrust',
            ]
        else:
            topics = [
                '/rexrov/dp_controller/trajectory',
                '/rexrov/dp_controller/reference',
                '/rexrov/pose_gt',
                '/hydrodynamics/current_velocity',
                '/rexrov/thruster_manager/input',
                '/rexrov/wrench_perturbation',
                '/rexrov/thrusters/0/thrust',
                '/rexrov/thrusters/1/thrust',
                '/rexrov/thrusters/2/thrust',
                '/rexrov/thrusters/3/thrust',
                '/rexrov/thrusters/4/thrust',
                '/rexrov/thrusters/5/thrust',
                '/rexrov/thrusters/6/thrust',
                '/rexrov/thrusters/7/thrust',
            ]

        actions.append(Node(
            package='rosbag2_py',
            executable='record',
            name='recording',
            output='screen',
            arguments=['-s', bag_filename, '--topics'] + topics,
            condition=IfCondition(condition=True)
        ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false', description='Record rosbag'),
        DeclareLaunchArgument('bag_filename', default_value='recording', description='Bag filename'),
        DeclareLaunchArgument('use_ned_frame', default_value='false', description='Use NED frame'),

        OpaqueFunction(function=launch_setup),
    ])
