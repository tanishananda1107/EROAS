# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions


def generate_launch_description():
    pkg = get_package_share_directory('uuv_gazebo_ros_plugins')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', 'localhost'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': os.path.join(
                    pkg, 'test', 'worlds', 'test_empty.sdf') + ' -r',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg, 'test', 'models',
                    'thrusters',
                    'test_upload_thrusters.launch.py')),
        ),

        Node(
            package='uuv_gazebo_ros_plugins',
            executable='test_thrusters',
            name='test_thrusters',
            output='screen',
        ),

        launch_testing.actions.ReadyToTest(),
    ])
