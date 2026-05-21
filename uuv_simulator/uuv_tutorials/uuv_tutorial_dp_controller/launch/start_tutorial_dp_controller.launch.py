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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    saturation = LaunchConfiguration('saturation').perform(context)
    Kp = LaunchConfiguration('Kp').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)

    output_dir = PathJoinSubstitution([FindPackageShare('uuv_thruster_manager'), 'config', model_name])
    config_file = PathJoinSubstitution([FindPackageShare('uuv_thruster_manager'), 'config', model_name, 'thruster_manager.yaml'])
    tam_file = PathJoinSubstitution([FindPackageShare('uuv_thruster_manager'), 'config', model_name, 'TAM.yaml'])

    thruster_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('uuv_thruster_manager'), 'launch', 'thruster_manager.launch.py'])]),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name,
            'output_dir': output_dir,
            'config_file': config_file,
            'tam_file': tam_file
        }.items()
    )

    trajectory_marker_publisher = Node(
        package='uuv_control_utils',
        executable='trajectory_marker_publisher.py',
        name='trajectory_marker_publisher',
        output='screen',
        namespace=uuv_name,
        remappings=[
            ('trajectory', 'dp_controller/trajectory'),
            ('waypoints', 'dp_controller/waypoints')
        ]
    )

    tutorial_dp_controller = Node(
        package='uuv_tutorial_dp_controller',
        executable='tutorial_dp_controller.py',
        name='tutorial_dp_controller',
        output='screen',
        namespace=uuv_name,
        remappings=[
            ('odom', 'pose_gt'),
            ('trajectory', 'dp_controller/trajectory'),
            ('input_trajectory', 'dp_controller/input_trajectory'),
            ('waypoints', 'dp_controller/waypoints'),
            ('error', 'dp_controller/error'),
            ('reference', 'dp_controller/reference'),
            ('thruster_output', 'thruster_manager/input_stamped')
        ],
        parameters=[{
            'saturation': float(saturation),
            'Kp': [float(x) for x in Kp.split(',')],
            'Kd': [float(x) for x in Kd.split(',')],
            'Ki': [float(x) for x in Ki.split(',')]
        }]
    )

    return [
        thruster_manager_launch,
        trajectory_marker_publisher,
        tutorial_dp_controller
    ]


def launch_main():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
        LaunchConfiguration('uuv_name'),
        LaunchConfiguration('model_name', default='$(arg uuv_name)'),
        LaunchConfiguration('saturation', default='5000'),
        LaunchConfiguration('Kp', default='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069'),
        LaunchConfiguration('Kd', default='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925'),
        LaunchConfiguration('Ki', default='321.417,321.417,321.417,2096.951,2096.951,2096.951'),
    ])


if __name__ == '__main__':
    launch_main()
