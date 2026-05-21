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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('uuv_gazebo_worlds'), 'launch', 'ocean_waves.launch.py'])])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('uuv_descriptions'), 'launch', 'upload_rexrov.launch.py'])]),
            launch_arguments={
                'x': '20',
                'y': '0',
                'z': '-20',
                'yaw': '0'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('uuv_tutorial_dp_controller'), 'launch', 'start_tutorial_dp_controller.launch.py'])]),
            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('uuv_gazebo'), 'launch', 'controller_demos', 'record_demo.launch.py'])]),
            launch_arguments={
                'record': 'false'
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('uuv_gazebo'), 'rviz', 'controller_demo.rviz'])]
        )
    ])
