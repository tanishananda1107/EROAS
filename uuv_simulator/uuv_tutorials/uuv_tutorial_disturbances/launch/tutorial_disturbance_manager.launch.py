from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_gazebo_worlds'),
                    'launch',
                    'empty_underwater_world.launch.py')),
            launch_arguments={
                'set_timeout': 'true',
                'timeout': '105'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_descriptions'),
                    'launch',
                    'upload_rexrov.launch.py')),
            launch_arguments={
                'x': '0',
                'y': '0',
                'z': '-20',
                'yaw': '0'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_trajectory_control'),
                    'launch',
                    'rov_nmb_sm_controller.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_control_utils'),
                    'launch',
                    'start_circular_trajectory.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov',
                'radius': '5',
                'center_z': '-25',
                'max_forward_speed': '0.6'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_control_utils'),
                    'launch',
                    'start_disturbance_manager.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov',
                'use_file': 'true',
                'disturbance_file': os.path.join(
                    get_package_share_directory(
                        'uuv_tutorial_disturbances'),
                    'config',
                    'disturbances.yaml')
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d',
                os.path.join(
                    get_package_share_directory('uuv_gazebo'),
                    'rviz',
                    'controller_demo.rviz')]
        )
    ])
