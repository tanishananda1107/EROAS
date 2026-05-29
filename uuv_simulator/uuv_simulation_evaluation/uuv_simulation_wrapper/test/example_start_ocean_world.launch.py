from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():

    record = DeclareLaunchArgument('record', default_value='false')
    timeout = DeclareLaunchArgument('timeout', default_value='20')
    gui = DeclareLaunchArgument('gui', default_value='false')
    bag_filename = DeclareLaunchArgument(
        'bag_filename',
        default_value='recording'
    )

    kp = DeclareLaunchArgument(
        'Kp',
        default_value='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069'
    )

    kd = DeclareLaunchArgument(
        'Kd',
        default_value='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925'
    )

    ki = DeclareLaunchArgument(
        'Ki',
        default_value='321.417,321.417,321.417,2096.951,2096.951,2096.951'
    )

    ocean_waves = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_descriptions'),
                'launch',
                'ocean_waves.launch.py'
            )
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    controller_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_gazebo'),
                'launch',
                'controller_demos',
                'record_demo.launch.py'
            )
        ),
        launch_arguments={
            'record': LaunchConfiguration('record'),
            'bag_filename': LaunchConfiguration('bag_filename')
        }.items()
    )

    return LaunchDescription([
        record,
        timeout,
        gui,
        bag_filename,
        kp,
        kd,
        ki,
        ocean_waves,
        controller_demo
    ])
