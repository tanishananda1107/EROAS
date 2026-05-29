from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_control_utils'),
                    'launch',
                    'set_thruster_output_efficiency.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov',
                'starting_time': '10',
                'duration': '30',
                'efficiency': '0.2',
                'thruster_id': '2'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('uuv_control_utils'),
                    'launch',
                    'set_thruster_output_efficiency.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov',
                'starting_time': '10',
                'duration': '30',
                'efficiency': '0.8',
                'thruster_id': '6'
            }.items()
        )
    ])
