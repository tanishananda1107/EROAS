from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'uuv_name'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_control_utils'
                    ),
                    'launch',
                    'start_disturbance_manager.launch.py'
                )
            ),

            launch_arguments={
                'uuv_name': uuv_name,
                'use_file': '1',
                'disturbance_file':
                os.path.join(
                    get_package_share_directory(
                        'uuv_gazebo'
                    ),
                    'config',
                    'disturbances',
                    'test_disturbances.yaml'
                )
            }.items()
        )
    ])
