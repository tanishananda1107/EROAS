from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    record = LaunchConfiguration(
        'record'
    )

    use_ned_frame = LaunchConfiguration(
        'use_ned_frame'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'record',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'use_ned_frame',
            default_value='false'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_gazebo_worlds'
                    ),
                    'launch',
                    'ocean_waves.launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_descriptions'
                    ),
                    'launch',
                    'upload_rexrov.launch.py'
                )
            ),

            launch_arguments={
                'use_ned_frame':
                use_ned_frame
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_trajectory_control'
                    ),
                    'launch',
                    'rov_nl_pid_controller.launch.py'
                )
            ),

            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov',
                'use_ned_frame':
                use_ned_frame
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_gazebo'
                    ),
                    'launch',
                    'controller_demos',
                    'record_demo.launch.py'
                )
            ),

            launch_arguments={
                'record': record,
                'use_ned_frame':
                use_ned_frame
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',

            arguments=[
                '-d',
                os.path.join(
                    get_package_share_directory(
                        'uuv_gazebo'
                    ),
                    'rviz',
                    'controller_demo.rviz'
                )
            ],

            output='screen'
        )
    ])
