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

    joy_id = LaunchConfiguration(
        'joy_id'
    )

    axis_yaw = LaunchConfiguration(
        'axis_yaw'
    )

    axis_x = LaunchConfiguration(
        'axis_x'
    )

    axis_y = LaunchConfiguration(
        'axis_y'
    )

    axis_z = LaunchConfiguration(
        'axis_z'
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

        DeclareLaunchArgument(
            'joy_id',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'axis_yaw',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'axis_x',
            default_value='4'
        ),

        DeclareLaunchArgument(
            'axis_y',
            default_value='3'
        ),

        DeclareLaunchArgument(
            'axis_z',
            default_value='1'
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
                'x': '20',
                'y': '0',
                'z': '-20',
                'yaw': '0',
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
                    'rov_nmb_sm_controller.launch.py'
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'uuv_teleop'
                    ),
                    'launch',
                    'uuv_teleop.launch.py'
                )
            ),

            launch_arguments={
                'uuv_name': 'rexrov',
                'joy_id': joy_id,
                'output_topic': 'cmd_vel',
                'message_type': 'twist',
                'axis_yaw': axis_yaw,
                'axis_x': axis_x,
                'axis_y': axis_y,
                'axis_z': axis_z,
                'gain_yaw': '0.2',
                'gain_x': '0.5',
                'gain_y': '0.5',
                'gain_z': '0.5'
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
