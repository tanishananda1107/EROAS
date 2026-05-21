#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)

from launch_ros.actions import Node, PushRosNamespace, SetParametersFromFile

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    model_name = LaunchConfiguration('model_name')
    uuv_name = LaunchConfiguration('uuv_name')
    joy_id = LaunchConfiguration('joy_id')

    inertial_yaml = os.path.join(
        get_package_share_directory(
            'uuv_control_cascaded_pid'
        ),
        'config',
        model_name.perform({}),
        'inertial.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'model_name'
        ),

        DeclareLaunchArgument(
            'uuv_name',
            default_value=model_name
        ),

        DeclareLaunchArgument(
            'joy_id',
            default_value='0'
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(
                    get_package_share_directory(
                        'uuv_thruster_manager'
                    ),
                    'launch',
                    'thruster_manager.launch.py'
                )
            ),

            launch_arguments={

                'uuv_name':uuv_name,

                'model_name':model_name

            }.items()
        ),

        GroupAction([

            PushRosNamespace(
                uuv_name
            ),

            SetParametersFromFile(
                inertial_yaml
            ),

            Node(

                package='uuv_control_cascaded_pid',

                executable='AccelerationControl',

                name='acceleration_control',

                output='screen',

                parameters=[{

                    'tf_prefix':
                    LaunchConfiguration(
                        'uuv_name'
                    )
                }]
            )
        ]),

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

                'uuv_name':uuv_name,

                'joy_id':joy_id,

                'output_topic':'cmd_accel',

                'message_type':'accel'

            }.items()
        )
    ])
