#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import *

from launch_ros.actions import *

from launch.substitutions import *

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'model_name'
        ),

        DeclareLaunchArgument(
            'uuv_name',
            default_value=
            LaunchConfiguration(
                'model_name'
            )
        ),

        DeclareLaunchArgument(
            'joy_id',
            default_value='0'
        ),

        GroupAction([

            PushRosNamespace(
                LaunchConfiguration(
                    'uuv_name'
                )
            ),

            Node(

                package=
                'uuv_control_cascaded_pid',

                executable=
                'AccelerationControl',

                name=
                'acceleration_control',

                output='screen',

                parameters=[{

                    'tf_prefix':
                    LaunchConfiguration(
                        'uuv_name'
                    )
                }]
            ),

            Node(

                package=
                'uuv_control_cascaded_pid',

                executable=
                'VelocityControl',

                name=
                'velocity_control',

                output='screen',

                remappings=[

                    (

                        'odom',

                        [
                            '/',
                            LaunchConfiguration(
                                'uuv_name'
                            ),

                            '/pose_gt'
                        ]
                    ),

                    (

                        'cmd_accel',

                        [
                            '/',
                            LaunchConfiguration(
                                'uuv_name'
                            ),

                            '/cmd_accel'
                        ]
                    )
                ]
            )
        ])
    ])
