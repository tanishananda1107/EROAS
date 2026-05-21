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

                output='screen'
            ),

            Node(

                package=
                'uuv_control_cascaded_pid',

                executable=
                'VelocityControl',

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
                    )
                ]
            ),

            Node(

                package=
                'uuv_control_cascaded_pid',

                executable=
                'PositionControl',

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
                    )
                ]
            )
        ])
    ])
