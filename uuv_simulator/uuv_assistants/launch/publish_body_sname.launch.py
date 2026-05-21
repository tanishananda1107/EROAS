#!/usr/bin/env python3

"""
Publish static transform:
base_link -> base_link_ned
"""

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'uuv_name'
        ),

        Node(

            package='tf2_ros',

            executable='static_transform_publisher',

            name='sname_frame_publisher',

            namespace=uuv_name,

            arguments=[

                '0',
                '0',
                '0',

                '0',
                '0',
                '3.141592653589793',

                ['/', uuv_name, '/base_link'],

                ['/', uuv_name, '/base_link_ned']

            ],

            output='screen'

        )

    ])
