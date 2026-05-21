#!/usr/bin/env python3

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(

            package='uuv_assistants',

            executable='publish_footprints',

            name='publish_footprints',

            output='screen'

        )

    ])
