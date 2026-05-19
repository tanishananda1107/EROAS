#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory(
        'nps_uw_multibeam_sonar'
    )

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'sonar_tank_seabat_f50_nps_multibeam.world'
    )

    return LaunchDescription([

        ExecuteProcess(

            cmd=[
                'gz',
                'sim',
                '-r',
                world_file
            ],

            output='screen'
        ),

        Node(

            package='image_view',

            executable='image_view',

            remappings=[
                (
                    'image',
                    '/seabat_f50/sonar_image'
                )
            ],

            parameters=[
                {
                    'autosize': True
                }
            ],

            output='screen'
        ),

        Node(

            package='tf2_ros',

            executable='static_transform_publisher',

            arguments=[

                '0', '0', '0',
                '0', '0', '0',

                'world',
                'forward_sonar_optical_link'
            ],

            output='screen'
        ),
    ])
