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
        'local_search.world'
    )

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'multibeam_sonar_blueview_p900.xacro'
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

            package='robot_state_publisher',

            executable='robot_state_publisher',

            parameters=[{

                'robot_description': open(
                    urdf_file,
                    'r'
                ).read()
            }],

            output='screen'
        ),

        ExecuteProcess(

            cmd=[

                'ros2',
                'run',
                'ros_gz_sim',
                'create',

                '-name',
                'blueview_p900',

                '-topic',
                'robot_description',

                '-x',
                '6',

                '-y',
                '0',

                '-z',
                '-93'
            ],

            output='screen'
        ),

        Node(

            package='image_view',

            executable='image_view',

            name='image_view_sonar',

            remappings=[

                (
                    'image',
                    '/blueview_p900/sonar_image'
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

                '0',
                '0',
                '0',

                '0',
                '0',
                '0',

                'map',

                'forward_sonar_optical_link'
            ],

            output='screen'
        ),
    ])
