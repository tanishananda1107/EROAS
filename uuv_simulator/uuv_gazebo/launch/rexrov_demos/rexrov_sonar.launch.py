from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    upload = os.path.join(
        get_package_share_directory(
            "uuv_descriptions"
        ),
        "launch",
        "upload_rexrov_default.launch.py"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload),

            launch_arguments={
                "mode":"sonar"
            }.items()
        ),

        Node(
            package="rviz2",
            executable="rviz2",

            arguments=[
                "-d",

                os.path.join(
                    get_package_share_directory(
                        "uuv_gazebo"
                    ),

                    "rviz",
                    "rexrov_sonar.rviz"
                )
            ]
        )

    ])
