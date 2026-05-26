from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    joy_id = LaunchConfiguration(
        "joy_id"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "joy_id",
            default_value="0"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "uuv_gazebo_worlds"
                    ),
                    "launch",
                    "rov_bop_panel.launch.py"
                )
            ),

            launch_arguments={
                "x": "2.5",
                "y": "0",
                "z": "-4",
                "angle": "3.14"
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "uuv_gazebo"
                    ),
                    "launch",
                    "rexrov_demos",
                    "rexrov_oberon_arms_demo.launch.py"
                )
            ),

            launch_arguments={
                "x": "0",
                "y": "0",
                "z": "-4",
                "joy_id": joy_id
            }.items()
        )
    ])
