from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    sonar = os.path.join(
        get_package_share_directory(
            "uuv_gazebo"
        ),

        "launch/rexrov_demos",

        "rexrov_sonar.launch.py"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                sonar
            ),

            launch_arguments={

                "axis_yaw":"2",

                "axis_x":"1",

                "axis_y":"0",

                "axis_z":"5"

            }.items()

        )

    ])
