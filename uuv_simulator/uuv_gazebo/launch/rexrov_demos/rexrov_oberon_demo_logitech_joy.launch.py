from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    demo = os.path.join(
        get_package_share_directory(
            "uuv_gazebo"
        ),
        "launch/rexrov_demos",
        "rexrov_oberon_demo.launch.py"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                demo
            ),

            launch_arguments={

                "axis_yaw":"2",
                "axis_x":"1",
                "axis_y":"0",
                "axis_z":"5",

                "axis_oberon_jc_azimuth":"2",
                "axis_oberon_jc_shoulder":"1",

                "axis_oberon_jc_elbow":"0",

                "axis_oberon_jc_roll":"3",

                "axis_oberon_jc_pitch":"4",

                "axis_oberon_jc_yaw":"5"

            }.items()

        )

    ])
