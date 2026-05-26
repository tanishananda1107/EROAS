from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import (
    LaunchConfiguration
)

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    get_package_share_directory(
                        "uuv_gazebo"
                    ),

                    "launch",

                    "rexrov_demos",

                    "rexrov_default.launch.py"
                )
            ),

            launch_arguments={

                "namespace":
                LaunchConfiguration(
                    "namespace"
                ),

                "joy_id":
                LaunchConfiguration(
                    "joy_id"
                ),

                "axis_yaw": "2",

                "axis_x": "1",

                "axis_y": "0",

                "axis_z": "5",

                "x":
                LaunchConfiguration(
                    "x"
                ),

                "y":
                LaunchConfiguration(
                    "y"
                ),

                "z":
                LaunchConfiguration(
                    "z"
                ),

                "yaw":
                LaunchConfiguration(
                    "yaw"
                )

            }.items()
        )
    ])
