from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from launch.substitutions import (
    LaunchConfiguration
)

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    namespace = LaunchConfiguration(
        "namespace"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    get_package_share_directory(
                        "uuv_descriptions"
                    ),

                    "launch",

                    "upload_rexrov_oberon4.launch.py"
                )
            ),

            launch_arguments={

                "namespace":
                namespace,

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
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    get_package_share_directory(
                        "uuv_thruster_manager"
                    ),

                    "launch",

                    "thruster_manager.launch.py"
                )
            ),

            launch_arguments={

                "uuv_name":
                namespace,

                "model_name":
                "rexrov"

            }.items()
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    get_package_share_directory(
                        "uuv_teleop"
                    ),

                    "launch",

                    "uuv_teleop.launch.py"
                )
            ),

            launch_arguments={

                "uuv_name":
                namespace,

                "joy_id":
                LaunchConfiguration(
                    "joy_id"
                ),

                "output_topic":
                "cmd_vel",

                "message_type":
                "twist"

            }.items()
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    get_package_share_directory(
                        "oberon4_control"
                    ),

                    "launch",

                    "joint_control.launch.py"
                )
            ),

            launch_arguments={

                "uuv_name":
                namespace,

                "arm_name":
                "oberon4"

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

                    "rexrov_default.rviz"
                )
            ],

            output="screen"
        )
    ])
