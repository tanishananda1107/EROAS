from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mode = LaunchConfiguration("mode")

    return LaunchDescription([

        DeclareLaunchArgument(
            "debug",
            default_value="0"
        ),

        DeclareLaunchArgument(
            "x",
            default_value="0"
        ),

        DeclareLaunchArgument(
            "y",
            default_value="0"
        ),

        DeclareLaunchArgument(
            "z",
            default_value="-20"
        ),

        DeclareLaunchArgument(
            "roll",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "pitch",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "yaw",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "mode",
            default_value="default"
        ),

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
        ),

        DeclareLaunchArgument(
            "use_ned_frame",
            default_value="false"
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                PathJoinSubstitution([
                    FindPackageShare(
                        "uuv_descriptions"
                    ),

                    "launch",

                    [
                        "upload_rexrov_",
                        mode,
                        ".launch.py"
                    ]
                ])
            ),

            launch_arguments={

                "debug":
                    LaunchConfiguration(
                        "debug"
                    ),

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

                "roll":
                    LaunchConfiguration(
                        "roll"
                    ),

                "pitch":
                    LaunchConfiguration(
                        "pitch"
                    ),

                "yaw":
                    LaunchConfiguration(
                        "yaw"
                    ),

                "namespace":
                    LaunchConfiguration(
                        "namespace"
                    ),

                "use_ned_frame":
                    LaunchConfiguration(
                        "use_ned_frame"
                    )

            }.items()
        )
    ])
