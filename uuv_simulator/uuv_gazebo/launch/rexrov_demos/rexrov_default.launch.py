from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.conditions import IfCondition

from launch.substitutions import (
    LaunchConfiguration
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    namespace = LaunchConfiguration(
        "namespace"
    )

    launch_rviz = LaunchConfiguration(
        "launch_rviz"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
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
            default_value="-70"
        ),

        DeclareLaunchArgument(
            "yaw",
            default_value="0"
        ),

        DeclareLaunchArgument(
            "joy_id",
            default_value="0"
        ),

        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "uuv_descriptions"
                    ),
                    "launch",
                    "upload_rexrov.launch.py"
                )
            ),

            launch_arguments={
                "mode": "default",
                "namespace": namespace,
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "yaw": LaunchConfiguration("yaw")
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
                "uuv_name": namespace,
                "model_name": "rexrov"
            }.items()
        ),

        Node(
            package="uuv_control_cascaded_pid",
            executable="AccelerationControl",

            namespace=namespace,

            name="acceleration_control",

            parameters=[
                {
                    "tf_prefix":
                    namespace
                },

                os.path.join(
                    get_package_share_directory(
                        "uuv_control_cascaded_pid"
                    ),

                    "config",
                    "rexrov",
                    "inertial.yaml"
                )
            ],

            output="screen"
        ),

        Node(
            package="uuv_control_cascaded_pid",

            executable="VelocityControl",

            namespace=namespace,

            name="velocity_control",

            parameters=[
                os.path.join(
                    get_package_share_directory(
                        "uuv_control_cascaded_pid"
                    ),

                    "config",
                    "rexrov",
                    "vel_pid_control.yaml"
                )
            ],

            remappings=[
                (
                    "odom",
                    f"/{namespace}/pose_gt"
                ),

                (
                    "cmd_accel",
                    f"/{namespace}/cmd_accel"
                )
            ],

            output="screen"
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
                "uuv_name": namespace,
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

            condition=
            IfCondition(
                launch_rviz
            ),

            output="screen"
        )
    ])
