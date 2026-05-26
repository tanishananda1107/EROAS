from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")

    upload = os.path.join(
        get_package_share_directory("uuv_descriptions"),
        "launch",
        "upload_rexrov_oberon_arms.launch.py"
    )

    teleop = os.path.join(
        get_package_share_directory("uuv_teleop"),
        "launch",
        "uuv_teleop.launch.py"
    )

    thruster = os.path.join(
        get_package_share_directory("uuv_thruster_manager"),
        "launch",
        "thruster_manager.launch.py"
    )

    oberon4 = os.path.join(
        get_package_share_directory("oberon4_control"),
        "launch",
        "joint_control.launch.py"
    )

    oberon7 = os.path.join(
        get_package_share_directory("oberon7_control"),
        "launch",
        "joint_control.launch.py"
    )

    return LaunchDescription([

        DeclareLaunchArgument("namespace", default_value="rexrov"),
        DeclareLaunchArgument("joy_id", default_value="0"),

        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="-20"),
        DeclareLaunchArgument("yaw", default_value="0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(thruster),
            launch_arguments={
                "uuv_name": namespace,
                "model_name": "rexrov"
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oberon4)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oberon7)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d",
                os.path.join(
                    get_package_share_directory("uuv_gazebo"),
                    "rviz",
                    "rexrov_default.rviz"
                )
            ]
        )

    ])
