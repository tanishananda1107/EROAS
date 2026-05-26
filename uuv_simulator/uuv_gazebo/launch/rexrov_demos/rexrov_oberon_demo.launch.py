from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.conditions import IfCondition, UnlessCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    use_jt = LaunchConfiguration("use_jt")

    upload = os.path.join(
        get_package_share_directory(
            "uuv_descriptions"
        ),
        "launch",
        "upload_rexrov_oberon7.launch.py"
    )

    jt = os.path.join(
        get_package_share_directory(
            "oberon7_control"
        ),
        "launch",
        "jt_cartesian_controller.launch.py"
    )

    jc = os.path.join(
        get_package_share_directory(
            "oberon7_control"
        ),
        "launch",
        "joint_control.launch.py"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_jt",
            default_value="false"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                upload
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(jt),
            condition=IfCondition(use_jt)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(jc),
            condition=UnlessCondition(use_jt)
        ),

        Node(
            package="rviz2",
            executable="rviz2"
        )

    ])
