from launch import LaunchDescription

from launch_ros.actions import Node

from launch.actions import ExecuteProcess

import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory(
        "uuv_thruster_manager"
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                "xacro",
                os.path.join(
                    pkg,
                    "test",
                    "test_vehicle_z_axis.urdf.xacro"
                )
            ]
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace="test_vehicle"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="test_vehicle"
        ),

        Node(
            package="uuv_thruster_manager",
            executable="thruster_allocator"
        )
    ])
