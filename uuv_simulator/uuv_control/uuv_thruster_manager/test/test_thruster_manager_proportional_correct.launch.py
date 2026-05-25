from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

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

            executable="thruster_allocator",

            namespace="test_vehicle"

        )
    ])
