import os
import xacro

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace = LaunchConfiguration(
        "namespace"
    )

    pkg = FindPackageShare(
        "uuv_descriptions"
    ).find(
        "uuv_descriptions"
    )

    file = os.path.join(

        pkg,

        "robots",

        "rexrov_default_noisy_pose.xacro"
    )

    robot = xacro.process_file(
        file
    ).toxml()

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
        ),

        Node(

            package=
            "robot_state_publisher",

            executable=
            "robot_state_publisher",

            namespace=namespace,

            parameters=[{

                "robot_description":
                robot
            }]
        ),

        Node(

            package=
            "ros_gz_sim",

            executable=
            "create",

            arguments=[

                "-topic",

                "robot_description",

                "-entity",

                "rexrov"
            ]
        )
    ])
