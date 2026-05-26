import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare(
        "uuv_descriptions"
    ).find(
        "uuv_descriptions"
    )

    file = os.path.join(

        pkg,

        "robots",

        "rexrov_oberon4.xacro"
    )

    robot = xacro.process_file(
        file
    ).toxml()

    return LaunchDescription([

        Node(

            package=
            "robot_state_publisher",

            executable=
            "robot_state_publisher",

            parameters=[{

                "robot_description":
                robot
            }]
        ),

        Node(

            package=
            "gazebo_ros",

            executable=
            "spawn_entity.py",

            arguments=[

                "-entity",

                "rexrov",

                "-topic",

                "robot_description"
            ]
        )
    ])
