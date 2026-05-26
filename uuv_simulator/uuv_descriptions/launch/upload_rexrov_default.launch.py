import os

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch_ros.actions import Node

from launch.substitutions import (
    LaunchConfiguration
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    namespace = LaunchConfiguration(
        "namespace"
    )

    xacro_file = os.path.join(

        FindPackageShare(
            "uuv_descriptions"
        ).find(
            "uuv_descriptions"
        ),

        "robots",

        "rexrov_default.xacro"
    )

    robot_desc = xacro.process_file(
        xacro_file
    ).toxml()

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
        ),

        Node(

            package="robot_state_publisher",

            executable=
            "robot_state_publisher",

            namespace=namespace,

            parameters=[{

                "robot_description":
                robot_desc,

                "publish_frequency":
                5.0
            }]
        ),

        Node(

            package=
            "gazebo_ros",

            executable=
            "spawn_entity.py",

            arguments=[

                "-topic",
                "robot_description",

                "-entity",
                "rexrov"
            ]
        ),

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    FindPackageShare(
                        "uuv_assistants"
                    ).find(
                        "uuv_assistants"
                    ),

                    "launch",

                    "message_to_tf.launch.py"
                )
            ),

            launch_arguments={

                "namespace":
                namespace

            }.items()
        )
    ])
