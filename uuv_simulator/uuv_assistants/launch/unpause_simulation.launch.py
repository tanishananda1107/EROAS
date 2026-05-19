from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("timeout", default_value="0"),

        Node(
            package="uuv_assistants",
            executable="unpause_simulation.py",
            name="unpause_simulation",
            parameters=[{
                "timeout": LaunchConfiguration("timeout")
            }],
            output="screen"
        )
    ])
