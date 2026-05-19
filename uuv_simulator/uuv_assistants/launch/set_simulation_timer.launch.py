from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("timeout"),

        Node(
            package="uuv_assistants",
            executable="set_simulation_timer.py",
            name="simulation_timeout",
            parameters=[{
                "timeout": LaunchConfiguration("timeout")
            }],
            output="screen"
        )
    ])
