from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_frame_publisher",
            arguments=[
                "0", "0", "0",
                "1.5707963267948966", "0", "3.141592653589793",
                "world",
                "world_ned"
            ]
        )
    ])
