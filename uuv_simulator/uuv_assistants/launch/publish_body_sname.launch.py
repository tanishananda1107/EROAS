from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    uuv_name = "rexrov"

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sname_frame_publisher",
            arguments=[
                "0", "0", "0",
                "0", "0", "3.141592653589793",
                f"{uuv_name}/base_link",
                f"{uuv_name}/base_link_ned"
            ]
        )
    ])
