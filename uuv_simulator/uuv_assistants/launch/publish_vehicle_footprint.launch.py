from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("uuv_name"),
        DeclareLaunchArgument("scale_footprint", default_value="10"),
        DeclareLaunchArgument("scale_label", default_value="10"),
        DeclareLaunchArgument("label_x_offset", default_value="60"),
        DeclareLaunchArgument("odom_topic", default_value="pose_gt"),

        Node(
            package="uuv_assistants",
            executable="publish_vehicle_footprint.py",
            namespace=LaunchConfiguration("uuv_name"),
            name="publish_footprints",
            remappings=[
                ("odom", LaunchConfiguration("odom_topic"))
            ],
            parameters=[{
                "scale_footprint": LaunchConfiguration("scale_footprint"),
                "scale_label": LaunchConfiguration("scale_label"),
                "label_x_offset": LaunchConfiguration("label_x_offset"),
            }],
            output="screen"
        )
    ])
