from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov"
        ),

        Node(
            package="uuv_assistants",
            executable="uuv_message_to_tf",
            name="ground_truth_to_tf",
            output="screen",

            parameters=[{
                "odometry_topic":
                    ["/", namespace, "/pose_gt"],

                "frame_id": "world",

                "stabilized_frame_id":
                    ["/", namespace, "/base_stabilized"],

                "footprint_frame_id":
                    ["/", namespace, "/base_footprint"],

                "child_frame_id":
                    ["/", namespace, "/base_link"]
            }]
        )
    ])
