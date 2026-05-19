from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    namespace = "rexrov"
    world_frame = "world"
    odom_topic = f"/{namespace}/pose_gt"

    return LaunchDescription([
        Node(
            package="uuv_assistants",
            executable="uuv_message_to_tf",
            name=f"ground_truth_to_tf_{namespace}",
            namespace=namespace,
            output="screen",
            parameters=[{
                "odometry_topic": odom_topic,
                "frame_id": world_frame,
                "stabilized_frame_id": f"/{namespace}/base_stabilized",
                "footprint_frame_id": f"/{namespace}/base_footprint",
                "child_frame_id": f"/{namespace}/base_link"
            }]
        )
    ])
