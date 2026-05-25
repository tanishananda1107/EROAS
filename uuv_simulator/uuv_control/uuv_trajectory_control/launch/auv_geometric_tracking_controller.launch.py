from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    uuv_name = LaunchConfiguration("uuv_name")
    gui_on = LaunchConfiguration("gui_on")
    use_ned_frame = LaunchConfiguration("use_ned_frame")

    common_params = {
        "max_forward_speed": 2.0,
        "base_link": "base_link",
        "is_underactuated": True,
        "min_thrust": 70,
        "max_thrust": 200,
        "gain_roll": 1.0,
        "gain_pitch": 1.0,
        "gain_yaw": 1.0,
        "idle_radius": 10.0,
        "look_ahead_delay": 5.0
    }

    return LaunchDescription([

        DeclareLaunchArgument("uuv_name"),
        DeclareLaunchArgument(
            "gui_on",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "use_ned_frame",
            default_value="false"
        ),

        GroupAction([

            PushRosNamespace(uuv_name),

            Node(
                package="uuv_control_utils",
                executable="trajectory_marker_publisher",
                name="trajectory_marker_publisher",
                condition=IfCondition(gui_on)
            ),

            Node(
                package="uuv_trajectory_control",
                executable="auv_geometric_tracking_controller",
                name="auv_geometric_tracking_controller",
                remappings=[("odom","pose_gt")],
                parameters=[
                    common_params,
                    {
                        "inertial_frame_id":"world"
                    }
                ],
                condition=UnlessCondition(use_ned_frame)
            ),

            Node(
                package="uuv_trajectory_control",
                executable="auv_geometric_tracking_controller",
                name="auv_geometric_tracking_controller",
                remappings=[("odom","pose_gt_ned")],
                parameters=[
                    common_params,
                    {
                        "base_link":"base_link_ned",
                        "inertial_frame_id":"world_ned"
                    }
                ],
                condition=IfCondition(use_ned_frame)
            )
        ])
    ])
