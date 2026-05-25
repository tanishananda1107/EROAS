from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    uuv_name=LaunchConfiguration("uuv_name")
    model_name=LaunchConfiguration("model_name")
    use_ned_frame=LaunchConfiguration("use_ned_frame")

    thruster_launch=os.path.join(
        get_package_share_directory(
            "uuv_thruster_manager"
        ),
        "launch",
        "thruster_manager.launch.py"
    )

    remaps=[
        ("trajectory","dp_controller/trajectory"),
        ("input_trajectory","dp_controller/input_trajectory"),
        ("waypoints","dp_controller/waypoints"),
        ("error","dp_controller/error"),
        ("reference","dp_controller/reference"),
        (
            "thruster_output",
            "thruster_manager/input_stamped"
        )
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            "uuv_name"
        ),

        DeclareLaunchArgument(
            "model_name"
        ),

        DeclareLaunchArgument(
            "use_ned_frame",
            default_value="false"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                thruster_launch
            ),

            launch_arguments={
                "uuv_name":uuv_name,
                "model_name":model_name
            }.items()
        ),

        GroupAction([

            PushRosNamespace(
                uuv_name
            ),

            Node(
                package="navigator_auv",
                executable=
                "sonar_reconstruction",
                name=
                "sonar_reconstruction"
            ),

            Node(
                package=
                "uuv_control_utils",

                executable=
                "trajectory_marker_publisher",

                remappings=[
                    (
                        "trajectory",
                        "dp_controller/trajectory"
                    ),

                    (
                        "waypoints",
                        "dp_controller/waypoints"
                    )
                ]
            ),

            Node(
                package=
                "uuv_trajectory_control",

                executable=
                "rov_pid_controller",

                name=
                "rov_pid_controller",

                remappings=
                [("odom","pose_gt")]
                + remaps,

                parameters=[{
                    "saturation":1200,

                    "Kp":[
                        11993.888,
                        11993.888,
                        11993.888,
                        19460.069,
                        19460.069,
                        19460.069
                    ],

                    "Kd":[
                        9077.459,
                        9077.459,
                        9077.459,
                        18880.925,
                        18880.925,
                        18880.925
                    ],

                    "Ki":[
                        321.417,
                        321.417,
                        321.417,
                        2096.951,
                        2096.951,
                        2096.951
                    ],

                    "inertial_frame_id":
                    "world"
                }],

                condition=
                UnlessCondition(
                    use_ned_frame
                )
            ),

            Node(
                package=
                "uuv_trajectory_control",

                executable=
                "rov_pid_controller",

                name=
                "rov_pid_controller",

                remappings=
                [("odom","pose_gt_ned")]
                + remaps,

                parameters=[{
                    "saturation":1200,
                    "inertial_frame_id":
                    "world_ned"
                }],

                condition=
                IfCondition(
                    use_ned_frame
                )
            )
        ])
    ])
