from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    uuv_name = LaunchConfiguration("uuv_name")
    model_name = LaunchConfiguration("model_name")
    use_ned_frame = LaunchConfiguration("use_ned_frame")

    thruster_launch = os.path.join(
        get_package_share_directory(
            "uuv_thruster_manager"
        ),
        "launch",
        "thruster_manager.launch.py"
    )

    remaps = [
        ("trajectory","dp_controller/trajectory"),
        ("input_trajectory",
         "dp_controller/input_trajectory"),
        ("waypoints",
         "dp_controller/waypoints"),
        ("error",
         "dp_controller/error"),
        ("reference",
         "dp_controller/reference"),
        ("thruster_output",
         "thruster_manager/input_stamped")
    ]

    return LaunchDescription([

        DeclareLaunchArgument("uuv_name"),
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
                package=
                "uuv_trajectory_control",

                executable=
                "rov_mb_fl_controller",

                name=
                "rov_mb_fl_controller",

                remappings=
                [("odom","pose_gt")]
                + remaps,

                parameters=[{

                    "saturation":1200,

                    "Kp":[
                        19987.218,
                        19987.218,
                        19987.218,
                        19460.293,
                        19460.293,
                        19460.293
                    ],

                    "Kd":[
                        11458.051,
                        11458.051,
                        11458.051,
                        17068.778,
                        17068.778,
                        17068.778
                    ],

                    "Ki":[
                        1689.976,
                        1689.976,
                        1689.976,
                        186.198,
                        186.198,
                        186.198
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
                "rov_mb_fl_controller",

                name=
                "rov_mb_fl_controller",

                remappings=
                [("odom","pose_gt_ned")]
                + remaps,

                parameters=[{
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
