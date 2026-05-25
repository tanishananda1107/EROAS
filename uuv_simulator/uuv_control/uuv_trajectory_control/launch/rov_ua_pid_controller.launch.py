from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_params_file = LaunchConfiguration(
        "use_params_file"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_params_file",
            default_value="false"
        ),

        Node(

            package=
            "uuv_trajectory_control",

            executable=
            "rov_ua_pid_controller",

            name=
            "rov_ua_pid_controller",

            remappings=[

                ("odom",
                 "pose_gt"),

                (
                    "trajectory",
                    "dp_controller/trajectory"
                ),

                (
                    "input_trajectory",
                    "dp_controller/input_trajectory"
                ),

                (
                    "waypoints",
                    "dp_controller/waypoints"
                ),

                (
                    "error",
                    "dp_controller/error"
                ),

                (
                    "reference",
                    "dp_controller/reference"
                ),

                (
                    "thruster_output",
                    "thruster_manager/input_stamped"
                )
            ],

            parameters=[{

                "saturation":
                1200,

                "Kp":[
                    10,
                    10,
                    10,
                    10
                ],

                "Kd":[
                    1,
                    1,
                    1,
                    1
                ],

                "Ki":[
                    0.5,
                    0.5,
                    0.5,
                    0.5
                ]
            }],

            condition=
            UnlessCondition(
                use_params_file
            )
        ),

        Node(

            package=
            "uuv_trajectory_control",

            executable=
            "rov_ua_pid_controller",

            parameters=[
                "/config/controllers/ua_pid/params.yaml"
            ],

            condition=
            IfCondition(
                use_params_file
            )
        )
    ])
