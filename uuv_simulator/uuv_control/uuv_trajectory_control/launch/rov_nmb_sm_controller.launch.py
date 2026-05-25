from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(

            package=
            "uuv_trajectory_control",

            executable=
            "rov_nmb_sm_controller",

            parameters=[{

                "saturation":
                1200,

                "max_forward_speed":
                0.5,

                "K":[5]*6,

                "Kd":[

                    4118.98,
                    4118.98,
                    4118.98,

                    8000,
                    8000,
                    8000
                ],

                "Ki":[

                    0.06144,
                    0.06144,
                    0.06144,

                    0.078,
                    0.078,
                    0.078
                ],

                "slope":[

                    0.182,
                    0.182,
                    0.182,

                    3.348,
                    3.348,
                    3.348
                ]
            }]
        )
    ])
