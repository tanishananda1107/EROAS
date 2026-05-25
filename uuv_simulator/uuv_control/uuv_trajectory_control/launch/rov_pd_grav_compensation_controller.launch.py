from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(

            package=
            "uuv_trajectory_control",

            executable=
            "rov_pd_grav_compensation_controller",

            parameters=[{

                "saturation":
                1200,

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
                ]
            }]
        )
    ])
