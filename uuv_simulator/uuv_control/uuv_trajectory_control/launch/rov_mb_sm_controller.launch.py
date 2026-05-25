from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params = {

        "saturation":1200,

        "lambda":[10]*6,

        "rho_constant":[10000]*6,

        "k":[500]*6,

        "c":[50,50,50,1,1,1],

        "adapt_slope":[100,10,10],

        "rho_0":[
            3000,
            3000,
            8000,
            1500,
            1500,
            8000
        ],

        "drift_prevent":
        0.03
    }

    return LaunchDescription([

        Node(
            package=
            "uuv_trajectory_control",

            executable=
            "rov_mb_sm_controller",

            name=
            "rov_mb_sm_controller",

            parameters=[
                params
            ]
        )
    ])
