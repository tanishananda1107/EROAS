from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(

            package=
            "uuv_trajectory_control",

            executable=
            "rov_nl_pid_controller",

            parameters=[{

                "saturation":
                6000,

                "Kp":[

                    6017.059667616178,
                    6017.059667616178,
                    6017.059667616178,

                    9731.391095849109,
                    9731.391095849109,
                    9731.391095849109
                ],

                "Kd":[

                    2682.9509286089447,
                    2682.9509286089447,
                    2682.9509286089447,

                    9440.462338720527,
                    9440.462338720527,
                    9440.462338720527
                ],

                "Ki":[0]*6,

                "Hm":[

                    1657.655912647713,
                    1657.655912647713,
                    1657.655912647713,

                    4193.901741127024,
                    4193.901741127024,
                    4193.901741127024
                ]
            }]
        )
    ])
