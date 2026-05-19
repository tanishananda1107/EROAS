import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    pkg = get_package_share_directory(
        'navigator_auv'
    )

    world_file = os.path.join(
        pkg,
        'worlds',
        'underwater.sdf'
    )

    model_path = os.path.join(
        pkg,
        'models'
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            model_path
        ),

        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-r',
                world_file
            ],
            output='screen'
        ),

        TimerAction(

            period=3.0,

            actions=[

                ExecuteProcess(

                    cmd=[

                        'gz',
                        'service',

                        '-s',
                        '/world/underwater/create',

                        '--reqtype',
                        'gz.msgs.EntityFactory',

                        '--reptype',
                        'gz.msgs.Boolean',

                        '--timeout',
                        '5000',

                        '--req',

                        f'sdf_filename: "{os.path.join(pkg, "models/rexrov2/model.sdf")}" '
                        'name: "rexrov2" '
                        'pose { position { x: 0 y: 0 z: -2 } }'
                    ],

                    output='screen'
                )
            ]
        ),

        TimerAction(

            period=5.0,

            actions=[

                Node(

                    package='ros_gz_bridge',

                    executable='parameter_bridge',

                    name='ros_gz_bridge',

                    arguments=[

                        '/rexrov2/pose_gt'
                        '@nav_msgs/msg/Odometry'
                        '[gz.msgs.Odometry',

                        '/rexrov2/cmd_vel'
                        '@geometry_msgs/msg/Twist'
                        ']gz.msgs.Twist',

                        '/rexrov2/sonar_pointcloud'
                        '@sensor_msgs/msg/PointCloud2'
                        '[gz.msgs.PointCloudPacked',

                        '/clock'
                        '@rosgraph_msgs/msg/Clock'
                        '[gz.msgs.Clock',
                    ],

                    output='screen'
                )
            ]
        ),

        TimerAction(

            period=8.0,

            actions=[

                Node(
                    package='navigator_auv',
                    executable='grid_detection.py',
                    name='grid_detection',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='sonar_reconstruction.py',
                    name='sonar_reconstruction',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='pose_plotter.py',
                    name='pose_plotter',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='contour_heading.py',
                    name='contour_heading',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='cbf_implementation.py',
                    name='cbf_implementation',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='velocity_cbf.py',
                    name='velocity_cbf',
                    output='screen'
                ),

                Node(
                    package='navigator_auv',
                    executable='turn_xy_cbf.py',
                    name='turn_xy_cbf',
                    output='screen'
                ),
            ]
        ),
    ])
