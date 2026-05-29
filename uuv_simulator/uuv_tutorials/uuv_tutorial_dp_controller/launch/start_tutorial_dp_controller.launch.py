from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Launch configurations
    # -------------------------------------------------------------------------

    uuv_name = LaunchConfiguration('uuv_name')
    model_name = LaunchConfiguration('model_name')

    saturation = LaunchConfiguration('saturation')

    kp = LaunchConfiguration('Kp')
    kd = LaunchConfiguration('Kd')
    ki = LaunchConfiguration('Ki')

    output_dir = LaunchConfiguration('output_dir')
    config_file = LaunchConfiguration('config_file')
    tam_file = LaunchConfiguration('tam_file')

    # -------------------------------------------------------------------------
    # Package paths
    # -------------------------------------------------------------------------

    thruster_manager_launch = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch',
        'thruster_manager.launch.py'
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription([

        # ---------------------------------------------------------------------
        # Arguments
        # ---------------------------------------------------------------------

        DeclareLaunchArgument(
            'uuv_name'
        ),

        DeclareLaunchArgument(
            'model_name',
            default_value=uuv_name
        ),

        DeclareLaunchArgument(
            'saturation',
            default_value='5000'
        ),

        DeclareLaunchArgument(
            'Kp',
            default_value='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069'
        ),

        DeclareLaunchArgument(
            'Kd',
            default_value='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925'
        ),

        DeclareLaunchArgument(
            'Ki',
            default_value='321.417,321.417,321.417,2096.951,2096.951,2096.951'
        ),

        DeclareLaunchArgument(
            'output_dir',
            default_value=[
                get_package_share_directory('uuv_thruster_manager'),
                '/config/',
                model_name
            ]
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value=[
                get_package_share_directory('uuv_thruster_manager'),
                '/config/',
                model_name,
                '/thruster_manager.yaml'
            ]
        ),

        DeclareLaunchArgument(
            'tam_file',
            default_value=[
                get_package_share_directory('uuv_thruster_manager'),
                '/config/',
                model_name,
                '/TAM.yaml'
            ]
        ),

        # ---------------------------------------------------------------------
        # Thruster manager
        # ---------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(thruster_manager_launch),
            launch_arguments={
                'uuv_name': uuv_name,
                'model_name': model_name,
                'output_dir': output_dir,
                'config_file': config_file,
                'tam_file': tam_file
            }.items()
        ),

        # ---------------------------------------------------------------------
        # Vehicle namespace group
        # ---------------------------------------------------------------------

        GroupAction([

            PushRosNamespace(uuv_name),

            # -------------------------------------------------------------
            # Trajectory marker publisher
            # -------------------------------------------------------------

            Node(
                package='uuv_control_utils',
                executable='trajectory_marker_publisher.py',
                name='trajectory_marker_publisher',
                output='screen',
                remappings=[
                    ('trajectory', 'dp_controller/trajectory'),
                    ('waypoints', 'dp_controller/waypoints')
                ]
            ),

            # -------------------------------------------------------------
            # DP controller
            # -------------------------------------------------------------

            Node(
                package='uuv_tutorial_dp_controller',
                executable='tutorial_dp_controller.py',
                name='tutorial_dp_controller',
                output='screen',

                remappings=[
                    ('odom', 'pose_gt'),
                    ('trajectory', 'dp_controller/trajectory'),
                    ('input_trajectory', 'dp_controller/input_trajectory'),
                    ('waypoints', 'dp_controller/waypoints'),
                    ('error', 'dp_controller/error'),
                    ('reference', 'dp_controller/reference'),
                    ('thruster_output',
                     'thruster_manager/input_stamped')
                ],

                parameters=[{
                    'saturation': 5000.0,

                    'Kp': [
                        11993.888,
                        11993.888,
                        11993.888,
                        19460.069,
                        19460.069,
                        19460.069
                    ],

                    'Kd': [
                        9077.459,
                        9077.459,
                        9077.459,
                        18880.925,
                        18880.925,
                        18880.925
                    ],

                    'Ki': [
                        321.417,
                        321.417,
                        321.417,
                        2096.951,
                        2096.951,
                        2096.951
                    ]
                }]
            )
        ])
    ])
