from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    record = LaunchConfiguration('record')

    # -------------------------------------------------------------------------
    # Package launch files
    # -------------------------------------------------------------------------

    ocean_waves_launch = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch',
        'ocean_waves.launch.py'
    )

    rexrov_launch = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        'upload_rexrov.launch.py'
    )

    controller_launch = os.path.join(
        get_package_share_directory('uuv_tutorial_dp_controller'),
        'launch',
        'start_tutorial_dp_controller.launch.py'
    )

    record_demo_launch = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'controller_demos',
        'record_demo.launch.py'
    )

    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz',
        'controller_demo.rviz'
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'record',
            default_value='false'
        ),

        # ---------------------------------------------------------------------
        # Ocean world
        # ---------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ocean_waves_launch)
        ),

        # ---------------------------------------------------------------------
        # Spawn RexROV
        # ---------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rexrov_launch),
            launch_arguments={
                'x': '20',
                'y': '0',
                'z': '-20',
                'yaw': '0'
            }.items()
        ),

        # ---------------------------------------------------------------------
        # Start controller
        # ---------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov'
            }.items()
        ),

        # ---------------------------------------------------------------------
        # Record demo
        # ---------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(record_demo_launch),
            launch_arguments={
                'record': record
            }.items()
        ),

        # ---------------------------------------------------------------------
        # RViz2
        # ---------------------------------------------------------------------

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                rviz_config
            ]
        )
    ])
