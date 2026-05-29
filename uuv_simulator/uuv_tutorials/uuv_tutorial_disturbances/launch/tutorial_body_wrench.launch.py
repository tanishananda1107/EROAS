from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    world_launch = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch',
        'empty_underwater_world.launch.py'
    )

    rexrov_launch = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        'upload_rexrov.launch.py'
    )

    controller_launch = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'launch',
        'rov_nmb_sm_controller.launch.py'
    )

    circular_traj_launch = os.path.join(
        get_package_share_directory('uuv_control_utils'),
        'launch',
        'start_circular_trajectory.launch.py'
    )

    body_wrench_launch = os.path.join(
        get_package_share_directory('uuv_control_utils'),
        'launch',
        'apply_body_wrench.launch.py'
    )

    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz',
        'controller_demo.rviz'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'set_timeout': 'true',
                'timeout': '105'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rexrov_launch),
            launch_arguments={
                'x': '0',
                'y': '0',
                'z': '-20',
                'yaw': '0'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov',
                'K': '5,5,5,5,5,5',
                'Kd': '4118.98,4118.98,4118.98,8000.0,8000.0,8000.0',
                'Ki': '0.06144,0.06144,0.06144,0.078,0.078,0.078',
                'slope': '0.182,0.182,0.182,3.348,3.348,3.348'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(circular_traj_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'radius': '5',
                'center_z': '-25',
                'max_forward_speed': '0.6'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(body_wrench_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'starting_time': '5',
                'duration': '10',
                'force_x': '3000',
                'force_y': '3000'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(body_wrench_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'starting_time': '20',
                'duration': '10',
                'force_y': '3000',
                'force_z': '3000'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(body_wrench_launch),
            launch_arguments={
                'uuv_name': 'rexrov',
                'starting_time': '35',
                'duration': '10',
                'torque_y': '3000',
                'torque_z': '3000'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
