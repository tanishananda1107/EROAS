# start_simulation.launch.py
#
# ROS 2 + Gazebo Harmonic + GZ Sim 8 version
#
# Usage:
# ros2 launch uuv_batch_run_example start_simulation.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # ----------------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------------

    declared_arguments = [

        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording'),

        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('timeout', default_value='70'),

        DeclareLaunchArgument('current_on', default_value='false'),
        DeclareLaunchArgument('current_vel', default_value='0.0'),
        DeclareLaunchArgument('horizontal_angle', default_value='0.0'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),

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

        DeclareLaunchArgument('teleop_on', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        DeclareLaunchArgument('radius', default_value='4'),
        DeclareLaunchArgument('center_x', default_value='0'),
        DeclareLaunchArgument('center_y', default_value='0'),
        DeclareLaunchArgument('center_z', default_value='20'),

        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('n_turns', default_value='1'),
        DeclareLaunchArgument('delta_z', default_value='2.0'),

        DeclareLaunchArgument('heading_offset', default_value='0'),

        DeclareLaunchArgument('duration', default_value='60'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.5'),

        DeclareLaunchArgument('unpause_timeout', default_value='5'),
    ]

    # ----------------------------------------------------------------------
    # Package paths
    # ----------------------------------------------------------------------

    uuv_descriptions_pkg = get_package_share_directory(
        'uuv_descriptions'
    )

    uuv_simulation_wrapper_pkg = get_package_share_directory(
        'uuv_simulation_wrapper'
    )

    uuv_assistants_pkg = get_package_share_directory(
        'uuv_assistants'
    )

    uuv_trajectory_control_pkg = get_package_share_directory(
        'uuv_trajectory_control'
    )

    uuv_control_utils_pkg = get_package_share_directory(
        'uuv_control_utils'
    )

    uuv_gazebo_pkg = get_package_share_directory(
        'uuv_gazebo'
    )

    # ----------------------------------------------------------------------
    # Empty underwater world
    # ----------------------------------------------------------------------

    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_descriptions_pkg,
                'launch',
                'empty_underwater_world.launch.py'
            )
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'paused': 'true'
        }.items()
    )

    # ----------------------------------------------------------------------
    # Simulation timer
    # ----------------------------------------------------------------------

    simulation_timer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_simulation_wrapper_pkg,
                'launch',
                'set_simulation_timer.launch.py'
            )
        ),
        launch_arguments={
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    # ----------------------------------------------------------------------
    # Unpause simulation
    # ----------------------------------------------------------------------

    unpause_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_assistants_pkg,
                'launch',
                'unpause_simulation.launch.py'
            )
        ),
        launch_arguments={
            'timeout': LaunchConfiguration('unpause_timeout')
        }.items()
    )

    # ----------------------------------------------------------------------
    # Spawn RexROV
    # ----------------------------------------------------------------------

    spawn_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_descriptions_pkg,
                'models',
                'rexrov',
                'launch',
                'upload_rexrov.launch.py'
            )
        ),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_ned_frame': 'true'
        }.items()
    )

    # ----------------------------------------------------------------------
    # PID controller
    # ----------------------------------------------------------------------

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_trajectory_control_pkg,
                'launch',
                'rov_pid_controller.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'Kp': LaunchConfiguration('Kp'),
            'Kd': LaunchConfiguration('Kd'),
            'Ki': LaunchConfiguration('Ki'),
            'use_ned_frame': 'true'
        }.items()
    )

    # ----------------------------------------------------------------------
    # Helical trajectory
    # ----------------------------------------------------------------------

    helical_trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_control_utils_pkg,
                'launch',
                'start_helical_trajectory.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'radius': LaunchConfiguration('radius'),
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'center_z': LaunchConfiguration('center_z'),
            'n_points': LaunchConfiguration('n_points'),
            'n_turns': LaunchConfiguration('n_turns'),
            'delta_z': LaunchConfiguration('delta_z'),
            'heading_offset': LaunchConfiguration('heading_offset'),
            'duration': LaunchConfiguration('duration'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed'),
            'start_time': '-1'
        }.items()
    )

    # ----------------------------------------------------------------------
    # Current perturbation
    # ----------------------------------------------------------------------

    current_perturbation = GroupAction(
        condition=IfCondition(LaunchConfiguration('current_on')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        uuv_control_utils_pkg,
                        'launch',
                        'set_timed_current_perturbation.launch.py'
                    )
                ),
                launch_arguments={
                    'starting_time': '0.0',
                    'end_time': '-1',
                    'current_vel': LaunchConfiguration('current_vel'),
                    'horizontal_angle': LaunchConfiguration(
                        'horizontal_angle'
                    )
                }.items()
            )
        ]
    )

    # ----------------------------------------------------------------------
    # ROS 2 bag recording
    # ----------------------------------------------------------------------

    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2',
            'bag',
            'record',

            '-o',
            LaunchConfiguration('bag_filename'),

            '/rexrov/dp_controller/trajectory',
            '/rexrov/dp_controller/reference',
            '/rexrov/pose_gt_ned',
            '/hydrodynamics/current_velocity',
            '/rexrov/thruster_manager/input',
            '/rexrov/wrench_perturbation',

            '/rexrov/thrusters/0/thrust',
            '/rexrov/thrusters/1/thrust',
            '/rexrov/thrusters/2/thrust',
            '/rexrov/thrusters/3/thrust',
            '/rexrov/thrusters/4/thrust',
            '/rexrov/thrusters/5/thrust',
            '/rexrov/thrusters/6/thrust',
            '/rexrov/thrusters/7/thrust'
        ],
        output='screen'
    )

    # ----------------------------------------------------------------------
    # RViz2
    # ----------------------------------------------------------------------

    rviz2 = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('gui')),
        cmd=[
            'rviz2',
            '-d',
            os.path.join(
                uuv_gazebo_pkg,
                'rviz',
                'controller_demo.rviz'
            )
        ],
        output='screen'
    )

    # ----------------------------------------------------------------------
    # Launch description
    # ----------------------------------------------------------------------

    return LaunchDescription(

        declared_arguments +

        [
            empty_world,
            simulation_timer,
            unpause_simulation,

            spawn_rexrov,
            pid_controller,

            helical_trajectory,
            current_perturbation,

            rosbag_record,
            rviz2
        ]
    )
