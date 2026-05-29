# start_task.launch.py
#
# ROS 2 + Gazebo Harmonic / GZ Sim 8 conversion
#
# Usage:
# ros2 launch uuv_simulation_wrapper start_task.launch.py
#

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction
)

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------

    declared_arguments = [

        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording'),

        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('timeout', default_value='100'),

        DeclareLaunchArgument('current_on', default_value='false'),
        DeclareLaunchArgument('current_vel', default_value='0.0'),
        DeclareLaunchArgument('horizontal_angle', default_value='0.0'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),

        DeclareLaunchArgument(
            'K',
            default_value='5,5,5,5,5,5'
        ),

        DeclareLaunchArgument(
            'Kd',
            default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'
        ),

        DeclareLaunchArgument(
            'Ki',
            default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'
        ),

        DeclareLaunchArgument(
            'slope',
            default_value='0.182,0.182,0.182,3.348,3.348,3.348'
        ),

        DeclareLaunchArgument('teleop_on', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        DeclareLaunchArgument('radius', default_value='8'),
        DeclareLaunchArgument('center_x', default_value='0'),
        DeclareLaunchArgument('center_y', default_value='0'),
        DeclareLaunchArgument('center_z', default_value='-20'),

        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('n_turns', default_value='1'),
        DeclareLaunchArgument('delta_z', default_value='4.0'),

        DeclareLaunchArgument('heading_offset', default_value='0'),

        DeclareLaunchArgument('duration', default_value='100'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.5'),

        DeclareLaunchArgument('unpause_timeout', default_value='5'),
    ]

    # -------------------------------------------------------------------------
    # Package paths
    # -------------------------------------------------------------------------

    uuv_descriptions_pkg = get_package_share_directory('uuv_descriptions')

    uuv_simulation_wrapper_pkg = get_package_share_directory(
        'uuv_simulation_wrapper'
    )

    rexrov2_description_pkg = get_package_share_directory(
        'rexrov2_description'
    )

    rexrov2_control_pkg = get_package_share_directory(
        'rexrov2_control'
    )

    uuv_control_utils_pkg = get_package_share_directory(
        'uuv_control_utils'
    )

    # -------------------------------------------------------------------------
    # Empty underwater world
    # -------------------------------------------------------------------------

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
            'paused': 'true',
            'set_timeout': 'true',
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    # -------------------------------------------------------------------------
    # Unpause simulation
    # -------------------------------------------------------------------------

    unpause_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_simulation_wrapper_pkg,
                'launch',
                'unpause_simulation.launch.py'
            )
        ),
        launch_arguments={
            'timeout': LaunchConfiguration('unpause_timeout')
        }.items()
    )

    # -------------------------------------------------------------------------
    # Vehicle spawn (GUI)
    # -------------------------------------------------------------------------

    spawn_vehicle_gui = GroupAction(
        condition=IfCondition(LaunchConfiguration('gui')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        rexrov2_description_pkg,
                        'launch',
                        'upload_rexrov2.launch.py'
                    )
                ),
                launch_arguments={
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw')
                }.items()
            )
        ]
    )

    # -------------------------------------------------------------------------
    # Vehicle spawn (headless simplified mesh)
    # -------------------------------------------------------------------------

    spawn_vehicle_headless = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gui')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        rexrov2_description_pkg,
                        'launch',
                        'upload_rexrov2.launch.py'
                    )
                ),
                launch_arguments={
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw'),
                    'use_simplified_mesh': 'true'
                }.items()
            )
        ]
    )

    # -------------------------------------------------------------------------
    # Controller
    # -------------------------------------------------------------------------

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rexrov2_control_pkg,
                'launch',
                'start_nmb_sm_controller.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov2',
            'K': LaunchConfiguration('K'),
            'Kd': LaunchConfiguration('Kd'),
            'Ki': LaunchConfiguration('Ki'),
            'slope': LaunchConfiguration('slope'),
            'teleop_on': LaunchConfiguration('teleop_on'),
            'joy_id': LaunchConfiguration('joy_id'),
            'gui_on': LaunchConfiguration('gui')
        }.items()
    )

    # -------------------------------------------------------------------------
    # Helical trajectory generator
    # -------------------------------------------------------------------------

    helical_trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uuv_control_utils_pkg,
                'launch',
                'start_helical_trajectory.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov2',
            'radius': LaunchConfiguration('radius'),
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'center_z': LaunchConfiguration('center_z'),
            'n_points': LaunchConfiguration('n_points'),
            'n_turns': LaunchConfiguration('n_turns'),
            'delta_z': LaunchConfiguration('delta_z'),
            'heading_offset': LaunchConfiguration('heading_offset'),
            'duration': LaunchConfiguration('duration'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed')
        }.items()
    )

    # -------------------------------------------------------------------------
    # Current perturbation
    # -------------------------------------------------------------------------

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

    # -------------------------------------------------------------------------
    # ROS 2 bag recording
    # -------------------------------------------------------------------------

    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2',
            'bag',
            'record',

            '-o',
            LaunchConfiguration('bag_filename'),

            '/rexrov2/dp_controller/trajectory',
            '/rexrov2/dp_controller/reference',
            '/rexrov2/pose_gt',
            '/hydrodynamics/current_velocity',
            '/rexrov2/thruster_manager/input',
            '/rexrov2/wrench_perturbation',

            '/rexrov2/thrusters/0/thrust',
            '/rexrov2/thrusters/1/thrust',
            '/rexrov2/thrusters/2/thrust',
            '/rexrov2/thrusters/3/thrust',
            '/rexrov2/thrusters/4/thrust',
            '/rexrov2/thrusters/5/thrust'
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription(
        declared_arguments + [

            empty_world,
            unpause_sim,

            spawn_vehicle_gui,
            spawn_vehicle_headless,

            controller,
            helical_trajectory,
            current_perturbation,

            rosbag_record
        ]
    )
