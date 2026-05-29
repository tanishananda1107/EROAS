from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction
)

from launch.conditions import IfCondition, UnlessCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    args = [
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('timeout', default_value='60'),

        DeclareLaunchArgument('current_on', default_value='true'),

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
        DeclareLaunchArgument('center_z', default_value='-20'),
        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('n_turns', default_value='1'),
        DeclareLaunchArgument('delta_z', default_value='4.0'),
        DeclareLaunchArgument('heading_offset', default_value='0'),
        DeclareLaunchArgument('duration', default_value='60'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.5'),

        DeclareLaunchArgument('unpause_timeout', default_value='5')
    ]

    # ------------------------------------------------------------------
    # Empty underwater world
    # ------------------------------------------------------------------

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'empty_underwater_world.launch.py'
            ])
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'paused': 'true',
            'set_timeout': 'true',
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    # ------------------------------------------------------------------
    # Unpause simulation
    # ------------------------------------------------------------------

    unpause = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_simulation_wrapper'),
                'launch',
                'unpause_simulation.launch.py'
            ])
        ),
        launch_arguments={
            'timeout': LaunchConfiguration('unpause_timeout')
        }.items()
    )

    # ------------------------------------------------------------------
    # Spawn REXROV2 with GUI
    # ------------------------------------------------------------------

    rexrov_gui = GroupAction(
        condition=IfCondition(LaunchConfiguration('gui')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('rexrov2_description'),
                        'launch',
                        'upload_rexrov2.launch.py'
                    ])
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

    # ------------------------------------------------------------------
    # Spawn simplified REXROV2 without GUI
    # ------------------------------------------------------------------

    rexrov_headless = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gui')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('rexrov2_description'),
                        'launch',
                        'upload_rexrov2.launch.py'
                    ])
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

    # ------------------------------------------------------------------
    # PID Controller
    # ------------------------------------------------------------------

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rexrov2_control'),
                'launch',
                'start_pid_controller.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': 'rexrov2',
            'Kp': LaunchConfiguration('Kp'),
            'Kd': LaunchConfiguration('Kd'),
            'Ki': LaunchConfiguration('Ki'),
            'teleop_on': LaunchConfiguration('teleop_on'),
            'joy_id': LaunchConfiguration('joy_id'),
            'gui_on': LaunchConfiguration('gui')
        }.items()
    )

    # ------------------------------------------------------------------
    # Helical trajectory
    # ------------------------------------------------------------------

    trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'start_helical_trajectory.launch.py'
            ])
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
            'heading_offset': '0',
            'duration': LaunchConfiguration('duration'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed')
        }.items()
    )

    # ------------------------------------------------------------------
    # Current perturbation velocity
    # ------------------------------------------------------------------

    current_velocity = Node(
        package='uuv_control_utils',
        executable='set_gm_current_perturbation',
        name='set_gm_velocity',
        output='screen',
        condition=IfCondition(LaunchConfiguration('current_on')),
        parameters=[{
            'component': 'velocity',
            'mean': 0.4,
            'min': 0.3,
            'max': 0.5,
            'noise': 0.005,
            'mu': 0.01
        }]
    )

    # ------------------------------------------------------------------
    # Current perturbation horizontal angle
    # ------------------------------------------------------------------

    current_angle = Node(
        package='uuv_control_utils',
        executable='set_gm_current_perturbation',
        name='set_gm_horz_angle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('current_on')),
        parameters=[{
            'component': 'horz_angle',
            'mean': 0.0,
            'min': -5.0,
            'max': 5.0,
            'noise': 0.005,
            'mu': 0.01
        }]
    )

    # ------------------------------------------------------------------
    # ROS2 bag recording
    # ------------------------------------------------------------------

    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_filename'),

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

    return LaunchDescription(
        args + [
            world,
            unpause,
            rexrov_gui,
            rexrov_headless,
            controller,
            trajectory,
            current_velocity,
            current_angle,
            rosbag_record
        ]
    )
