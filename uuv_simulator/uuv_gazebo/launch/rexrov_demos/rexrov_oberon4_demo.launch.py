#!/usr/bin/env python3
"""
Launch file for rexrov oberon4 demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration."""
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    joy_id = LaunchConfiguration('joy_id').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    axis_x = LaunchConfiguration('axis_x').perform(context)
    axis_y = LaunchConfiguration('axis_y').perform(context)
    axis_z = LaunchConfiguration('axis_z').perform(context)
    axis_yaw = LaunchConfiguration('axis_yaw').perform(context)
    deadman_button = LaunchConfiguration('deadman_button').perform(context)
    exclusion_buttons = LaunchConfiguration('exclusion_buttons').perform(context)
    axis_oberon_azimuth = LaunchConfiguration('axis_oberon_azimuth').perform(context)
    axis_oberon_shoulder = LaunchConfiguration('axis_oberon_shoulder').perform(context)
    axis_oberon_wrist = LaunchConfiguration('axis_oberon_wrist').perform(context)
    oberon_exclusion_button = LaunchConfiguration('oberon_exclusion_button').perform(context)
    oberon_deadman_button = LaunchConfiguration('oberon_deadman_button').perform(context)
    oberon_home_button = LaunchConfiguration('oberon_home_button').perform(context)
    gripper_open_button = LaunchConfiguration('gripper_open_button').perform(context)
    gripper_close_button = LaunchConfiguration('gripper_close_button').perform(context)

    actions = []

    # Include upload_rexrov_oberon4
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'upload_rexrov_oberon4.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    ))

    # Include thruster_manager
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': namespace,
            'model_name': 'rexrov',
        }.items(),
    ))

    # Acceleration control node
    actions.append(Node(
        package='uuv_control_cascaded_pid',
        executable='AccelerationControl.py',
        name='acceleration_control',
        output='screen',
        parameters=[{
            'tf_prefix': f'{namespace}/'
        }]
    ))

    # Velocity control node
    actions.append(Node(
        package='uuv_control_cascaded_pid',
        executable='VelocityControl.py',
        name='velocity_control',
        output='screen',
        remappings=[
            ('odom', f'/{namespace}/pose_gt'),
            ('cmd_accel', f'/{namespace}/cmd_accel'),
        ]
    ))

    # Include uuv_teleop
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_teleop'),
                'launch',
                'uuv_teleop.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': namespace,
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': axis_yaw,
            'axis_x': axis_x,
            'axis_y': axis_y,
            'axis_z': axis_z,
            'deadman_button': deadman_button,
            'exclusion_buttons': exclusion_buttons,
        }.items(),
    ))

    # Include oberon4 joint control
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('oberon4_control'),
                'launch',
                'joint_control.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': namespace,
            'arm_name': 'oberon4',
            'deadman_button': oberon_deadman_button,
            'exclusion_button': oberon_exclusion_button,
            'home_button': oberon_home_button,
            'gripper_open_button': gripper_open_button,
            'gripper_close_button': gripper_close_button,
            'axis_azimuth': axis_oberon_azimuth,
            'axis_shoulder': axis_oberon_shoulder,
            'axis_wrist': axis_oberon_wrist,
        }.items(),
    ))

    # RViz
    actions.append(Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('uuv_gazebo'),
            'rviz',
            'rexrov_default.rviz'
        ])]
    ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0', description='X position'),
        DeclareLaunchArgument('y', default_value='0', description='Y position'),
        DeclareLaunchArgument('z', default_value='-70', description='Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('namespace', default_value='rexrov', description='Namespace'),

        DeclareLaunchArgument('axis_x', default_value='4', description='Axis for x'),
        DeclareLaunchArgument('axis_y', default_value='3', description='Axis for y'),
        DeclareLaunchArgument('axis_z', default_value='1', description='Axis for z'),
        DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis for yaw'),
        DeclareLaunchArgument('deadman_button', default_value='-1', description='Deadman button'),
        DeclareLaunchArgument('exclusion_buttons', default_value='4,5', description='Exclusion buttons'),

        DeclareLaunchArgument('axis_oberon_azimuth', default_value='0', description='Axis for oberon azimuth'),
        DeclareLaunchArgument('axis_oberon_shoulder', default_value='1', description='Axis for oberon shoulder'),
        DeclareLaunchArgument('axis_oberon_wrist', default_value='3', description='Axis for oberon wrist'),

        DeclareLaunchArgument('oberon_exclusion_button', default_value='4', description='Oberon exclusion button'),
        DeclareLaunchArgument('oberon_deadman_button', default_value='5', description='Oberon deadman button'),
        DeclareLaunchArgument('oberon_home_button', default_value='7', description='Oberon home button'),

        DeclareLaunchArgument('gripper_open_button', default_value='1', description='Gripper open button'),
        DeclareLaunchArgument('gripper_close_button', default_value='2', description='Gripper close button'),

        OpaqueFunction(function=launch_setup),
    ])
