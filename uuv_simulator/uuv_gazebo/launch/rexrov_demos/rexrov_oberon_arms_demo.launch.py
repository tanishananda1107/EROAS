#!/usr/bin/env python3
"""
Launch file for rexrov oberon arms demo.
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
    vehicle_deadman_button = LaunchConfiguration('vehicle_deadman_button').perform(context)
    vehicle_exclusion_buttons = LaunchConfiguration('vehicle_exclusion_buttons').perform(context)
    axis_oberon_jc_azimuth = LaunchConfiguration('axis_oberon_jc_azimuth').perform(context)
    axis_oberon_jc_shoulder = LaunchConfiguration('axis_oberon_jc_shoulder').perform(context)
    axis_oberon_jc_elbow = LaunchConfiguration('axis_oberon_jc_elbow').perform(context)
    axis_oberon_jc_roll = LaunchConfiguration('axis_oberon_jc_roll').perform(context)
    axis_oberon_jc_pitch = LaunchConfiguration('axis_oberon_jc_pitch').perform(context)
    axis_oberon_jc_yaw = LaunchConfiguration('axis_oberon_jc_yaw').perform(context)
    axis_oberon4_azimuth = LaunchConfiguration('axis_oberon4_azimuth').perform(context)
    axis_oberon4_shoulder = LaunchConfiguration('axis_oberon4_shoulder').perform(context)
    axis_oberon4_wrist = LaunchConfiguration('axis_oberon4_wrist').perform(context)
    oberon_home_button = LaunchConfiguration('oberon_home_button').perform(context)
    oberon_exclusion_button = LaunchConfiguration('oberon_exclusion_button').perform(context)
    oberon_deadman_button = LaunchConfiguration('oberon_deadman_button').perform(context)
    oberon4_exclusion_button = LaunchConfiguration('oberon4_exclusion_button').perform(context)
    oberon4_deadman_button = LaunchConfiguration('oberon4_deadman_button').perform(context)
    gripper_open_button = LaunchConfiguration('gripper_open_button').perform(context)
    gripper_close_button = LaunchConfiguration('gripper_close_button').perform(context)

    actions = []

    # Include upload_rexrov_oberon_arms
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'upload_rexrov_oberon_arms.launch.py'
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
            'deadman_button': vehicle_deadman_button,
            'exclusion_buttons': vehicle_exclusion_buttons,
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
            'deadman_button': oberon4_deadman_button,
            'exclusion_button': oberon4_exclusion_button,
            'home_button': oberon_home_button,
            'gripper_open_button': gripper_open_button,
            'gripper_close_button': gripper_close_button,
            'axis_azimuth': axis_oberon4_azimuth,
            'axis_shoulder': axis_oberon4_shoulder,
            'axis_wrist': axis_oberon4_wrist,
        }.items(),
    ))

    # Include oberon7 joint control
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('oberon7_control'),
                'launch',
                'joint_control.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': namespace,
            'arm_name': 'oberon7',
            'deadman_button': oberon_deadman_button,
            'exclusion_button': oberon_exclusion_button,
            'axis_azimuth': axis_oberon_jc_azimuth,
            'axis_shoulder': axis_oberon_jc_shoulder,
            'axis_elbow': axis_oberon_jc_elbow,
            'axis_roll': axis_oberon_jc_roll,
            'axis_pitch': axis_oberon_jc_pitch,
            'axis_yaw': axis_oberon_jc_yaw,
            'home_button': oberon_home_button,
            'gripper_open_button': gripper_open_button,
            'gripper_close_button': gripper_close_button,
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
        DeclareLaunchArgument('z', default_value='-20', description='Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('namespace', default_value='rexrov', description='Namespace'),

        DeclareLaunchArgument('axis_x', default_value='4', description='Axis for x'),
        DeclareLaunchArgument('axis_y', default_value='3', description='Axis for y'),
        DeclareLaunchArgument('axis_z', default_value='1', description='Axis for z'),
        DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis for yaw'),
        DeclareLaunchArgument('vehicle_deadman_button', default_value='-1', description='Vehicle deadman button'),
        DeclareLaunchArgument('vehicle_exclusion_buttons', default_value='4,5', description='Vehicle exclusion buttons'),

        DeclareLaunchArgument('axis_oberon_jc_azimuth', default_value='3', description='Axis for oberon azimuth'),
        DeclareLaunchArgument('axis_oberon_jc_shoulder', default_value='4', description='Axis for oberon shoulder'),
        DeclareLaunchArgument('axis_oberon_jc_elbow', default_value='1', description='Axis for oberon elbow'),
        DeclareLaunchArgument('axis_oberon_jc_roll', default_value='6', description='Axis for oberon roll'),
        DeclareLaunchArgument('axis_oberon_jc_pitch', default_value='7', description='Axis for oberon pitch'),
        DeclareLaunchArgument('axis_oberon_jc_yaw', default_value='0', description='Axis for oberon yaw'),

        DeclareLaunchArgument('axis_oberon4_azimuth', default_value='0', description='Axis for oberon4 azimuth'),
        DeclareLaunchArgument('axis_oberon4_shoulder', default_value='1', description='Axis for oberon4 shoulder'),
        DeclareLaunchArgument('axis_oberon4_wrist', default_value='3', description='Axis for oberon4 wrist'),

        DeclareLaunchArgument('oberon_home_button', default_value='7', description='Oberon home button'),
        DeclareLaunchArgument('oberon_exclusion_button', default_value='4', description='Oberon exclusion button'),
        DeclareLaunchArgument('oberon_deadman_button', default_value='5', description='Oberon deadman button'),
        DeclareLaunchArgument('oberon4_exclusion_button', default_value='5', description='Oberon4 exclusion button'),
        DeclareLaunchArgument('oberon4_deadman_button', default_value='4', description='Oberon4 deadman button'),

        DeclareLaunchArgument('gripper_open_button', default_value='1', description='Gripper open button'),
        DeclareLaunchArgument('gripper_close_button', default_value='2', description='Gripper close button'),

        OpaqueFunction(function=launch_setup),
    ])
