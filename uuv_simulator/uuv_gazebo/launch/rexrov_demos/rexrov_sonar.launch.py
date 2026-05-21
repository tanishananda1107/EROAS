from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Setup for joystick configuration
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rexrov'
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='-85'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0'
    )
    joy_id_arg = DeclareLaunchArgument(
        'joy_id',
        default_value='0'
    )
    axis_yaw_arg = DeclareLaunchArgument(
        'axis_yaw',
        default_value='0'
    )
    axis_x_arg = DeclareLaunchArgument(
        'axis_x',
        default_value='4'
    )
    axis_y_arg = DeclareLaunchArgument(
        'axis_y',
        default_value='3'
    )
    axis_z_arg = DeclareLaunchArgument(
        'axis_z',
        default_value='1'
    )

    # Include upload_rexrov_default.launch
    upload_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'upload_rexrov_default.launch'
            ])
        ]),
        launch_arguments={
            'mode': 'sonar',
            'namespace': LaunchConfiguration('namespace'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    # Include thruster_manager.launch
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': LaunchConfiguration('namespace'),
            'model_name': 'rexrov'
        }.items()
    )

    # Include uuv_teleop.launch
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_teleop'),
                'launch',
                'uuv_teleop.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': LaunchConfiguration('namespace'),
            'joy_id': LaunchConfiguration('joy_id'),
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': LaunchConfiguration('axis_yaw'),
            'axis_x': LaunchConfiguration('axis_x'),
            'axis_y': LaunchConfiguration('axis_y'),
            'axis_z': LaunchConfiguration('axis_z')
        }.items()
    )

    # Launch RViz
    rviz = Node(
        package='rviz',
        executable='rviz',
        arguments=[
            '-d',
            os.path.join(
                FindPackageShare('uuv_gazebo').perform(None),
                'rviz',
                'rexrov_sonar.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        joy_id_arg,
        axis_yaw_arg,
        axis_x_arg,
        axis_y_arg,
        axis_z_arg,
        upload_rexrov,
        thruster_manager,
        teleop,
        rviz
    ])
