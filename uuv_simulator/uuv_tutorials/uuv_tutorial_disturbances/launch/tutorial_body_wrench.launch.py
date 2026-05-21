from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Setup for a timeout for the simulation run
    set_timeout_arg = DeclareLaunchArgument(
        'set_timeout',
        default_value='false'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='105'
    )

    # Vehicle's initial position
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
        default_value='-20'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0'
    )

    # Controller parameters
    K_arg = DeclareLaunchArgument(
        'K',
        default_value='5,5,5,5,5,5'
    )
    Kd_arg = DeclareLaunchArgument(
        'Kd',
        default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'
    )
    Ki_arg = DeclareLaunchArgument(
        'Ki',
        default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'
    )
    slope_arg = DeclareLaunchArgument(
        'slope',
        default_value='0.182,0.182,0.182,3.348,3.348,3.348'
    )

    # Include empty_underwater_world.launch
    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_gazebo_worlds'),
                'launch',
                'empty_underwater_world.launch'
            ])
        ]),
        launch_arguments={
            'set_timeout': LaunchConfiguration('set_timeout'),
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    # Include upload_rexrov.launch
    upload_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'upload_rexrov.launch'
            ])
        ]),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    # Include rov_nmb_sm_controller.launch
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'launch',
                'rov_nmb_sm_controller.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'K': LaunchConfiguration('K'),
            'Kd': LaunchConfiguration('Kd'),
            'Ki': LaunchConfiguration('Ki'),
            'slope': LaunchConfiguration('slope')
        }.items()
    )

    # Include start_circular_trajectory.launch
    circular_trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'start_circular_trajectory.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': 'rexrov',
            'radius': '5',
            'center_z': '-25',
            'max_forward_speed': '0.6'
        }.items()
    )

    # Include apply_body_wrench.launch (first wrench: force_x, force_y)
    body_wrench_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'apply_body_wrench.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '5',
            'duration': '10',
            'force_x': '3000',
            'force_y': '3000'
        }.items()
    )

    # Include apply_body_wrench.launch (second wrench: force_y, force_z)
    body_wrench_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'apply_body_wrench.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '20',
            'duration': '10',
            'force_y': '3000',
            'force_z': '3000'
        }.items()
    )

    # Include apply_body_wrench.launch (third wrench: torque_y, torque_z)
    body_wrench_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'apply_body_wrench.launch'
            ])
        ]),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '35',
            'duration': '10',
            'torque_y': '3000',
            'torque_z': '3000'
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
                'controller_demo.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        set_timeout_arg,
        timeout_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        K_arg,
        Kd_arg,
        Ki_arg,
        slope_arg,
        empty_world,
        upload_rexrov,
        controller,
        circular_trajectory,
        body_wrench_1,
        body_wrench_2,
        body_wrench_3,
        rviz
    ])
