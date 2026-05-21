from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
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

    # Parameters for the current velocity model
    starting_time_arg = DeclareLaunchArgument(
        'starting_time',
        default_value='10'
    )
    end_time_arg = DeclareLaunchArgument(
        'end_time',
        default_value='40'
    )

    # Magnitude of the current velocity in m/s
    current_vel_arg = DeclareLaunchArgument(
        'current_vel',
        default_value='0.7'
    )

    # Angles given in degrees
    horizontal_angle_arg = DeclareLaunchArgument(
        'horizontal_angle',
        default_value='50'
    )
    vertical_angle_arg = DeclareLaunchArgument(
        'vertical_angle',
        default_value='20'
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

    # Include set_timed_current_perturbation.launch
    timed_current = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control_utils'),
                'launch',
                'set_timed_current_perturbation.launch'
            ])
        ]),
        launch_arguments={
            'starting_time': LaunchConfiguration('starting_time'),
            'end_time': LaunchConfiguration('end_time'),
            'current_vel': LaunchConfiguration('current_vel'),
            'horizontal_angle': LaunchConfiguration('horizontal_angle'),
            'vertical_angle': LaunchConfiguration('vertical_angle')
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
        starting_time_arg,
        end_time_arg,
        current_vel_arg,
        horizontal_angle_arg,
        vertical_angle_arg,
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
        timed_current,
        rviz
    ])
