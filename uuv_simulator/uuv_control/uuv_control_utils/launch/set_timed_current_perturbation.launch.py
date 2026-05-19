
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    starting_time = DeclareLaunchArgument(
        'starting_time',
        default_value='0.0'
    )

    end_time = DeclareLaunchArgument(
        'end_time',
        default_value='-1'
    )

    current_vel = DeclareLaunchArgument(
        'current_vel',
        default_value='1'
    )

    horizontal_angle = DeclareLaunchArgument(
        'horizontal_angle',
        default_value='0.0'
    )

    vertical_angle = DeclareLaunchArgument(
        'vertical_angle',
        default_value='0.0'
    )

    return LaunchDescription([

        starting_time,
        end_time,
        current_vel,
        horizontal_angle,
        vertical_angle,

        Node(
            package='uuv_control_utils',
            executable='set_timed_current_perturbation.py',
            name='set_timed_current_perturbation',
            output='screen',
            parameters=[
                {'starting_time': LaunchConfiguration('starting_time')},
                {'end_time': LaunchConfiguration('end_time')},
                {'current_velocity': LaunchConfiguration('current_vel')},
                {'horizontal_angle': LaunchConfiguration('horizontal_angle'[38D[K
LaunchConfiguration('horizontal_angle')},
                {'vertical_angle': LaunchConfiguration('vertical_angle')}
            ]
        )
    ])

Note that I removed the `DeclareLaunchArgument` and replaced it with the ne[2D[K
new equivalent. Also, I kept the same code for the node launch action, as i[1D[K
converted.

