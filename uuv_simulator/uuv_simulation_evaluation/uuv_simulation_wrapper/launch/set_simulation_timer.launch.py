from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='0'
    )

    simulation_timer_node = Node(
        package='uuv_simulation_wrapper',
        executable='set_simulation_timer',
        name='simulation_timeout',
        output='screen',
        parameters=[{
            'timeout': LaunchConfiguration('timeout')
        }]
    )

    return LaunchDescription([
        timeout_arg,
        simulation_timer_node
    ])
