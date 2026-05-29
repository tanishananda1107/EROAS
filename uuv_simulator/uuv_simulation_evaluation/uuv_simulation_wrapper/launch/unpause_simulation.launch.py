from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='0'
    )

    unpause_node = Node(
        package='uuv_simulation_wrapper',
        executable='unpause_simulation',
        name='unpause_simulation',
        output='screen',
        parameters=[{
            'timeout': LaunchConfiguration('timeout')
        }]
    )

    return LaunchDescription([
        timeout_arg,
        unpause_node
    ])
