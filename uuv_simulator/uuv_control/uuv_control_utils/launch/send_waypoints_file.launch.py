
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument(
            'filename',
            default_value='/tmp/example_waypoints.yaml'
        ),
        DeclareLaunchArgument('start_time', default_value='-1'),
        DeclareLaunchArgument('interpolator', default_value='lipb'),

        Node(
            package='uuv_control_utils',
            executable='send_waypoint_file',
            namespace=LaunchConfiguration('uuv_name'),
            name='send_waypoint_file',
            output='screen',
            parameters=[
                declare_parameter({'filename': LaunchConfiguration('filenam[28D[K
LaunchConfiguration('filename')}),
                declare_parameter({'start_time': LaunchConfiguration('start[26D[K
LaunchConfiguration('start_time')}),
                declare_parameter({'interpolator': LaunchConfiguration('int[24D[K
LaunchConfiguration('interpolator')})
            ]
        )
    ])

this code.

