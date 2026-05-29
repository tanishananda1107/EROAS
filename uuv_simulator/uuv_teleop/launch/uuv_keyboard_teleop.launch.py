from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),

        DeclareLaunchArgument(
            'output_topic',
            default_value='cmd_vel'
        ),

        DeclareLaunchArgument(
            'message_type',
            default_value='twist'
        ),

        Node(
            package='uuv_teleop',
            executable='vehicle_keyboard_teleop',
            namespace=uuv_name,
            name='keyboard_uuv_velocity_teleop',
            output='screen',

            remappings=[
                (
                    'output',
                    ["/", uuv_name, "/", output_topic]
                )
            ],

            parameters=[{
                'type':
                    LaunchConfiguration('message_type')
            }]
        )
    ])
