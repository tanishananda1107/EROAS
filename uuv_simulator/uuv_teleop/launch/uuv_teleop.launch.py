from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        DeclareLaunchArgument('deadman_button', default_value='-1'),

        DeclareLaunchArgument(
            'exclusion_buttons',
            default_value='[4,5]'
        ),

        DeclareLaunchArgument('axis_roll', default_value='-1'),
        DeclareLaunchArgument('axis_pitch', default_value='-1'),
        DeclareLaunchArgument('axis_yaw', default_value='0'),

        DeclareLaunchArgument('axis_x', default_value='4'),
        DeclareLaunchArgument('axis_y', default_value='3'),
        DeclareLaunchArgument('axis_z', default_value='1'),

        DeclareLaunchArgument('gain_roll', default_value='0.0'),
        DeclareLaunchArgument('gain_pitch', default_value='0.0'),
        DeclareLaunchArgument('gain_yaw', default_value='0.2'),

        DeclareLaunchArgument('gain_x', default_value='2'),
        DeclareLaunchArgument('gain_y', default_value='0.3'),
        DeclareLaunchArgument('gain_z', default_value='0.3'),

        DeclareLaunchArgument(
            'output_topic',
            default_value='cmd_vel'
        ),

        DeclareLaunchArgument(
            'message_type',
            default_value='twist'
        ),

        Node(
            package='joy',
            executable='joy_node',
            namespace=uuv_name,
            name='joystick',
            output='screen',

            parameters=[{
                'autorepeat_rate': 10.0,
                'device_id':
                    LaunchConfiguration('joy_id')
            }]
        ),

        Node(
            package='uuv_teleop',
            executable='vehicle_teleop',
            namespace=uuv_name,
            name='joy_uuv_velocity_teleop',
            output='screen',

            remappings=[
                (
                    'output',
                    [
                        "/",
                        uuv_name,
                        "/",
                        LaunchConfiguration('output_topic')
                    ]
                ),

                (
                    'joy',
                    [
                        "/",
                        uuv_name,
                        "/joy"
                    ]
                )
            ],

            parameters=[{
                'type':
                    LaunchConfiguration('message_type'),

                'deadman_button':
                    LaunchConfiguration('deadman_button'),

                'exclusion_buttons':
                    LaunchConfiguration('exclusion_buttons'),

                'mapping.x.axis':
                    LaunchConfiguration('axis_x'),

                'mapping.x.gain':
                    LaunchConfiguration('gain_x'),

                'mapping.y.axis':
                    LaunchConfiguration('axis_y'),

                'mapping.y.gain':
                    LaunchConfiguration('gain_y'),

                'mapping.z.axis':
                    LaunchConfiguration('axis_z'),

                'mapping.z.gain':
                    LaunchConfiguration('gain_z'),

                'mapping.roll.axis':
                    LaunchConfiguration('axis_roll'),

                'mapping.roll.gain':
                    LaunchConfiguration('gain_roll'),

                'mapping.pitch.axis':
                    LaunchConfiguration('axis_pitch'),

                'mapping.pitch.gain':
                    LaunchConfiguration('gain_pitch'),

                'mapping.yaw.axis':
                    LaunchConfiguration('axis_yaw'),

                'mapping.yaw.gain':
                    LaunchConfiguration('gain_yaw')
            }]
        )
    ])
