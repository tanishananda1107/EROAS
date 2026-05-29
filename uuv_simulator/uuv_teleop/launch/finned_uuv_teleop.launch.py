from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')
    joy_id = LaunchConfiguration('joy_id')
    use_param_file = LaunchConfiguration('use_param_file')
    filename = LaunchConfiguration('filename')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('use_param_file', default_value='false'),
        DeclareLaunchArgument('filename', default_value='.'),

        DeclareLaunchArgument('axis_thruster', default_value='1'),
        DeclareLaunchArgument('axis_roll', default_value='0'),
        DeclareLaunchArgument('axis_pitch', default_value='4'),
        DeclareLaunchArgument('axis_yaw', default_value='3'),

        DeclareLaunchArgument('thruster_rotor_gain', default_value='0.0009'),
        DeclareLaunchArgument('max_thrust', default_value='200'),
        DeclareLaunchArgument('thruster_topic', default_value='thrusters/0/input'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins/'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='/input'),
        DeclareLaunchArgument('thruster_joy_gain', default_value='1.0'),

        DeclareLaunchArgument('n_fins', default_value='4'),
        DeclareLaunchArgument('gain_roll', default_value='[1,1,1,1]'),
        DeclareLaunchArgument('gain_pitch', default_value='[1,1,-1,-1]'),
        DeclareLaunchArgument('gain_yaw', default_value='[-1,1,1,-1]'),

        Node(
            package='uuv_teleop',
            executable='finned_uuv_teleop',
            namespace=uuv_name,
            name='finned_uuv_teleop',
            output='screen',
            condition=IfCondition(use_param_file),
            parameters=[filename]
        ),

        Node(
            package='joy',
            executable='joy_node',
            namespace=uuv_name,
            name='joystick',
            output='screen',
            parameters=[{
                'autorepeat_rate': 10.0,
                'device_id': joy_id
            }]
        ),

        Node(
            package='uuv_teleop',
            executable='finned_uuv_teleop',
            namespace=uuv_name,
            name='finned_uuv_teleop_manual',
            output='screen',
            condition=UnlessCondition(use_param_file),
            parameters=[{
                'axis_thruster': LaunchConfiguration('axis_thruster'),
                'axis_roll': LaunchConfiguration('axis_roll'),
                'axis_pitch': LaunchConfiguration('axis_pitch'),
                'axis_yaw': LaunchConfiguration('axis_yaw'),

                'n_fins': LaunchConfiguration('n_fins'),

                'thruster_joy_gain':
                    LaunchConfiguration('thruster_joy_gain'),

                'thruster_model.name': 'proportional',

                'thruster_model.max_thrust':
                    LaunchConfiguration('max_thrust'),

                'thruster_model.params.gain':
                    LaunchConfiguration('thruster_rotor_gain'),

                'gain_roll': LaunchConfiguration('gain_roll'),
                'gain_pitch': LaunchConfiguration('gain_pitch'),
                'gain_yaw': LaunchConfiguration('gain_yaw'),

                'thruster_topic':
                    LaunchConfiguration('thruster_topic'),

                'fin_topic_prefix':
                    LaunchConfiguration('fin_topic_prefix'),

                'fin_topic_suffix':
                    LaunchConfiguration('fin_topic_suffix')
            }]
        )
    ])
