from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rexrov2_control'),
                    'launch',
                    'start_thruster_manager.launch.py'
                ])
            ]),
            launch_arguments={
                'model_name': 'rexrov2',
                'uuv_name': uuv_name,
            }.items(),
        ),

        GroupAction([
            Node(
                package='uuv_control_cascaded_pid',
                executable='AccelerationControl.py',
                namespace=uuv_name,
                parameters=[{
                    'pid.mass': 1862.0,
                    'pid.ixx': 525.39,
                    'pid.ixy': 0.0,
                    'pid.ixz': 0.0,
                    'pid.iyy': 794.20,
                    'pid.iyz': 0.0,
                    'pid.izz': 691.23,
                }],
                output='screen'
            ),
            Node(
                package='uuv_control_cascaded_pid',
                executable='VelocityControl.py',
                namespace=uuv_name,
                parameters=[{
                    'odom_vel_in_world': False,
                    'linear_p': 10.0,
                    'linear_i': 2.0,
                    'linear_d': 0.0,
                    'linear_sat': 20.0,
                    'angular_p': 10.0,
                    'angular_i': 2.0,
                    'angular_d': 1.0,
                    'angular_sat': 5.0,
                }],
                output='screen'
            )
        ])
    ])
