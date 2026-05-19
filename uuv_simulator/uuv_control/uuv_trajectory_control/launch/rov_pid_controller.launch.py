from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import rclpy
from tf2_ros import TransformListener
from ament_cmake import cmake

def generate_launch_description():
    uuv_name = DeclareLaunchArgument('uuv_name')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),

        ExecuteProcess(
            cmd=['ros2 run', 'navigator_auv', 'sonar_reconstruction'],
            namespace=uuv_name,
            name='sonar_reconstruction',
            output='screen'
        ),

        Node(
            package='uuv_trajectory_control',
            executable='rov_pid_controller',
            namespace=uuv_name,
            name='rov_pid_controller',
            output='screen',
            declare_declare=['saturation', 1200.0],
            parameters=[{
                'Kp': [11993.888]*6,
                'Kd': [9077.459]*6,
                'Ki': [321.417]*6,
                'inertial_frame_id': 'world'
            }],
            remappings=[
                ('odom', 'pose_gt'),
                ('trajectory', 'dp_controller/trajectory'),
                ('thruster_output', 'thruster_manager/input_stamped')
            ]
        )
    ])
Note that I removed the `LaunchDescription` constructor and replaced it wit[3D[K
with a list of launch actions.

