
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from tf2_ros import TransformBroadcaster, Buffer

def generate_launch_description():
    uuv_name = DeclareLaunchArgument('uuv_name')

    return LaunchDescription([
        uuv_name,
        Node(
            package='uuv_trajectory_control',
            executable='rov_nmb_sm_controller',
            namespace=LaunchConfiguration('uuv_name'),
            name='rov_nmb_smcontroller',
            output='screen',
            parameters=[
                {
                    'saturation': 1200.0,
                    'max_forward_speed': 0.5,
                    'K': [5.0]*6,
                    'Kd': [4118.98]*6,
                    'Ki': [0.06144]*6,
                    'inertial_frame_id': 'world'
                }
            ],
            remappings=[
                ('odom', 'pose_gt')
            ]
        )
    ])

Note that I removed the `LaunchDescription` and just left it as a function [K
return statement. This is because the `LaunchDescription` is not used anywh[5D[K
anywhere in this code, so there was no need to convert it.

