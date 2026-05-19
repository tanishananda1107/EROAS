
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.qos import QoSProfile
import rclpy

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),

        Node(
            package='uuv_trajectory_control',
            executable='rov_nl_pid_controller',
            namespace=uuv_name,
            name='rov_nl_pid_controller',
            output='screen',
            parameters=[{
                'saturation': 6000.0,
                'Kp': [6017.059]*6,
                'Kd': [2682.950]*6,
                'Ki': [0.0]*6,
                'Hm': [1657.655]*6,
                'inertial_frame_id': 'world'
            }],
            remappings=[
                ('odom', 'pose_gt')
            ]
        )
    ])

Note that I did not touch the Python code that is used to generate the laun[4D[K
launch description, as it seems correct for ROS2.

