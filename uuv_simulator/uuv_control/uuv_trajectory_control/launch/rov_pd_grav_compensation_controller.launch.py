
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.node import Node as RCLNode

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        Node(
            package='uuv_trajectory_control',
            executable='rov_pd_grav_compensation_controller',
            namespace=uuv_name,
            name='rov_pd_grav_compensation_controller',
            output='screen',
            parameters=[
                {'saturation': 1200.0},
                {'Kp': [11993.888]*6, 'Kd': [9077.459]*6, 'inertial_frame_i[17D[K
'inertial_frame_id': 'world'}
            ],
            remappings=[('odom', 'pose_gt')],
        )
    ])

Note that the changes are minimal as the original code is already a ROS2-st[7D[K
ROS2-style launch file, and only minor adjustments were needed to make it c[1D[K
compatible with ROS2.

