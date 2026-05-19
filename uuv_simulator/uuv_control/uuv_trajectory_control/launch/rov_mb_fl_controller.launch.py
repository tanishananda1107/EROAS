
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.node import Node as RCLNode
import rclpy
from tf2_ros import TransformBroadcaster

def generate_launch_description():
    
    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([
        
        DeclareLaunchArgument('uuv_name'),

        Node(
            package='uuv_trajectory_control',
            executable='rov_mb_fl_controller',
            namespace=uuv_name,
            name='rov_mb_fl_controller',
            output='screen',
            parameters=[{
                'saturation': 1200.0,
                'Kp': [19987.218]*6,
                'Kd': [11458.051]*6,
                'Ki': [1689.976]*6,
                'inertial_frame_id': 'world'
            }],
            remappings=[
                ('odom', 'pose_gt'),
                ('trajectory', 'dp_controller/trajectory'),
                ('waypoints', 'dp_controller/waypoints'),
                ('thruster_output', 'thruster_manager/input_stamped')
            ]
        ),
    ])

Please note that you need to replace `rov_mb_fl_controller` with the actual[6D[K
actual name of your ROS2 node and make sure it's in the correct location (u[2D[K
(usually `src` directory) within your package.

