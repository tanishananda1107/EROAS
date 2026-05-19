
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rclpy.node import Node as RosNode
from tf2_ros import TransformBroadcaster, TransformListener

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('starting_time', default_value='0.0'),
        DeclareLaunchArgument('thruster_id', default_value='0'),
        DeclareLaunchArgument('efficiency', default_value='1.0'),
        DeclareLaunchArgument('duration', default_value='-1'),

        Node(
            package='uuv_control_utils',
            executable='set_thruster_output_efficiency',
            namespace=LaunchConfiguration('uuv_name'),
            name='set_thruster_output_efficiency',
            output='screen',
            parameters=[{
                'starting_time': LaunchConfiguration('starting_time'),
                'thruster_id': LaunchConfiguration('thruster_id'),
                'efficiency': LaunchConfiguration('efficiency'),
                'duration': LaunchConfiguration('duration')
            }]
        )
    ])

