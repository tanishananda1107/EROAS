
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.node import Node as RCLPYNode
from tf2_ros import TransformBroadcaster, Buffer
from rclpy.time import Clock

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('component'),
        DeclareLaunchArgument('mean', default_value='0.0'),
        DeclareLaunchArgument('min', default_value='0.0'),
        DeclareLaunchArgument('max', default_value='0.0'),
        DeclareLaunchArgument('noise', default_value='0.0'),
        DeclareLaunchArgument('mu', default_value='0.0'),

        Node(
            package='uuv_control_utils',
            executable='set_gm_current_perturbation',
            name='set_gm_current_perturbation',
            output='screen',
            parameters=[
                {'component': LaunchConfiguration('component')},
                {'mean': LaunchConfiguration('mean')},
                {'min': LaunchConfiguration('min')},
                {'max': LaunchConfiguration('max')},
                {'noise': LaunchConfiguration('noise')},
                {'mu': LaunchConfiguration('mu')}
            ]
        )
    ])

