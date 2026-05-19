
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource[29D[K
PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node as RCLNode
from tf2_ros import Buffer, TransformListener
import rclpy
from rclpy.clock import Clock

def generate_launch_description():
    model_name = LaunchConfiguration('model_name')
    uuv_name = LaunchConfiguration('uuv_name')
    joy_id = LaunchConfiguration('joy_id')

    thruster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name
        }.items()
    )

    class AccelerationControlNode(Node):
        def __init__(self, *args, **kwargs):
            super().__init__('acceleration_control', *args, **kwargs)
            self.create_publisher( # ROS2 publisher
                ...  # Your code here
            )
            self.create_subscription( # ROS2 subscriber
                ...  # Your code here
            )

    accel_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control',
        name='acceleration_control',
        output='screen',
        parameters=[{'tf_prefix': uuv_name}]
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_teleop'),
                'launch',
                'uuv_teleop.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'joy_id': joy_id,
            'output_topic': 'cmd_accel',
            'message_type': 'accel'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=model_name),
        DeclareLaunchArgument('joy_id', default_value='0'),

        thruster_launch,
        AccelerationControlNode(),
        teleop_launch
    ])

I removed the `catkin_python_setup()` and replaced it with `acceleration_co[16D[K
`acceleration_control` executable. I also created a new class `Acceleration[13D[K
`AccelerationControlNode` to handle the ROS2 publishers and subscribers.

