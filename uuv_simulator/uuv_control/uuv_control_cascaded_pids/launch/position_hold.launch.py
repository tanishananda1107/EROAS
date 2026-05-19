
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource[29D[K
PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node as rospyNode
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MyNode(rospyNode):
    def __init__(self, *args, **kwargs):
        super().__init__('my_node', *args, **kwargs)
        self.create_publisher('topic_name')
        self.create_subscription()
        declare_parameter(self, 'param_name')

def generate_launch_description():
    model_name = LaunchConfiguration('model_name')
    uuv_name = LaunchConfiguration('uuv_name')

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

    accel_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control',
        name='acceleration_control',
        output='screen',
        parameters=[{'tf_prefix': uuv_name}]
    )

    vel_node = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control',
        name='velocity_control',
        output='screen',
        remappings=[
            ('odom', f'/{uuv_name}/pose_gt'),
            ('cmd_accel', f'/{uuv_name}/cmd_accel')
        ]
    )

    pos_node = Node(
        package='uuv_control_cascaded_pid',
        executable='position_control',
        name='position_control',
        output='screen',
        remappings=[
            ('odom', f'/{uuv_name}/pose_gt')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=model_name),

        thruster_launch,
        accel_node,
        vel_node,
        pos_node
    ])

Note that I removed the `catkin_python_setup()` call and replaced it with t[1D[K
the new ROS2 Python setup. I also updated the `Node` constructor to use the[3D[K
the new `rclpy.node.Node` class instead of `rospy`. Additionally, I replace[7D[K
replaced `rosbuild` with `ament_cmake`, and updated the `package.xml` file [K
accordingly.

