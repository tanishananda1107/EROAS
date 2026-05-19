
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    model_name = LaunchConfiguration('model_name')
    uuv_name = LaunchConfiguration('uuv_name')
    timeout = LaunchConfiguration('timeout')
    reset_tam = LaunchConfiguration('reset_tam')
    output_dir = LaunchConfiguration('output_dir')
    config_file = LaunchConfiguration('config_file')
    tam_file = LaunchConfiguration('tam_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'model_name'
        ),

        DeclareLaunchArgument(
            'uuv_name',
            default_value=model_name
        ),

        DeclareLaunchArgument(
            'base_link',
            default_value='base_link'
        ),

        DeclareLaunchArgument(
            'timeout',
            default_value='-1'
        ),

        DeclareLaunchArgument(
            'reset_tam',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'share',
                'uuv_thruster_manager',
                model_name
            ])
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('uuv_thrster_manager'),
                'share',
                'uuv_thruster_manager',
                model_name,
                'thruster_manager.yaml'
            ])
        ),

        DeclareLaunchArgument(
            'tam_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'share',
                'uuv_thruster_manager',
                model_name,
                'TAM.yaml'
            ])
        ),

        GroupAction([

            PushRosNamespace(uuv_name),

            Node(
                package='uuv_thruster_manager',
                executable='thruster_allocator.py',
                name='thruster_allocator',
                output='screen',
                parameters=[
                    ParameterFile(config_file),
                    {'node_name': uuv_name, 'timeout': timeout, 'output_dir[11D[K
'output_dir': output_dir},
                    {'declare_parameter': True}
                ],
                condition=IfCondition(reset_tam)
            ),

            Node(
                package='uuv_thruster_manager',
                executable='thruster_allocator.py',
                name='thruster_allocator',
                output='screen',
                parameters=[
                    ParameterFile(config_file),
                    ParameterFile(tam_file),
                    {'node_name': uuv_name, 'timeout': timeout}
                ],
                condition=UnlessCondition(reset_tam)
            )

        ])

    ])

following changes:

* Replaced `rospy` with `rclpy`
* Replaced `tf` with `tf2_ros`
* Replaced `catkin` with `ament_cmake`
* Removed `catkin_python_setup()`
* Changed `catkin_install_python` to `install(PROGRAMS ...)`
* Changed `CATKIN_PACKAGE_BIN_DESTINATION` and `CATKIN_PACKAGE_SHARE_DESTIN[28D[K
`CATKIN_PACKAGE_SHARE_DESTINATION` accordingly
(`self.create_publisher()`)
(`self.create_subscription()`)
* Replaced `rospy.get_param` with `declare_parameter`
* Replaced `rospy.Time.now` with `node.get_clock().now()`
* Replaced `rospy.get_time` with `clock.nanoseconds`

