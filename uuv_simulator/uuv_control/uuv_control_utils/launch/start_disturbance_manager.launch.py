
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('use_file', default_value='false'),
        DeclareLaunchArgument('disturbance_file', default_value=''),

        DeclareLaunchArgument('current_starting_time', default_value='20.0'[20D[K
default_value='20.0'),
        DeclareLaunchArgument('current_vel', default_value='1'),
        DeclareLaunchArgument('current_horz_angle', default_value='0.0'),
        DeclareLaunchArgument('current_vert_angle', default_value='0.0'),
        DeclareLaunchArgument('current_duration', default_value='10'),

        DeclareLaunchArgument('force_x', default_value='0'),
        DeclareLaunchArgument('force_y', default_value='2000'),
        DeclareLaunchArgument('force_z', default_value='0'),

        DeclareLaunchArgument('torque_x', default_value='0'),
        DeclareLaunchArgument('torque_y', default_value='0'),
        DeclareLaunchArgument('torque_z', default_value='0'),

        DeclareLaunchArgument('wrench_starting_time', default_value='30'),
        DeclareLaunchArgument('wrench_duration', default_value='20'),

        Node(
            package='uuv_control_utils',
            executable='disturbance_manager',
            namespace=LaunchConfiguration('uuv_name'),
            name='disturbance_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_file')),
            parameters=[LaunchConfiguration('disturbance_file')]
        ),

        Node(
            package='uuv_control_utils',
            executable='disturbance_manager',
            namespace=LaunchConfiguration('uuv_name'),
            name='disturbance_manager',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_file')),
            parameters=[{
                'disturbances': [
                    {
                        'type': 'current',
                        'starting_time': node.get_clock().now(),
                        'velocity': LaunchConfiguration('current_vel'),
                        'horizontal_angle': LaunchConfiguration('current_ho[31D[K
LaunchConfiguration('current_horz_angle'),
                        'vertical_angle': LaunchConfiguration('current_vert[33D[K
LaunchConfiguration('current_vert_angle'),
                        'duration': LaunchConfiguration('current_duration')[39D[K
LaunchConfiguration('current_duration')
                    },
                    {
                        'type': 'wrench',
                        'starting_time': node.get_clock().now(),
                        'duration': LaunchConfiguration('wrench_duration'),[39D[K
LaunchConfiguration('wrench_duration'),
                        'force': [
                            LaunchConfiguration('force_x'),
                            LaunchConfiguration('force_y'),
                            LaunchConfiguration('force_z')
                        ],
                        'torque': [
                            LaunchConfiguration('torque_x'),
                            LaunchConfiguration('torque_y'),
                            LaunchConfiguration('torque_z')
                        ]
                    }
                ]
            }]
        )
    ])

Note that I removed the `catkin_package()` and replaced it with the ROS2 pa[2D[K
package format. I also replaced `rospy` with `rclpy`, `tf` with `tf2_ros`, [K
and `catkin` with `ament_cmake`. Additionally, I updated the Python code to[2D[K
to use the new ROS2 APIs for publishers (`self.create_publisher()`), subscr[6D[K
subscribers (`self.create_subscription()`), and services (`create_service()[18D[K
(`create_service()`).

