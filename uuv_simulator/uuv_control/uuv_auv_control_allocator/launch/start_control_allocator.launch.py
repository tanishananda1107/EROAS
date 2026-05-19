
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy


def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')
    base_link = LaunchConfiguration('base_link')
    output_dir = LaunchConfiguration('output_dir')
    input_topic = LaunchConfiguration('input_topic')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('input_topic', default_value='control_allocat[30D[K
default_value='control_allocation/control_input'),

        DeclareLaunchArgument('thruster_topic_prefix', default_value='thrus[20D[K
default_value='thrusters'),
        DeclareLaunchArgument('thruster_topic_suffix', default_value='input[20D[K
default_value='input'),
        DeclareLaunchArgument('thruster_frame_base', default_value='thruste[22D[K
default_value='thruster_'),
        DeclareLaunchArgument('max_thrust', default_value='120'),
        DeclareLaunchArgument('thruster_conversion_fcn', default_value='pro[18D[K
default_value='proportional'),
        DeclareLaunchArgument('thruster_gain', default_value='0.0'),
        DeclareLaunchArgument('thruster_input', default_value='0,1,2,3'),
        DeclareLaunchArgument('thruster_output', default_value='0,1,2,3'),

        DeclareLaunchArgument('fin_frame_base', default_value='fin'),
        DeclareLaunchArgument('fluid_density', default_value='1028.0'),
        DeclareLaunchArgument('lift_coefficient', default_value='0.0'),
        DeclareLaunchArgument('fin_area', default_value='0.0'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='input'),
        DeclareLaunchArgument('fin_lower_joint_limit', default_value='-1.57[20D[K
default_value='-1.57'),
        DeclareLaunchArgument('fin_upper_joint_limit', default_value='1.57'[20D[K
default_value='1.57'),

        DeclareLaunchArgument('timeout', default_value='-1'),
        DeclareLaunchArgument('update_rate', default_value='10'),

        Node(
            package='uuv_auv_control_allocator',
            executable='control_allocator',
            namespace=uuv_name,
            output='screen',
            remappings=[
                ('control_input', input_topic)
            ],
            parameters=[{
                'output_dir': output_dir,
                'base_link': base_link,
                'thruster_config': {
                    'topic_prefix': LaunchConfiguration('thruster_topic_pre[39D[K
LaunchConfiguration('thruster_topic_prefix'),
                    'topic_suffix': LaunchConfiguration('thruster_topic_suf[39D[K
LaunchConfiguration('thruster_topic_suffix'),
                    'frame_base': LaunchConfiguration('thruster_frame_base'[41D[K
LaunchConfiguration('thruster_frame_base'),
                    'max_thrust': LaunchConfiguration('max_thrust'),
                    'conversion_fcn': LaunchConfiguration('thruster_convers[37D[K
LaunchConfiguration('thruster_conversion_fcn'),
                    'conversion_fcn_params': {
                        'gain': LaunchConfiguration('thruster_gain'),
                        'input': LaunchConfiguration('thruster_input'),
                        'output': LaunchConfiguration('thruster_output'),
                    },
                },
                'fin_config': {
                    'frame_base': LaunchConfiguration('fin_frame_base'),
                    'fluid_density': LaunchConfiguration('fluid_density'),
                    'lift_coefficient': LaunchConfiguration('lift_coefficie[35D[K
LaunchConfiguration('lift_coefficient'),
                    'fin_area': LaunchConfiguration('fin_area'),
                    'topic_prefix': LaunchConfiguration('fin_topic_prefix')[39D[K
LaunchConfiguration('fin_topic_prefix'),
                    'topic_suffix': LaunchConfiguration('fin_topic_suffix')[39D[K
LaunchConfiguration('fin_topic_suffix'),
                    'lower_limit': LaunchConfiguration('fin_lower_joint_lim[40D[K
LaunchConfiguration('fin_lower_joint_limit'),
                    'upper_limit': LaunchConfiguration('fin_upper_joint_lim[40D[K
LaunchConfiguration('fin_upper_joint_limit'),
                },
                'timeout': LaunchConfiguration('timeout'),
                'update_rate': LaunchConfiguration('update_rate'),
            }]
        )
    ])

a summary of the changes:

* Replaced `rospy` imports with `rclpy`.
* Replaced `tf` with `tf2_ros`.
* Removed `catkin_python_setup()` and replaced it with an `install(PROGRAMS[17D[K
`install(PROGRAMS ...)` command.
* Replaced `CATKIN_PACKAGE_BIN_DESTINATION` and `CATKIN_PACKAGE_SHARE_DESTI[27D[K
`CATKIN_PACKAGE_SHARE_DESTINATION` with ROS2-specific equivalents.
* Updated the `package.xml` file to reflect the changes:
	+ Replaced `rosbuild` with `ament_cmake`.
	+ Replaced `rospy` dependencies with `rclpy`.
* In Python code, replaced:
	+ `rospy.Publisher` with `self.create_publisher()`
	+ `rospy.Subscriber` with `self.create_subscription()`
	+ `rospy.get_param` with `declare_parameter`
	+ `rospy.Time.now` with `node.get_clock().now()`
	+ `rospy.get_time` with `clock.nanoseconds`
* Replaced `create_service()` with the equivalent ROS2 service migration.

