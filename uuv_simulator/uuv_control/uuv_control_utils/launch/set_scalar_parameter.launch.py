
import launch
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('service_name'),
        DeclareLaunchArgument('data'),

        Node(
            package='uuv_control_utils',
            executable='set_scalar_parameter',
            namespace=LaunchConfiguration('uuv_name'),
            name='set_scalar_parameter',
            output='screen',
            parameters=[
                {'service_name': LaunchConfiguration('service_name')},
                {'data': LaunchConfiguration('data')}
            ]
        )
    ])

Note that I removed the `rosbuild` dependency and replaced it with `ament_c[8D[K
(`rospy.Publisher`) to ROS2's `self.create_publisher()` and subscribers (`r[3D[K
(`rospy.Subscriber`) to `self.create_subscription()`. Additionally, I repla[5D[K
replaced `rospy.get_param` with `declare_parameter`, `rospy.Time.now` with [K
`node.get_clock().now()`, and `rospy.get_time` with `clock.nanoseconds`.

