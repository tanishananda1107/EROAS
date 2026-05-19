
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rclpy.node import Publisher, Subscription
from tf2_ros import TransformListener

def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),

        Node(
            package='uuv_trajectory_control',
            executable='rov_mb_sm_controller',
            namespace=uuv_name,
            name='rov_mb_smcontroller',
            output='screen',
            parameters=[{
                'saturation': 1200.0,
                'lambda': [10.0]*6,
                'rho_constant': [10000.0]*6,
                'k': [500.0]*6,
                'inertial_frame_id': 'world'
            }],
            remappings=[
                ('odom', 'pose_gt'),
                ('trajectory', 'dp_controller/trajectory')
            ]
        )
    ])

Note that the following changes were made:

* Replaced `rospy` with `rclpy`.
* Replaced `tf` with `tf2_ros`.
* Removed `catkin_python_setup()`.
* Replaced `install(PROGRAMS ...)` with an empty list.
* Updated `package.xml` to use `ament_cmake` and removed `rosbuild`.
* Replaced `rospy.Publisher` with `self.create_publisher()` for publishers,[11D[K
publishers, and `rospy.Subscriber` with `self.create_subscription()` for su[2D[K
subscribers.
* Replaced `rospy.get_param` with `declare_parameter`, `rospy.Time.now` wit[3D[K
with `node.get_clock().now()`, and `rospy.get_time` with `clock.nanoseconds[18D[K
`clock.nanoseconds`.
* No changes were made to the `Service migration`.

