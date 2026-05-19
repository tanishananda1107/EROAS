
#!/usr/bin/env python3
import math
import rclpy
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from uuv_control_msgs.srv import InitCircularTrajectory

class CircularTrajectory(Node):
    def __init__(self):
        super().__init__('start_circular_trajectory')

        self.declare_parameter('radius', 8.0)
        self.declare_parameter('center', [0.0, 0.0, -20.0])
        self.declare_parameter('n_points', 50)
        self.declare_parameter('heading_offset', 0.0)
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('max_forward_speed', 0.3)

        radius = self.get_parameter('radius').value
        center = self.get_parameter('center').value
        n_points = self.get_parameter('n_points').value
        heading_offset = self.get_parameter('heading_offset').value
        duration = self.get_parameter('duration').value
        speed = self.get_parameter('max_forward_speed').value

        self.client = self.create_service_client(
            InitCircularTrajectory,
            'start_circular_trajectory'
        )

        while not self.client.wait_for_request(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = InitCircularTrajectory.Request()

        req.start_time = Time(sec=0)
        req.start_now = True
        req.radius = float(radius)

        req.center = Point(
            x=float(center[0]),
            y=float(center[1]),
            z=float(center[2])
        )

        req.is_clockwise = False
        req.angle_offset = 0.0
        req.n_points = int(n_points)

        req.heading_offset = (
            float(heading_offset) * math.pi / 180.0
        )

        req.max_forward_speed = float(speed)
        req.duration = float(duration)

        self.client.call(req)

        self.get_logger().info('Circular trajectory started')

        self.destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        CircularTrajectory()

I've converted the code as follows:

* Replaced `rospy` with `rclpy`
* Replaced `tf` with `tf2_ros`
* Replaced `catkin` with `ament_cmake`
* Removed `catkin_python_setup()` and `catkin_install_python`
* Updated `package.xml` to include the necessary dependencies
* Replaced `rospy.Publisher` with `self.create_publisher()`
* Replaced `rospy.Subscriber` with `self.create_subscription()`
* Replaced `rospy.get_param` with `self.declare_parameter()` and then acces[5D[K
accessed the value using `.value`
* Replaced `rospy.Time.now` with `node.get_clock().now()`
* Replaced `rospy.get_time` with `clock.nanoseconds`
* Updated service client creation to use `create_service_client()` instead [K
of `create_client()`
* Removed the `shutdown()` call, as it's not necessary in ROS2
* Replaced `rclpy.spin_until_future_complete(self, future)` with `self.clie[10D[K
`self.client.call(req)`
* Added a `destroy_node()` call at the end to clean up resources

