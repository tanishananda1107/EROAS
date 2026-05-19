
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from uuv_control_msgs.srv import InitWaypointsFromFile
from tf2_ros import Buffer, TransformException


class SendWaypointFile(Node):

    def __init__(self):
        super().__init__('send_waypoint_file')

        self.declare_parameter('filename', '')
        self.declare_parameter('start_time', -1.0)
        self.declare_parameter('interpolator', 'lipb')

        filename = self.get_parameter('filename').value
        start_time = self.get_parameter('start_time').value
        interpolator = self.get_parameter('interpolator').value

        start_now = start_time < 0.0

        self.client = self.create_service_client(
            InitWaypointsFromFile,
            'init_waypoints_from_file'
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = InitWaypointsFromFile.Request()

        req.start_time = Time(
            sec=int(start_time if start_time > 0 else 0)
        )

        req.start_now = start_now
        req.filename.data = filename.encode()
        req.interpolator.data = interpolator.encode()

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Waypoint file sent')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    SendWaypointFile()

`catkin` with `ament_cmake`. I also removed the `catkin_python_setup()` cal[3D[K
call, as it is no longer needed in ROS2. Additionally, I updated the publis[6D[K
publisher and subscriber code to use the new methods provided by `rclpy`.

