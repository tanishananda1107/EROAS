
#!/usr/bin/env python3

from math import pi
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from uuv_control_msgs.srv import InitHelicalTrajectory


class HelicalTrajectory(Node):

    def __init__(self):
        super().__init__('start_helical_trajectory')

        self.declare_parameter('radius', 8.0)
        self.declare_parameter('center', [0.0, 0.0, -30.0])
        self.declare_parameter('n_points', 50)
        self.declare_parameter('heading_offset', 0.0)
        self.declare_parameter('duration', 150.0)
        self.declare_parameter('n_turns', 1)
        self.declare_parameter('delta_z', 5.0)
        self.declare_parameter('max_forward_speed', 0.3)

        center = self.get_parameter('center').value

        self.client = self.create_service_client(
            InitHelicalTrajectory,
            'start_helical_trajectory'
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = InitHelicalTrajectory.Request()

        req.start_time = Time(sec=0)
        req.start_now = True

        req.radius = float(
            self.get_parameter('radius').value
        )

        req.center = Point(
            x=float(center[0]),
            y=float(center[1]),
            z=float(center[2])
        )

        req.is_clockwise = False
        req.angle_offset = 0.0

        req.n_points = int(
            self.get_parameter('n_points').value
        )

        req.heading_offset = (
            float(
                self.get_parameter(
                    'heading_offset').value
            ) * pi / 180.0
        )

        req.max_forward_speed = float(
            self.get_parameter(
                'max_forward_speed').value
        )

        req.duration = float(
            self.get_parameter('duration').value
        )

        req.n_turns = int(
            self.get_parameter('n_turns').value
        )

        req.delta_z = float(
            self.get_parameter('delta_z').value
        )

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Helical trajectory started')

        clock = self.get_clock()
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        node_clock = time.time()

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    HelicalTrajectory()


if __name__ == '__main__':
    main()

made are:

* Replaced `rospy` with `rclpy`.
* Replaced `tf` with `tf2_ros`.
* Replaced `catkin` with `ament_cmake`.
* Removed `catkin_python_setup()`.
* Changed `CATKIN_PACKAGE_BIN_DESTINATION` and `CATKIN_PACKAGE_SHARE_DESTIN[28D[K
`CATKIN_PACKAGE_SHARE_DESTINATION` to `lib/${PROJECT_NAME}` and `share/${PR[11D[K
`share/${PROJECT_NAME}`, respectively.
`self.create_publisher()`.
`self.create_subscription()`.
* Replaced `rospy.get_param` with `declare_parameter`.
* Replaced `rospy.Time.now` with `node.get_clock().now()`.
* Replaced `rospy.get_time` with `clock.nanoseconds`.

