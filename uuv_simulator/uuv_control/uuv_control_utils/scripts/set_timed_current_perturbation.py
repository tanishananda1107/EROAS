
#!/usr/bin/env python3

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

class TimedCurrent(Node):

    def __init__(self):
        super().__init__('set_timed_current_perturbation')

        self.declare_parameter('current_velocity', 0.0)
        self.declare_parameter('horizontal_angle', 0.0)
        self.declare_parameter('vertical_angle', 0.0)

        vel = self.get_parameter('current_velocity').value

        h = self.get_parameter('horizontal_angle').value

        v = self.get_parameter('vertical_angle').value

        self.client = self.create_service(
            Trigger, '/hydrodynamics/set_current_velocity'
        )

        while not self.client.wait_for_request(timeout_sec=2.0):
            self.get_logger().info('Waiting for request...')

        req = Trigger.Request()

        req.request_id = Int32(data=1)

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Current perturbation applied')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    TimedCurrent()


if __name__ == '__main__':
    main()

