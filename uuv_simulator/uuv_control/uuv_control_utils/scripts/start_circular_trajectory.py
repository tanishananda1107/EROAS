#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from uuv_control_msgs.srv import InitCircularTrajectory


class CircularTrajectory(Node):

    def __init__(self):

        super().__init__(
            'start_circular_trajectory'
        )

        params = [
            'radius',
            'center',
            'n_points',
            'heading_offset',
            'duration',
            'max_forward_speed',
            'start_time'
        ]

        for p in params:
            self.declare_parameter(p)

        self.client = self.create_client(
            InitCircularTrajectory,
            'start_circular_trajectory'
        )

        while not self.client.wait_for_service(
                timeout_sec=2.0):

            self.get_logger().info(
                'Waiting service'
            )

        self.send()

    def send(self):

        req = InitCircularTrajectory.Request()

        center = self.get_parameter(
            'center').value

        req.start_now = True

        req.radius = float(
            self.get_parameter(
                'radius').value
        )

        req.center = Point(
            x=center[0],
            y=center[1],
            z=center[2]
        )

        req.is_clockwise = False

        req.angle_offset = 0.0

        req.n_points = int(
            self.get_parameter(
                'n_points').value
        )

        req.heading_offset = math.radians(
            self.get_parameter(
                'heading_offset').value
        )

        req.max_forward_speed = float(
            self.get_parameter(
                'max_forward_speed').value
        )

        req.duration = float(
            self.get_parameter(
                'duration').value
        )

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            'Circular trajectory created'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CircularTrajectory()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
