#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point

from uuv_control_msgs.srv import (
    InitHelicalTrajectory
)


class HelicalTrajectory(Node):

    def __init__(self):

        super().__init__(
            'start_helical_trajectory'
        )

        params = [
            'radius',
            'center',
            'n_points',
            'heading_offset',
            'duration',
            'n_turns',
            'delta_z',
            'max_forward_speed'
        ]

        for p in params:
            self.declare_parameter(p)

        self.client = self.create_client(
            InitHelicalTrajectory,
            'start_helical_trajectory'
        )

        while not self.client.wait_for_service(
                2.0):

            pass

        self.run()

    def run(self):

        center = self.get_parameter(
            'center').value

        req = InitHelicalTrajectory.Request()

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

        req.n_turns = float(
            self.get_parameter(
                'n_turns').value
        )

        req.delta_z = float(
            self.get_parameter(
                'delta_z').value
        )

        future = self.client.call_async(
            req
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main(args=None):

    rclpy.init(args=args)

    node = HelicalTrajectory()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
