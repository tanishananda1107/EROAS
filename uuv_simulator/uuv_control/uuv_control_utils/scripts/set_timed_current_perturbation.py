#!/usr/bin/env python3

import sys
import math
import time

import rclpy
from rclpy.node import Node

from uuv_world_ros_plugins_msgs.srv import SetCurrentVelocity


class TimedCurrentPerturbation(Node):

    def __init__(self):
        super().__init__('set_timed_current_perturbation')

        self.declare_parameter('starting_time', 0.0)
        self.declare_parameter('end_time', -1.0)
        self.declare_parameter('current_velocity', 0.0)
        self.declare_parameter('horizontal_angle', 0.0)
        self.declare_parameter('vertical_angle', 0.0)

        self.starting_time = self.get_parameter(
            'starting_time').value

        self.end_time = self.get_parameter(
            'end_time').value

        self.vel = self.get_parameter(
            'current_velocity').value

        self.horz_angle = math.radians(
            self.get_parameter('horizontal_angle').value
        )

        self.vert_angle = math.radians(
            self.get_parameter('vertical_angle').value
        )

        self.client = self.create_client(
            SetCurrentVelocity,
            '/hydrodynamics/set_current_velocity'
        )

        while not self.client.wait_for_service(2.0):
            self.get_logger().info(
                'Waiting current service...'
            )

        self.run()

    def set_current(self, vel):

        req = SetCurrentVelocity.Request()

        req.velocity = float(vel)
        req.horizontal_angle = self.horz_angle
        req.vertical_angle = self.vert_angle

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        return future.result()

    def run(self):

        while time.time() < self.starting_time:
            time.sleep(0.01)

        self.get_logger().info(
            'Applying current perturbation'
        )

        self.set_current(self.vel)

        if self.end_time > 0:

            while time.time() < self.end_time:
                time.sleep(0.01)

            self.get_logger().info(
                'Stopping perturbation'
            )

            self.set_current(0.0)


def main(args=None):

    rclpy.init(args=args)

    node = TimedCurrentPerturbation()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
