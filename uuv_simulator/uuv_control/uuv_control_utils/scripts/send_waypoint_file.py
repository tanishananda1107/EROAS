#!/usr/bin/env python3

import sys
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time

from uuv_control_msgs.srv import InitWaypointsFromFile


class SendWaypointFile(Node):

    def __init__(self):

        super().__init__('send_waypoint_file')

        self.declare_parameter('filename', '')
        self.declare_parameter('start_time', -1.0)
        self.declare_parameter('interpolator', 'lipb')

        filename = self.get_parameter(
            'filename').value

        if filename == '':
            raise RuntimeError('Filename missing')

        start_time = self.get_parameter(
            'start_time').value

        start_now = start_time < 0

        interpolator = self.get_parameter(
            'interpolator').value

        client = self.create_client(
            InitWaypointsFromFile,
            'init_waypoints_from_file'
        )

        while not client.wait_for_service(2.0):
            self.get_logger().info('Waiting...')

        req = InitWaypointsFromFile.Request()

        req.start_now = start_now

        req.filename = String(data=filename)

        req.interpolator = String(
            data=interpolator
        )

        req.start_time = Time(
            sec=int(start_time)
        )

        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            f'Waypoint file loaded {filename}'
        )


def main():

    rclpy.init()

    node = SendWaypointFile()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
