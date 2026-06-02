#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from uuv_gazebo_ros_plugins_msgs.srv import SetFloat


class ScalarParameter(Node):

    def __init__(self):

        super().__init__(
            'set_scalar_parameter'
        )

        self.declare_parameter(
            'service_name'
        )

        self.declare_parameter(
            'data'
        )

        service_name = self.get_parameter(
            'service_name'
        ).value

        data = self.get_parameter(
            'data'
        ).value

        client = self.create_client(
            SetFloat,
            service_name
        )

        while not client.wait_for_service(
                2.0):
            pass

        req = SetFloat.Request()

        req.data = float(data)

        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main():

    rclpy.init()

    node = ScalarParameter()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
