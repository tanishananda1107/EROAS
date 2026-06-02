#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from uuv_gazebo_ros_plugins_msgs.srv import (
    SetThrusterState
)


class ThrusterState(Node):

    def __init__(self):

        super().__init__(
            'set_thruster_state'
        )

        self.declare_parameter(
            'thruster_id',
            0
        )

        self.declare_parameter(
            'is_on',
            True
        )

        thruster = self.get_parameter(
            'thruster_id'
        ).value

        state = self.get_parameter(
            'is_on'
        ).value

        vehicle = self.get_namespace(
        ).replace('/', '')

        srv = (
            f'/{vehicle}/thrusters/'
            f'{thruster}/'
            f'set_thruster_state'
        )

        client = self.create_client(
            SetThrusterState,
            srv
        )

        while not client.wait_for_service(
                2.0):
            pass

        req = SetThrusterState.Request()

        req.on = bool(state)

        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main():

    rclpy.init()

    node = ThrusterState()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
