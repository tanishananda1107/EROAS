#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from uuv_gazebo_ros_plugins_msgs.srv import (
    SetThrusterEfficiency
)


class ThrusterEfficiency(Node):

    def __init__(self):

        super().__init__(
            'set_thruster_output_efficiency'
        )

        self.declare_parameter(
            'thruster_id',
            0
        )

        self.declare_parameter(
            'efficiency',
            1.0
        )

        thruster = self.get_parameter(
            'thruster_id'
        ).value

        eff = self.get_parameter(
            'efficiency'
        ).value

        vehicle = self.get_namespace(
        ).replace('/', '')

        srv = (
            f'/{vehicle}/thrusters/'
            f'{thruster}/'
            f'set_thrust_force_efficiency'
        )

        client = self.create_client(
            SetThrusterEfficiency,
            srv
        )

        while not client.wait_for_service(
                2.0):
            pass

        req = SetThrusterEfficiency.Request()

        req.efficiency = eff

        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main():

    rclpy.init()

    node = ThrusterEfficiency()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
