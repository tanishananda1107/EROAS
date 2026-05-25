#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from numpy import pi

from uuv_world_ros_plugins_msgs.srv import SetCurrentModel


class GMCurrentPerturbation(Node):

    def __init__(self):

        super().__init__(
            'set_gm_current_perturbation'
        )

        params = [
            'component',
            'mean',
            'min',
            'max',
            'noise',
            'mu'
        ]

        for p in params:
            self.declare_parameter(p)

        values = {}

        for p in params:
            values[p] = self.get_parameter(
                p
            ).value

        srv = (
            f'/hydrodynamics/'
            f'set_current_'
            f'{values["component"]}_model'
        )

        client = self.create_client(
            SetCurrentModel,
            srv
        )

        while not client.wait_for_service(
                2.0):
            pass

        req = SetCurrentModel.Request()

        req.mean = values['mean']
        req.min = values['min']
        req.max = values['max']
        req.noise = values['noise']
        req.mu = values['mu']

        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main():

    rclpy.init()

    node = GMCurrentPerturbation()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
