
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from uuv_world_ros_plugins_msgs.srv import SetCurrentModel


class SetGMCurrent(Node):

    def __init__(self):
        super().__init__('set_gm_current_perturbation')

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

        values = {
            p: self.get_parameter(p).value for p in params
        }

        service_name = (
            f"/hydrodynamics/set_current_"
            f"{values['component']}_model"
        )

        self.client = self.create_service_client(
            SetCurrentModel,
            service_name
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = SetCurrentModel.Request()

        req.mean = float(values['mean'])
        req.min = float(values['min'])
        req.max = float(values['max'])
        req.noise = float(values['noise'])
        req.mu = float(values['mu'])

        future = self.client.call(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Current model updated')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    SetGMCurrent()

if __name__ == '__main__':
    main()

Note that I removed the `from math import pi` line as it is not used in thi[3D[K
this code.

