#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from uuv_control_interfaces import DPControllerBase


class ROVPDGCompController(
        Node,
        DPControllerBase):

    _LABEL = (
        'PD controller with '
        'compensation of restoring forces'
    )

    def __init__(self):

        Node.__init__(
            self,
            'rov_pd_grav_compensation_controller'
        )

        DPControllerBase.__init__(
            self,
            is_model_based=True
        )

        self.get_logger().info(
            self._LABEL
        )

        self._Kp = np.zeros(
            (6, 6)
        )

        self._Kd = np.zeros(
            (6, 6)
        )

        self._tau = np.zeros(6)

        self.declare_parameter(
            'Kp',
            [0.0] * 6
        )

        self.declare_parameter(
            'Kd',
            [0.0] * 6
        )

        kp_diag = self.get_parameter(
            'Kp'
        ).value

        kd_diag = self.get_parameter(
            'Kd'
        ).value

        self._Kp = np.diag(
            kp_diag
        )

        self._Kd = np.diag(
            kd_diag
        )

        self._is_init = True

    def update_controller(self):

        if not self._is_init:
            return False

        self._vehicle_model._update_restoring(
            use_sname=True
        )

        self._tau = (
            np.dot(
                self._Kp,
                self.error_pose_euler
            )
            +
            np.dot(
                self._Kd,
                self._errors['vel']
            )
            +
            self._vehicle_model.restoring_forces
        )

        self.publish_control_wrench(
            self._tau
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = ROVPDGCompController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
