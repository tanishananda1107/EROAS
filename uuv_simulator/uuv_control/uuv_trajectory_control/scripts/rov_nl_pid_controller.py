#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces import (
    DPPIDControllerBase
)


class ROVNLPIDController(
        Node,
        DPPIDControllerBase):

    _LABEL = (
        'MIMO Nonlinear PID'
    )

    def __init__(self):

        Node.__init__(
            self,
            'rov_nl_pid_controller'
        )

        DPPIDControllerBase.__init__(
            self,
            self,
            True
        )

        self._Hm = np.eye(6)

        self.declare_parameter(
            'Hm',
            [0.0] * 6
        )

        hm = self.get_parameter(
            'Hm'
        ).value

        self._Hm = (
            self._vehicle_model.Mtotal
            +
            np.diag(hm)
        )

        self._tau = np.zeros(6)

        self._accel_ff = np.zeros(
            6
        )

        self._pid_control = np.zeros(
            6
        )

        self._is_init = True

    def update_controller(self):

        if not self._is_init:
            return False

        acc = (
            self._vehicle_model.compute_acc(
                self._vehicle_model.to_SNAME(
                    self._tau
                ),
                use_sname=False
            )
        )

        self._accel_ff = np.dot(
            self._Hm,
            acc
        )

        self._pid_control = (
            self.update_pid()
        )

        self._tau = (
            self._pid_control
            -
            self._accel_ff
            +
            self._vehicle_model.restoring_forces
        )

        self.publish_control_wrench(
            self._tau
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = (
        ROVNLPIDController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
