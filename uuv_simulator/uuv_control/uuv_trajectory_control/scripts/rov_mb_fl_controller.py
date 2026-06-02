#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces import (
    DPPIDControllerBase
)


class ROVMBFLController(
        Node,
        DPPIDControllerBase):

    _LABEL = (
        'Model Based FL'
    )

    def __init__(self):

        Node.__init__(
            self,
            'rov_mb_fl_controller'
        )

        DPPIDControllerBase.__init__(
            self,
            self,
            True
        )

        self._tau = np.zeros(
            6
        )

        self._pid_control = np.zeros(
            6
        )

        self._last_vel = np.zeros(
            6
        )

        self._last_t = None

        self._is_init = True

    def update_controller(self):

        if not self._is_init:
            return False

        t = (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

        if self._last_t is None:

            self._last_t = t

            self._last_vel = (
                self._vehicle_model
                .to_SNAME(
                    self._reference[
                        'vel'
                    ]
                )
            )

            return False

        dt = t - self._last_t

        if dt <= 0:

            return False

        self._pid_control = (
            self.update_pid()
        )

        vel = (
            self._vehicle_model
            .to_SNAME(
                self._reference[
                    'vel'
                ]
            )
        )

        acc = (
            vel
            -
            self._last_vel
        ) / dt

        self._vehicle_model._update_damping(
            vel
        )

        self._vehicle_model._update_coriolis(
            vel
        )

        self._vehicle_model._update_restoring(
            q=self._reference[
                'rot'
            ],
            use_sname=True
        )

        self._tau = (
            np.dot(
                self._vehicle_model.Mtotal,
                acc
            )
            +
            np.dot(
                self._vehicle_model.Ctotal,
                vel
            )
            +
            np.dot(
                self._vehicle_model.Dtotal,
                vel
            )
            +
            self._vehicle_model.restoring_forces
        )

        self.publish_control_wrench(
            self._pid_control
            +
            self._vehicle_model
            .from_SNAME(
                self._tau
            )
        )

        self._last_t = t

        self._last_vel = vel

        return True


def main(args=None):

    rclpy.init(args=args)

    node = (
        ROVMBFLController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
