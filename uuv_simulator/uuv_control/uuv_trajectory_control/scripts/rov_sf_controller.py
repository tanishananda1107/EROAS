#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces import (
    DPControllerBase
)


class ROVSFController(
        Node,
        DPControllerBase):

    def __init__(self):

        Node.__init__(
            self,
            "rov_sf_controller"
        )

        DPControllerBase.__init__(
            self,
            self,
            True
        )

        self._tau = np.zeros(6)

        self._Kd = np.eye(6)

        self._delta = np.eye(6)

        self._prev_t = None

        self._prev_vel_r = np.zeros(
            6
        )

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

        error = np.hstack(
            (
                self._errors[
                    'pos'
                ],
                self.error_orientation_quat
            )
        )

        vel_r = (
            self._reference[
                'vel'
            ]
            +
            np.dot(
                self._delta,
                error
            )
        )

        if self._prev_t is None:

            self._prev_t = t

            self._prev_vel_r = vel_r

            return False

        dt = t - self._prev_t

        if dt <= 0:

            return False

        s = (
            self._errors[
                'vel'
            ]
            +
            np.dot(
                self._delta,
                error
            )
        )

        d_vel = (
            vel_r
            -
            self._prev_vel_r
        ) / dt

        self._tau = (
            np.dot(
                self._vehicle_model.Mtotal,
                d_vel
            )
        )

        self.publish_control_wrench(
            self._tau
            +
            np.dot(
                self._Kd,
                s
            )
        )

        self._prev_t = t

        self._prev_vel_r = vel_r

        return True


def main(args=None):

    rclpy.init(args=args)

    node = ROVSFController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
