#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces import (
    DPControllerBase
)


class ROVNMBSMController(
        Node,
        DPControllerBase):

    _LABEL = (
        "Non Model Sliding"
    )

    def __init__(self):

        Node.__init__(
            self,
            "rov_nmb_sm_controller"
        )

        DPControllerBase.__init__(
            self,
            self,
            is_model_based=False
        )

        self.declare_parameter(
            "K",
            [0.0]*6
        )

        self.declare_parameter(
            "Kd",
            [0.0]*6
        )

        self.declare_parameter(
            "Ki",
            [0.0]*6
        )

        self.declare_parameter(
            "slope",
            [0.0]*6
        )

        self._K = np.array(
            self.get_parameter(
                "K"
            ).value
        )

        self._Kd = np.array(
            self.get_parameter(
                "Kd"
            ).value
        )

        self._Ki = np.array(
            self.get_parameter(
                "Ki"
            ).value
        )

        self._slope = np.array(
            self.get_parameter(
                "slope"
            ).value
        )

        self._tau = np.zeros(6)

        self._prev_t = -1.0

        self._int_lin = np.zeros(3)

        self._int_ang = np.zeros(3)

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

        dt = (
            t
            -
            self._prev_t
        )

        if self._prev_t < 0:

            dt = 0

        ep_lin = (
            self._errors[
                "pos"
            ]
        )

        ev_lin = (
            self._errors[
                "vel"
            ][0:3]
        )

        ep_ang = (
            self.error_orientation_rpy
        )

        ev_ang = (
            self._errors[
                "vel"
            ][3:6]
        )

        s_lin = (
            -ev_lin
            -
            self._slope[0:3]
            * ep_lin
        )

        s_ang = (
            -ev_ang
            -
            self._slope[3:6]
            * ep_ang
        )

        self._int_lin += (
            np.sign(
                s_lin
            )
            * dt
        )

        self._int_ang += (
            np.sign(
                s_ang
            )
            * dt
        )

        sr_lin = (
            s_lin
            +
            self._Ki[0:3]
            *
            self._int_lin
        )

        sr_ang = (
            s_ang
            +
            self._Ki[3:6]
            *
            self._int_ang
        )

        force = (
            -self._Kd[0:3]
            * sr_lin
        )

        torque = (
            -self._Kd[3:6]
            * sr_ang
        )

        self._tau = np.hstack(
            (
                force,
                torque
            )
        )

        self.publish_control_wrench(
            self._tau
        )

        self._prev_t = t

        return True


def main(args=None):

    rclpy.init(args=args)

    node = (
        ROVNMBSMController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
