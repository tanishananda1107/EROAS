#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces import (
    DPControllerBase
)

from uuv_control_interfaces.vehicle import (
    cross_product_operator
)

from uuv_control_msgs.srv import (
    SetMBSMControllerParams,
    GetMBSMControllerParams
)


class ROVMBSMController(
        Node,
        DPControllerBase):

    _LABEL = (
        'Model Based Sliding Mode'
    )

    def __init__(self):

        Node.__init__(
            self,
            'rov_mb_sm_controller'
        )

        DPControllerBase.__init__(
            self,
            True
        )

        self.declare_parameter(
            'lambda',
            [0.0]*6
        )

        self.declare_parameter(
            'rho_constant',
            [0.0]*6
        )

        self.declare_parameter(
            'k',
            [0.0]*6
        )

        self.declare_parameter(
            'c',
            [0.0]*6
        )

        self.declare_parameter(
            'adapt_slope',
            [0.0]*3
        )

        self.declare_parameter(
            'rho_0',
            [0.0]*6
        )

        self.declare_parameter(
            'drift_prevent',
            0.0
        )

        self._lambda = np.array(
            self.get_parameter(
                'lambda'
            ).value
        )

        self._rho_constant = np.array(
            self.get_parameter(
                'rho_constant'
            ).value
        )

        self._k = np.array(
            self.get_parameter(
                'k'
            ).value
        )

        self._c = np.array(
            self.get_parameter(
                'c'
            ).value
        )

        self._adapt_slope = np.array(
            self.get_parameter(
                'adapt_slope'
            ).value
        )

        self._rho_0 = np.array(
            self.get_parameter(
                'rho_0'
            ).value
        )

        self._drift_prevent = (
            self.get_parameter(
                'drift_prevent'
            ).value
        )

        self._int = np.zeros(6)

        self._rho_adapt = np.zeros(
            6
        )

        self._tau = np.zeros(
            6
        )

        self._prev_t = -1.0

        self.create_service(
            SetMBSMControllerParams,
            "set_mb_sm_controller_params",
            self.set_callback
        )

        self.create_service(
            GetMBSMControllerParams,
            "get_mb_sm_controller_params",
            self.get_callback
        )

        self._is_init = True

    def set_callback(
            self,
            req,
            res):

        res.success = True

        return res

    def get_callback(
            self,
            req,
            res):

        res.lambda_values = (
            self._lambda.tolist()
        )

        return res

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
                'pos'
            ]
        )

        ev_lin = (
            self._errors[
                'vel'
            ][0:3]
        )

        ep_ang = (
            self.error_orientation_rpy
        )

        ev_ang = (
            self._errors[
                'vel'
            ][3:6]
        )

        ep = np.hstack(
            (
                ep_lin,
                ep_ang
            )
        )

        ev = np.hstack(
            (
                ev_lin,
                ev_ang
            )
        )

        s = (
            -ev
            -
            self._lambda
            * ep
        )

        rot_dot = np.dot(
            cross_product_operator(
                self._vehicle_model
                ._vel[3:6]
            ),
            self._vehicle_model
            .rotBtoI
        )

        acc_est = (
            self._reference[
                'acc'
            ]
            +
            self._lambda * ev
        )

        acc = (
            self._vehicle_model
            .to_SNAME(
                acc_est
            )
        )

        f_eq = (
            self._vehicle_model
            .compute_force(
                acc,
                use_sname=False
            )
        )

        f_lin = (
            -self._k * s
        )

        rho_total = (
            self._rho_adapt
            +
            self._rho_constant
        )

        self._rho_adapt += (

            self._adapt_slope[0]
            *
            np.abs(s)

        ) * dt

        f_robust = (

            -rho_total
            *
            (
                2
                /
                np.pi
            )
            *
            np.arctan(
                self._c
                * s
            )
        )

        self._tau = (
            f_eq
            +
            f_lin
            +
            f_robust
        )

        self.publish_control_wrench(
            self._tau
        )

        self._prev_t = t

        return True


def main(args=None):

    rclpy.init(args=args)

    node = (
        ROVMBSMController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
