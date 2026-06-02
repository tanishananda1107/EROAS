#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node

from uuv_control_interfaces.dp_controller_base import (
    DPControllerBase
)

from uuv_control_msgs.srv import (
    SetPIDParams,
    GetPIDParams
)


class ROVUnderActuatedPIDController(
        Node,
        DPControllerBase):

    def __init__(self):

        Node.__init__(
            self,
            "rov_ua_pid_controller"
        )

        DPControllerBase.__init__(
            self,
            self
        )

        self.get_logger().info(
            "Initializing Underactuated PID"
        )

        self._Kp = np.zeros(
            (4, 4)
        )

        self._Kd = np.zeros(
            (4, 4)
        )

        self._Ki = np.zeros(
            (4, 4)
        )

        self._int = np.zeros(
            4
        )

        self._error_pose = np.zeros(
            4
        )

        self._tau = np.zeros(
            6
        )

        self.declare_parameter(
            "Kp",
            [0.0] * 4
        )

        self.declare_parameter(
            "Kd",
            [0.0] * 4
        )

        self.declare_parameter(
            "Ki",
            [0.0] * 4
        )

        self._Kp = np.diag(
            self.get_parameter(
                "Kp"
            ).value
        )

        self._Kd = np.diag(
            self.get_parameter(
                "Kd"
            ).value
        )

        self._Ki = np.diag(
            self.get_parameter(
                "Ki"
            ).value
        )

        self.create_service(
            SetPIDParams,
            "set_pid_params",
            self.set_pid_callback
        )

        self.create_service(
            GetPIDParams,
            "get_pid_params",
            self.get_pid_callback
        )

        self._is_init = True

    def set_pid_callback(
            self,
            req,
            res):

        self._Kp = np.diag(
            req.kp
        )

        self._Kd = np.diag(
            req.kd
        )

        self._Ki = np.diag(
            req.ki
        )

        res.success = True

        return res

    def get_pid_callback(
            self,
            req,
            res):

        res.kp = [
            self._Kp[i, i]
            for i in range(4)
        ]

        res.kd = [
            self._Kd[i, i]
            for i in range(4)
        ]

        res.ki = [
            self._Ki[i, i]
            for i in range(4)
        ]

        return res

    def update_controller(self):

        if not self._is_init:

            return False

        cur_error = np.array(

            [
                self.error_pose_euler[0],
                self.error_pose_euler[1],
                self.error_pose_euler[2],
                self.error_pose_euler[5]
            ]

        )

        self._int += (

            0.5
            *
            (
                cur_error
                +
                self._error_pose
            )
            *
            self._dt

        )

        self._error_pose = cur_error

        error_vel = np.array(

            [
                self._errors['vel'][0],
                self._errors['vel'][1],
                self._errors['vel'][2],
                self._errors['vel'][5]
            ]

        )

        ua_tau = (

            np.dot(
                self._Kp,
                cur_error
            )

            +

            np.dot(
                self._Kd,
                error_vel
            )

            +

            np.dot(
                self._Ki,
                self._int
            )

        )

        self._tau = np.array(

            [
                ua_tau[0],
                ua_tau[1],
                ua_tau[2],
                0,
                0,
                ua_tau[3]
            ]

        )

        self.publish_control_wrench(
            self._tau
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = (
        ROVUnderActuatedPIDController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
