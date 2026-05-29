#!/usr/bin/env python3

# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
import numpy as np

from uuv_control_interfaces.dp_controller_base import DPControllerBase


class TutorialDPController(DPControllerBase):
    """
    ROS 2 + Gazebo Harmonic / GZ Sim 8 version
    of the tutorial dynamic positioning controller.
    """

    def __init__(self):
        super().__init__(is_model_based=False)

        self.get_logger().info('Initializing TutorialDPController')

        # Gain matrices
        self._Kp = np.zeros((6, 6))
        self._Kd = np.zeros((6, 6))
        self._Ki = np.zeros((6, 6))

        # Integrator state
        self._int = np.zeros(6)

        # Previous pose error
        self._error_pose = np.zeros(6)

        # Declare ROS 2 parameters
        self.declare_parameter(
            'Kp',
            [11993.888, 11993.888, 11993.888,
             19460.069, 19460.069, 19460.069]
        )

        self.declare_parameter(
            'Kd',
            [9077.459, 9077.459, 9077.459,
             18880.925, 18880.925, 18880.925]
        )

        self.declare_parameter(
            'Ki',
            [321.417, 321.417, 321.417,
             2096.951, 2096.951, 2096.951]
        )

        # Read parameters
        kp_diag = self.get_parameter('Kp').value
        kd_diag = self.get_parameter('Kd').value
        ki_diag = self.get_parameter('Ki').value

        # Validate parameter sizes
        if len(kp_diag) != 6:
            raise RuntimeError('Kp must contain 6 coefficients')

        if len(kd_diag) != 6:
            raise RuntimeError('Kd must contain 6 coefficients')

        if len(ki_diag) != 6:
            raise RuntimeError('Ki must contain 6 coefficients')

        # Build diagonal matrices
        self._Kp = np.diag(kp_diag)
        self._Kd = np.diag(kd_diag)
        self._Ki = np.diag(ki_diag)

        self.get_logger().info(f'Kp=\n{self._Kp}')
        self.get_logger().info(f'Kd=\n{self._Kd}')
        self.get_logger().info(f'Ki=\n{self._Ki}')

        self._is_init = True

    def _reset_controller(self):
        """
        Reset controller internal states.
        """

        super()._reset_controller()

        self._error_pose = np.zeros(6)
        self._int = np.zeros(6)

        self.get_logger().info('Controller reset')

    def update_controller(self):
        """
        Main PID control loop.
        """

        if not self._is_init:
            return False

        # Wait until odometry is initialized
        if not self.odom_is_init:
            return False

        # Integrate pose error
        self._int = (
            self._int
            + 0.5
            * (self.error_pose_euler + self._error_pose)
            * self._dt
        )

        # Store previous error
        self._error_pose = self.error_pose_euler.copy()

        # PID control law
        tau = (
            np.dot(self._Kp, self.error_pose_euler)
            + np.dot(self._Kd, self._errors['vel'])
            + np.dot(self._Ki, self._int)
        )

        # Publish wrench command
        self.publish_control_wrench(tau)

        return True


def main(args=None):
    rclpy.init(args=args)

    print('Tutorial - DP Controller (ROS 2 / Gazebo Harmonic)')

    try:
        node = TutorialDPController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'Exception: {e}')

    finally:
        rclpy.shutdown()

    print('Exiting')


if __name__ == '__main__':
    main()
