#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from uuv_control_interfaces import DPPIDControllerBase


class ROVPIDController(Node, DPPIDControllerBase):
    _LABEL = 'PID'

    def __init__(self):
        Node.__init__(self, 'rov_pid_controller')

        self._tau = np.zeros(6)

        DPPIDControllerBase.__init__(
            self,
            self,
            False,
            None,
            True
        )

        self._is_init = True

    def update_controller(self):
        if not self._is_init:
            return False

        self._tau = self.update_pid()

        self.publish_control_wrench(
            self._tau
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = ROVPIDController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
