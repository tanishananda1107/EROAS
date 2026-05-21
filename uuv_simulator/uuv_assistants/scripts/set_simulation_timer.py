#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

import time


class SimulationTimer(Node):

    def __init__(self):

        super().__init__(
            "simulation_timer"
        )

        timeout=self.declare_parameter(
            "timeout",
            10.0
        ).value

        time.sleep(timeout)

        self.get_logger().info(
            "timeout reached"
        )


def main():

    rclpy.init()

    node=SimulationTimer()

    rclpy.shutdown()


if __name__=="__main__":
    main()
