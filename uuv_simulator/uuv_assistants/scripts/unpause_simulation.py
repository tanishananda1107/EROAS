#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

import time

import subprocess


class Unpause(Node):

    def __init__(self):

        super().__init__(
            "unpause_simulation"
        )

        timeout=self.declare_parameter(
            "timeout",
            0.0
        ).value

        time.sleep(timeout)

        subprocess.run([
            "gz",
            "service",
            "-s",
            "/world/default/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "pause:false"
        ])


def main():

    rclpy.init()

    node=Unpause()

    rclpy.shutdown()


if __name__=="__main__":
    main()
