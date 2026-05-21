#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from copy import deepcopy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PolygonStamped,Point32

from visualization_msgs.msg import Marker

from tf_transformations import euler_from_quaternion


class VehicleFootprint(Node):

    MARKER=np.array([
        [0,0.75],
        [-0.5,-0.25],
        [0.5,-0.25]
    ])

    def __init__(self):

        super().__init__("generate_vehicle_footprint")

        self.scale=self.declare_parameter(
            "scale_footprint",
            10.0
        ).value

        self.label_scale=self.declare_parameter(
            "scale_label",
            10.0
        ).value

        self.offset=self.declare_parameter(
            "label_x_offset",
            60.0
        ).value

        self.fp_pub=self.create_publisher(
            PolygonStamped,
            "footprint",
            10
        )

        self.label_pub=self.create_publisher(
            Marker,
            "label",
            10
        )

        self.sub=self.create_subscription(
            Odometry,
            "odom",
            self.callback,
            10
        )

    def callback(self,msg):

        pass


def main():

    rclpy.init()

    node=VehicleFootprint()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__=="__main__":
    main()
