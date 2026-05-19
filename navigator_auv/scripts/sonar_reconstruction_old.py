#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

from sensor_msgs_py import point_cloud2 as pc2

import sensor_msgs_py.point_cloud2 as pcl2

import numpy as np


class SonarReconstructionOld(Node):

    def __init__(self):

        super().__init__(
            'sonar_reconstruction_old'
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/rexrov2/sonar_pointcloud',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/sonar_reconstruction_old',
            10
        )

        self.get_logger().info(
            'Sonar Reconstruction Old Started'
        )

    def callback(self, msg):

        points = list(

            pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        )

        if len(points) == 0:

            return

        arr = np.array(points)

        filtered = arr[
            arr[:, 0] > 0
        ]

        cloud = pcl2.create_cloud_xyz32(
            msg.header,
            filtered.tolist()
        )

        self.publisher.publish(cloud)


def main(args=None):

    rclpy.init(args=args)

    node = SonarReconstructionOld()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
