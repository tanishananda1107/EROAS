#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

from sensor_msgs_py import point_cloud2 as pc2

import numpy as np


class SonarObsAvoidanceMergedOld(Node):

    SAFE_DISTANCE = 2.0

    def __init__(self):

        super().__init__(
            'sonar_obs_avoidance_merged_old'
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/rexrov2/sonar_pointcloud',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/rexrov2/cmd_vel',
            10
        )

        self.get_logger().info(
            'Sonar Obstacle Avoidance Started'
        )

    def callback(self, msg):

        cmd = Twist()

        points = list(

            pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        )

        if len(points) == 0:

            cmd.linear.x = 0.5

            self.publisher.publish(cmd)

            return

        arr = np.array(points)

        dists = np.linalg.norm(
            arr,
            axis=1
        )

        nearest = np.min(dists)

        if nearest < self.SAFE_DISTANCE:

            left = np.sum(arr[:, 1] > 0)
            right = np.sum(arr[:, 1] < 0)

            cmd.linear.x = 0.1

            if left > right:

                cmd.angular.z = -0.5

            else:

                cmd.angular.z = 0.5

        else:

            cmd.linear.x = 0.5

        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = SonarObsAvoidanceMergedOld()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
