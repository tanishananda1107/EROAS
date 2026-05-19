#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker


class VehicleFootprint(Node):

    MARKER = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

    def __init__(self):
        super().__init__('vehicle_footprint')

        self.namespace = self.get_namespace().replace('/', '')

        self.scale = 10.0
        self.label_scale = 10.0
        self.label_offset = 60.0

        self.sub = self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            10
        )

        self.pub_fp = self.create_publisher(PolygonStamped, 'footprint', 10)
        self.pub_label = self.create_publisher(Marker, 'label', 10)

        self.label = Marker()
        self.label.header.frame_id = 'world'
        self.label.ns = self.namespace
        self.label.type = Marker.TEXT_VIEW_FACING
        self.label.text = self.namespace
        self.label.scale.z = self.label_scale
        self.label.color.a = 1.0
        self.label.color.g = 1.0

    def rot(self, a):
        return np.array([[np.cos(a), -np.sin(a)],
                         [np.sin(a), np.cos(a)]])

    def quat_to_yaw(self, q):
        import math
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = self.quat_to_yaw(msg.pose.pose.orientation)

        marker = deepcopy(self.MARKER) * self.scale

        points = []
        for i in range(3):
            marker[i] = self.rot(yaw - np.pi/2).dot(marker[i])
            marker[i][0] += x
            marker[i][1] += y

            p = Point32()
            p.x, p.y = float(marker[i][0]), float(marker[i][1])
            points.append(p)

        poly = PolygonStamped()
        poly.header.frame_id = 'world'
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.polygon.points = points

        self.pub_fp.publish(poly)

        self.label.pose.position.x = x + self.label_offset
        self.label.pose.position.y = y

        self.pub_label.publish(self.label)


def main():
    rclpy.init()
    node = VehicleFootprint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
