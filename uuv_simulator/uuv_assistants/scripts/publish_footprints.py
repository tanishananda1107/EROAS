#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32

from tf_transformations import euler_from_quaternion


class PublishFootprints(Node):

    MARKER=np.array([
        [0,0.75],
        [-0.5,-0.25],
        [0.5,-0.25]
    ])

    def __init__(self):

        super().__init__("publish_footprints")

        self.pub=self.create_publisher(
            PolygonStamped,
            "footprint",
            10
        )

        self.sub=self.create_subscription(
            Odometry,
            "odom",
            self.callback,
            10
        )

    def rot(self,a):

        return np.array([
            [np.cos(a),-np.sin(a)],
            [np.sin(a),np.cos(a)]
        ])

    def callback(self,msg):

        q=[
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        yaw=euler_from_quaternion(q)[2]

        marker=deepcopy(self.MARKER)

        pts=[]

        for i in range(marker.shape[0]):

            marker[i,:]=np.dot(
                self.rot(yaw-np.pi/2),
                marker[i,:]
            )

            marker[i,0]+=msg.pose.pose.position.x
            marker[i,1]+=msg.pose.pose.position.y

            p=Point32()

            p.x=float(marker[i,0])
            p.y=float(marker[i,1])

            pts.append(p)

        poly=PolygonStamped()

        poly.header.frame_id="world"

        poly.polygon.points=pts

        self.pub.publish(poly)


def main():

    rclpy.init()

    node=PublishFootprints()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
