#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from visualization_msgs.msg import MarkerArray


class WorldPublisher(Node):

    def __init__(self):

        super().__init__("publish_world_models")

        self.pub=self.create_publisher(
            MarkerArray,
            "world_models",
            10
        )

        self.timer=self.create_timer(
            10.0,
            self.publish_models
        )

    def publish_models(self):

        self.pub.publish(
            MarkerArray()
        )


def main():

    rclpy.init()

    node=WorldPublisher()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__=="__main__":
    main()
