#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2


class ImageResizer(Node):

    def __init__(self):

        super().__init__('image_resizer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/camera/resized',
            10
        )

        self.get_logger().info(
            'Image resizer started'
        )

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        resized = cv2.resize(
            frame,
            (320, 240)
        )

        out_msg = self.bridge.cv2_to_imgmsg(
            resized,
            encoding='bgr8'
        )

        self.publisher.publish(out_msg)


def main(args=None):

    rclpy.init(args=args)

    node = ImageResizer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
