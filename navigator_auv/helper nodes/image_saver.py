#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2


class ImageSaver(Node):

    def __init__(self):

        super().__init__('image_saver')

        self.bridge = CvBridge()

        self.count = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        self.get_logger().info(
            'Image saver node started'
        )

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        filename = (
            f'saved_{self.count:04d}.jpg'
        )

        cv2.imwrite(filename, frame)

        self.get_logger().info(
            f'Saved {filename}'
        )

        self.count += 1


def main(args=None):

    rclpy.init(args=args)

    node = ImageSaver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
