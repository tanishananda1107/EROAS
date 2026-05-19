#!/usr/bin/env python3

import os

import cv2

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge


class CameraImageSaver(Node):

    SAVE_DIR = os.path.expanduser(
        '~/eroas_images'
    )

    def __init__(self):

        super().__init__('camera_image_saver')

        os.makedirs(
            self.SAVE_DIR,
            exist_ok=True
        )

        self.bridge = CvBridge()

        self.counter = 0

        self.subscription = self.create_subscription(
            Image,
            '/rexrov2/camera/image_raw',
            self.callback,
            10
        )

        self.get_logger().info(
            'Camera image saver started'
        )

    def callback(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            filename = os.path.join(
                self.SAVE_DIR,
                f'image_{self.counter:06d}.png'
            )

            cv2.imwrite(
                filename,
                image
            )

            self.get_logger().info(
                f'Saved {filename}'
            )

            self.counter += 1

        except Exception as e:

            self.get_logger().error(str(e))


def main(args=None):

    rclpy.init(args=args)

    node = CameraImageSaver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
