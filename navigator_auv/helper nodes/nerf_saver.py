#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2

import os


class NerfSaver(Node):

    def __init__(self):

        super().__init__('nerf_saver')

        self.bridge = CvBridge()

        self.index = 0

        os.makedirs('nerf_dataset', exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        self.get_logger().info(
            'NeRF saver started'
        )

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        filename = (
            f'nerf_dataset/frame_{self.index:05d}.png'
        )

        cv2.imwrite(filename, frame)

        self.index += 1


def main(args=None):

    rclpy.init(args=args)

    node = NerfSaver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
