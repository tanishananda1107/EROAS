#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys


class ImageSaver(Node):
    def __init__(self, image_prefix):
        super().__init__('image_saver')

        self.bridge = CvBridge()
        self.image_directory = '/home/user/sonar_images'
        self.image_prefix = image_prefix
        self.image_counter = 0
        self.cam_image_filename = ''
        self.cv_cam_image = None

        os.makedirs(self.image_directory, exist_ok=True)

        self.cam_image_sub = self.create_subscription(
            Image,
            '/rexrov2/rexrov2/camera/image_raw',
            self.camera_callback,
            10
        )
        self.sonar_image_sub = self.create_subscription(
            Image,
            '/rexrov2/blueview_p900/sonar_image',
            self.callback,
            10
        )
        self.get_logger().info("ImageSaver node started.")

    def camera_callback(self, data):
        try:
            self.cv_cam_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.cam_image_filename = os.path.join(
                self.image_directory,
                f'camera_{self.image_prefix}.png'
            )
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            sonar_filename = os.path.join(
                self.image_directory,
                f'sonar_{self.image_prefix}.png'
            )

            cv2.imwrite(sonar_filename, cv_image)

            if self.cv_cam_image is not None and self.cam_image_filename:
                cv2.imwrite(self.cam_image_filename, self.cv_cam_image)

            self.get_logger().info(f"Both images saved: {sonar_filename}")
            self.image_counter += 1

            if self.image_counter > 0:
                self.get_logger().info("Shutting down after saving images.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing sonar image: {e}")


def main(args=None):
    rclpy.init(args=args)

    image_prefix = 'image'
    for arg in sys.argv[1:]:
        if arg.startswith('--image_prefix='):
            image_prefix = arg.split('=')[1]

    node = ImageSaver(image_prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
