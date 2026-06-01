#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys


class NerfSaver(Node):
    def __init__(self, image_prefix):
        super().__init__('nerf_saver')

        self.bridge = CvBridge()
        self.image_directory = '/home/user/sonar_images'
        self.image_prefix = image_prefix
        self.image_counter = 0
        self.cv_cam_image = None
        self.cam_image_filename = ''

        os.makedirs(self.image_directory, exist_ok=True)

        self.cam_image_sub = self.create_subscription(
            Image,
            '/rexrov2/rexrov2/camera/image_raw',
            self.camera_callback,
            10
        )

        # Save every 5 seconds
        self.timer = self.create_timer(5.0, self.save_images)
        self.get_logger().info("NerfSaver node started. Saving every 5 seconds.")

    def camera_callback(self, data):
        try:
            self.cv_cam_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.cam_image_filename = os.path.join(
                self.image_directory,
                f'camera_{self.image_prefix}_{self.image_counter}.png'
            )
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def save_images(self):
        if self.cv_cam_image is not None:
            try:
                cv2.imwrite(self.cam_image_filename, self.cv_cam_image)
                self.get_logger().info(f"Saved: {self.cam_image_filename}")
                self.image_counter += 1
                self.cv_cam_image = None
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")
        else:
            self.get_logger().warn("No camera image received yet, skipping save.")


def main(args=None):
    rclpy.init(args=args)

    image_prefix = 'image'
    for arg in sys.argv[1:]:
        if arg.startswith('--image_prefix='):
            image_prefix = arg.split('=')[1]

    node = NerfSaver(image_prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
