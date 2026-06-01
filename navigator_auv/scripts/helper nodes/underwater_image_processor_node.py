#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class UnderwaterImageProcessor(Node):
    def __init__(self):
        super().__init__('underwater_image_processor')

        texture_image_path = self.declare_parameter(
            'texture_image_path', '/home/user/P1.jpg'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()
        self.texture_image = cv2.imread(texture_image_path, cv2.IMREAD_COLOR)

        if self.texture_image is None:
            self.get_logger().warn(f"Could not load texture image from: {texture_image_path}")

        self.subscription = self.create_subscription(
            Image,
            '/rexrov2/rexrov2/camera/image_raw',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/rexrov2/underwater_image_1', 10)
        self.get_logger().info("UnderwaterImageProcessor node started.")

    def add_blue_green_tint(self, image):
        tint = np.full_like(image, (50, 100, 150), dtype=np.uint8)  # BGR
        return cv2.addWeighted(image, 0.7, tint, 0.1, 0)

    def blend_with_texture(self, image):
        if self.texture_image is None:
            return image
        texture_resized = cv2.resize(self.texture_image, (image.shape[1], image.shape[0]))
        return cv2.addWeighted(image, 0.7, texture_resized, 0.3, 0)

    def adjust_brightness_contrast(self, image, brightness=0, contrast=30):
        img = np.int16(image)
        img = img * (contrast / 127 + 1) - contrast + brightness
        img = np.clip(img, 0, 255)
        return np.uint8(img)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            tinted = self.add_blue_green_tint(cv_image)
            blended = self.blend_with_texture(tinted)
            underwater = self.adjust_brightness_contrast(blended)

            image_msg = self.bridge.cv2_to_imgmsg(underwater, "bgr8")
            image_msg.header = data.header
            self.publisher.publish(image_msg)

            self.get_logger().info(
                f"Published underwater image — width: {image_msg.width}, height: {image_msg.height}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in UnderwaterImageProcessor callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
