#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os


class CameraImageSaver(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.bridge = CvBridge()
        os.makedirs('/tmp/allen', exist_ok=True)

        image_topic = "/rexrov2/rexrov2/camera/image_raw"
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {image_topic}")

    def image_callback(self, msg):
        self.get_logger().info("Received an image!")
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        filepath = f'/tmp/allen/{timestamp}.jpeg'
        cv2.imwrite(filepath, cv2_img)
        self.get_logger().info(f"Saved: {filepath}")


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
