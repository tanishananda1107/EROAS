#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()
        self.distance_threshold = 7

        self.subscription = self.create_subscription(
            Image,
            '/rexrov2/underwater_image',
            self.callback,
            1
        )
        self.publisher = self.create_publisher(Image, '/rexrov2/detected_objects', 10)
        self.get_logger().info("YOLODetector (color-based) node started.")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])

            mask = cv2.inRange(hsv, lower_red1, upper_red1) + \
                   cv2.inRange(hsv, lower_red2, upper_red2)

            blurred = cv2.GaussianBlur(mask, (5, 5), 0)

            contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered = [c for c in contours if cv2.contourArea(c) > 1500]
            hulls = [cv2.convexHull(c) for c in filtered]
            cv2.drawContours(cv_image, hulls, -1, (0, 255, 0), 2)

            height, width, _ = cv_image.shape
            center_x = width // 2
            center_y = height - 1

            # Blue dotted vertical line
            for i in range(0, height, 10):
                cv2.line(cv_image,
                         (center_x, center_y - i),
                         (center_x, center_y - i - 5),
                         (255, 0, 0), 2)

            # White dotted angled lines
            angles = [-25, -20, -15, -10, -5, 5, 10, 15, 20, 25]
            for angle in angles:
                radians = np.deg2rad(angle)
                for i in range(0, height, 10):
                    start_y = center_y - i
                    end_y = start_y - 5
                    start_x = int(center_x + i * np.tan(radians))
                    end_x = int(center_x + (i + 5) * np.tan(radians))
                    cv2.line(cv_image, (start_x, start_y), (end_x, end_y), (255, 255, 255), 2)

            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header = data.header
            self.publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error in YOLODetector callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
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
