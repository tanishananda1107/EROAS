#!/usr/bin/env python3
"""
grid_detection.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - rospy → rclpy / Node
  - rospy.logerr → self.get_logger().error
  - rospy.loginfo → self.get_logger().info
  - Spin / shutdown unchanged in structure, adapted to rclpy
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        self.sonar_max_range = 10.0    # m
        self.sonar_interest_range = 7.0  # m

        self.subscriber = self.create_subscription(
            Image,
            '/rexrov2/blueview_p900/sonar_image',
            self.callback,
            1)

        self.publisher = self.create_publisher(Image, '/rexrov2/detected_objects', 10)

        self.get_logger().info('YOLO Detector Node started')

    # ------------------------------------------------------------------
    def callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

            mask = mask1 + mask2

            blurred = cv2.GaussianBlur(mask, (5, 5), 0)
            contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            min_contour_area = 500
            filtered_contours = [c for c in contours if cv2.contourArea(c) > min_contour_area]
            hulls = [cv2.convexHull(c) for c in filtered_contours]
            cv2.drawContours(cv_image, hulls, -1, (0, 255, 0), 2)

            height, width, _ = cv_image.shape
            center_x = width // 2
            center_y = height - 1

            meters_per_pixel = self.sonar_max_range / height
            lines_of_interest = [
                (self.sonar_interest_range - i) / meters_per_pixel for i in range(5, 0, -1)
            ]

            for line in lines_of_interest:
                y = int(line)
                cv2.line(cv_image, (0, y), (width, y), (255, 255, 255), 2)

            # Central vertical dotted line (blue)
            for i in range(0, height, 10):
                cv2.line(cv_image,
                         (center_x, center_y - i),
                         (center_x, center_y - i - 5),
                         (255, 0, 0), 2)

            # Angled sector lines
            angles = [-25, -20, -15, -10, -5, 5, 10, 15, 20, 25]
            for angle in angles:
                radians = np.deg2rad(angle)
                obstacle_within_range = False

                for i in range(0, int(self.sonar_interest_range / meters_per_pixel), 10):
                    start_y = center_y - i
                    end_y = start_y - 5
                    start_x = int(center_x + i * np.tan(radians))
                    end_x = int(center_x + (i + 5) * np.tan(radians))
                    if np.any(mask[start_y:end_y, start_x:end_x] > 0):
                        obstacle_within_range = True
                        break

                if not obstacle_within_range:
                    polygon = np.array([
                        [center_x, center_y],
                        [int(center_x + self.sonar_interest_range / meters_per_pixel * np.tan(radians)),
                         int(center_y - self.sonar_interest_range / meters_per_pixel)],
                        [int(center_x + self.sonar_interest_range / meters_per_pixel * np.tan(radians + np.deg2rad(5))),
                         int(center_y - self.sonar_interest_range / meters_per_pixel)],
                        [center_x, center_y],
                    ])
                    cv2.fillPoly(cv_image, [polygon], (0, 255, 0, 50))

                for i in range(0, height, 10):
                    start_y = center_y - i
                    end_y = start_y - 5
                    start_x = int(center_x + i * np.tan(radians))
                    end_x = int(center_x + (i + 5) * np.tan(radians))
                    cv2.line(cv_image, (start_x, start_y), (end_x, end_y), (255, 255, 255), 2)

            image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f'Error in YOLODetector callback: {e}')


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down YOLO Detector Node')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
