#!/usr/bin/env python3
"""
contour_heading_old.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
"""

import math

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarIntensityPublisher(Node):
    def __init__(self):
        super().__init__('sonar_intensity_publisher')

        self.sonar_sub = self.create_subscription(
            ProjectedSonarImage,
            '/rexrov2/blueview_p900/sonar_image_raw',
            self.sonar_callback,
            10)

        self.publisher = self.create_publisher(Image, '/rexrov2/detected_objects', 10)
        self.bridge = CvBridge()

        self.get_logger().info('Sonar Intensity Publisher Node Running')

    # ------------------------------------------------------------------
    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi / 4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    # ------------------------------------------------------------------
    def sonar_callback(self, msg: ProjectedSonarImage):
        start_time = self.get_clock().now().nanoseconds / 1e9

        data_array = np.frombuffer(msg.image.data, dtype=np.uint8)
        points_contour = self.find_contour_points(data_array)

        img_width = 1000
        img_height = 1000
        display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        for (i, j) in points_contour:
            x, y = self.polar_to_cartesian(i, j, 512, 598)
            xi = int(500 + x * 500 / 15)
            yi = int(1000 - y * 1000 / 15)
            cv2.circle(display_image, (xi, yi), 5, (0, 0, 255), -1)

        ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        self.publisher.publish(ros_image)

        end_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Processing time: {end_time - start_time:.4f} s')

    # ------------------------------------------------------------------
    def find_contour_points(self, data):
        no_of_beams = 512
        range_bin = 590
        range_window = 3
        contour_points = []

        for i in range(10, no_of_beams, 10):
            for j in range(250, range_bin - 90, 7):
                window_intensities = []
                for k in range(range_window):
                    intensity1 = data[i + (j + k) * no_of_beams]
                    window_intensities.append(intensity1)
                if np.mean(window_intensities) > 15:
                    contour_points.append((i, j))
                    break

        return contour_points


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = SonarIntensityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
