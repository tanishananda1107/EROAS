#!/usr/bin/env python3
"""
contour_heading.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - rospy → rclpy / Node
  - rospy.Rate(10) → no manual spin loop; rospy.spin() → rclpy.spin()
  - rospy.get_time() → self.get_clock().now().nanoseconds / 1e9
  - Logging → self.get_logger().*
  - marine_acoustic_msgs must be available as a ROS 2 package
  - cv_bridge is available for ROS 2 (pip install cv-bridge or ros-<distro>-cv-bridge)
"""

import math

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
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
        self.slope_publisher = self.create_publisher(Float32, '/rexrov2/dominant_slope', 10)
        self.avg_positive_slope_publisher = self.create_publisher(
            Float32, '/rexrov2/avg_right_slope', 10)
        self.avg_negative_slope_publisher = self.create_publisher(
            Float32, '/rexrov2/avg_left_slope', 10)

        self.bridge = CvBridge()
        self.slopes = []

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
    def print_polynomial_coefficients(self, coeffs):
        self.get_logger().info(
            f'Polynomial: y = {coeffs[0]:.4f}x^2 + {coeffs[1]:.4f}x + {coeffs[2]:.4f}')

    # ------------------------------------------------------------------
    def compute_polynomial_derivative(self, coeffs):
        a, b, _ = coeffs
        return [2 * a, b]

    # ------------------------------------------------------------------
    def find_and_plot_curve(self, x_coords, y_coords):
        sorted_points = sorted(zip(x_coords, y_coords))
        x_coords, y_coords = zip(*sorted_points)

        if any(x2 <= x1 for x1, x2 in zip(x_coords, x_coords[1:])):
            self.get_logger().warn('x_coords are not strictly increasing. Removing duplicates.')
            unique_points = []
            for x, y in zip(x_coords, y_coords):
                if not unique_points or unique_points[-1][0] != x:
                    unique_points.append((x, y))
            x_coords, y_coords = zip(*unique_points)

        coeffs = np.polyfit(x_coords, y_coords, 2)
        self.print_polynomial_coefficients(coeffs)
        derivative_coeffs = self.compute_polynomial_derivative(coeffs)

        x_fit = np.linspace(min(x_coords), max(x_coords), num=500)
        y_fit = np.polyval(coeffs, x_fit)

        img_width = 1000
        img_height = 1000
        display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        for i in range(len(x_fit) - 1):
            x1 = int(500 + x_fit[i] * 500 / 15)
            y1 = int(1000 - y_fit[i] * 1000 / 15)
            x2 = int(500 + x_fit[i + 1] * 500 / 15)
            y2 = int(1000 - y_fit[i + 1] * 1000 / 15)
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.slopes = []
        positive_tangent_slopes = []
        negative_tangent_slopes = []

        for x, y in zip(x_coords, y_coords):
            tangent_slope = np.polyval(derivative_coeffs, x)
            if x > 0:
                positive_tangent_slopes.append(tangent_slope)
            elif x < 0:
                negative_tangent_slopes.append(tangent_slope)

        self.slopes = positive_tangent_slopes + negative_tangent_slopes

        avg_positive_slope = float(np.mean(positive_tangent_slopes)) if positive_tangent_slopes else 0.0
        avg_negative_slope = float(np.mean(negative_tangent_slopes)) if negative_tangent_slopes else 0.0

        dominant_slope = (avg_positive_slope
                          if abs(avg_positive_slope) > abs(avg_negative_slope)
                          else avg_negative_slope)

        self.get_logger().info(f'Dominant Slope: {dominant_slope:.4f}')

        ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        self.slope_publisher.publish(Float32(data=dominant_slope))
        self.avg_positive_slope_publisher.publish(Float32(data=avg_positive_slope))
        self.avg_negative_slope_publisher.publish(Float32(data=avg_negative_slope))
        self.publisher.publish(ros_image)

    # ------------------------------------------------------------------
    def sonar_callback(self, msg: ProjectedSonarImage):
        # ROS 2 clock returns nanoseconds
        start_ns = self.get_clock().now().nanoseconds
        start_time = start_ns / 1e9

        data_array = np.frombuffer(msg.image.data, dtype=np.uint8)
        points_contour = self.find_contour_points(data_array)
        points_cartesian = [
            self.polar_to_cartesian(i, j, 512, 598) for (i, j) in points_contour
        ]

        if points_cartesian:
            x_coords, y_coords = zip(*points_cartesian)
            self.find_and_plot_curve(x_coords, y_coords)

        end_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Processing time: {end_time - start_time:.4f} s')

    # ------------------------------------------------------------------
    def find_contour_points(self, data):
        no_of_beams = 512
        range_bin = 590
        range_window = 3
        contour_points = []

        for i in range(10, no_of_beams, 10):
            for j in range(20, range_bin - 90, 7):
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
