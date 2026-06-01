#!/usr/bin/env python3
"""
cubic_spline_contour.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
"""

import math

import cv2
import numpy as np
import scipy.interpolate as spi
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
    def print_spline_equations(self, cs):
        c0, c1, c2, c3 = cs.c
        num_segments = len(c0)
        for idx in range(num_segments):
            a = c3[idx]; b = c2[idx]; c = c1[idx]; d = c0[idx]
            self.get_logger().info(
                f'Segment {idx + 1}: S(x) = {a:.4f}(x-x{idx})^3 + '
                f'{b:.4f}(x-x{idx})^2 + {c:.4f}(x-x{idx}) + {d:.4f}')

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

        cs = spi.CubicSpline(x_coords, y_coords, bc_type='natural')
        self.print_spline_equations(cs)

        x_fit = np.linspace(min(x_coords), max(x_coords), num=500)
        y_fit = cs(x_fit)

        img_width = 1000
        img_height = 1000
        display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        for i in range(len(x_fit) - 1):
            x1 = int(500 + x_fit[i] * 500 / 15)
            y1 = int(1000 - y_fit[i] * 1000 / 15)
            x2 = int(500 + x_fit[i + 1] * 500 / 15)
            y2 = int(1000 - y_fit[i + 1] * 1000 / 15)
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        self.publisher.publish(ros_image)
        return cs

    # ------------------------------------------------------------------
    def sonar_callback(self, msg: ProjectedSonarImage):
        start_time = self.get_clock().now().nanoseconds / 1e9

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
