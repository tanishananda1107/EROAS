#!/usr/bin/env python3
# ROS 2 port of sonar_reconstruction_global_old.py
# Uses independent subscribers (no time sync) — mirrors the original design.
import rclpy
from rclpy.node import Node
import math
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


class SonarReconstructionGlobalOld(Node):
    def __init__(self):
        super().__init__('sonar_intensity_publisher')

        # Independent subscribers (original design had no sync)
        self.create_subscription(
            Float64, '/rexrov2/sonar_joint_position_controller/command',
            self.pivot_callback, 10)
        self.create_subscription(
            Odometry, '/rexrov2/pose_gt',
            self.height_callback, 10)
        self.create_subscription(
            ProjectedSonarImage,
            '/rexrov2/blueview_p900/sonar_image_raw',
            self.sonar_callback, 10)

        # Publishers
        self.pc_pub      = self.create_publisher(
            PointCloud2, '/rexrov2/point_cloud',         10)
        self.hist_pub    = self.create_publisher(
            PointCloud2, '/rexrov2/point_cloud_history', 10)

        # State
        self.pivot_angle         = 0.0
        self.x_position          = 0.0
        self.y_position          = 0.0
        self.z_position          = 0.0
        self.quat                = None
        self.translation_vector  = [0.0, 0.0, 0.0]
        self.point_cloud_history = []

    # ------------------------------------------------------------------ #
    def pivot_callback(self, msg):
        self.pivot_angle = msg.data

    def height_callback(self, msg):
        self.x_position = msg.pose.pose.position.x
        self.y_position = msg.pose.pose.position.y
        self.z_position = msg.pose.pose.position.z
        self.quat       = msg.pose.pose.orientation
        self.translation_vector = [
            self.x_position, self.y_position, self.z_position]

    def sonar_callback(self, msg):
        if self.quat is None:
            return          # pose not yet received

        data_array  = np.frombuffer(msg.image.data, dtype=np.uint8)
        contour_pts = self.find_contour_points(data_array)

        min_range = self.find_min(contour_pts)
        if min_range is not None:
            self.get_logger().info(
                f'min_range: {min_range:.4f}  pivot: {self.pivot_angle:.4f}')

        local_pts  = [self.polar_to_cartesian(i, j, 512, 598)
                      for i, j in contour_pts]
        R          = self._quaternion_to_rotation_matrix(self.quat)
        t          = np.array(self.translation_vector)
        global_pts = np.array([R @ np.array(p) + t for p in local_pts])

        # Current-frame cloud
        header          = msg.header
        header.frame_id = 'rexrov2/base_link'
        self.pc_pub.publish(create_cloud_xyz32(header, global_pts))

        # History cloud
        self.point_cloud_history.extend(global_pts.tolist())
        if self.point_cloud_history:
            hist_hdr          = msg.header
            hist_hdr.frame_id = 'world'
            self.hist_pub.publish(
                create_cloud_xyz32(hist_hdr, self.point_cloud_history))

    # ------------------------------------------------------------------ #
    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """Sonar polar → robot-local Cartesian (original global_old mapping)."""
        angle_rad = i * 0.0030739647336304188 + math.pi / 4
        distance  = j * 15.0 / max_bins

        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad) * math.cos(self.pivot_angle)
        z = distance * math.sin(angle_rad) * math.sin(self.pivot_angle)
        return x, y, z

    # ------------------------------------------------------------------ #
    def _quaternion_to_rotation_matrix(self, q):
        # NOTE: original used (x,y,z,w) order — preserved here.
        w, x, y, z = q.w, q.x, q.y, q.z
        return np.array([
            [1 - 2*(y**2 + z**2),  2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),        1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),        2*(y*z + x*w),   1 - 2*(x**2 + y**2)]])

    # ------------------------------------------------------------------ #
    def find_min(self, points):
        if not points:
            return None
        return min(j for _, j in points)

    def find_contour_points(self, data):
        no_of_beams  = 512
        range_bin    = 590
        range_window = 3

        contour_points = []
        for i in range(5, no_of_beams, 1):
            for j in range(150, range_bin - 90, 4):
                window = []
                for k in range(range_window):
                    window.append(data[i + (j + k) * no_of_beams])
                if np.mean(window) > 15:
                    contour_points.append((i, j))
                    break
        return contour_points


# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    node = SonarReconstructionGlobalOld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
