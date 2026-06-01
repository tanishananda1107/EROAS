#!/usr/bin/env python3
# ROS 2 port of sonar_reconstruction_old.py
# Uses ApproximateTimeSynchronizer (same as sonar_reconstruction.py)
# but keeps the original find_contour_points loop structure and
# publishes a history cloud.
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import PointCloud2, JointState
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from message_filters import Subscriber, ApproximateTimeSynchronizer


class SonarReconstructionOld(Node):
    def __init__(self):
        super().__init__('sonar_intensity_publisher')

        # Synchronised subscribers
        js_sub    = Subscriber(self, JointState,
                               '/rexrov2/joint_states')
        pose_sub  = Subscriber(self, Odometry,
                               '/rexrov2/pose_gt')
        sonar_sub = Subscriber(self, ProjectedSonarImage,
                               '/rexrov2/blueview_p900/sonar_image_raw')

        self.sync = ApproximateTimeSynchronizer(
            [js_sub, pose_sub, sonar_sub], queue_size=100, slop=1.0)
        self.sync.registerCallback(self.synchronized_callback)

        # Publishers
        self.pc_pub   = self.create_publisher(
            PointCloud2, '/rexrov2/point_cloud',         10)
        self.hist_pub = self.create_publisher(
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
    def synchronized_callback(self, js_msg, height_msg, sonar_msg):
        try:
            # Joint state
            if 'rexrov2/sonar_vertical_joint' in js_msg.name:
                idx = js_msg.name.index('rexrov2/sonar_vertical_joint')
                self.pivot_angle = js_msg.position[idx]

            # Pose
            self.x_position = height_msg.pose.pose.position.x
            self.y_position = height_msg.pose.pose.position.y
            self.z_position = height_msg.pose.pose.position.z
            self.quat       = height_msg.pose.pose.orientation
            self.translation_vector = [
                self.x_position, self.y_position, self.z_position]

            self.process_sonar_data(sonar_msg)
        except Exception as e:
            self.get_logger().error(f'synchronized_callback error: {e}')

    # ------------------------------------------------------------------ #
    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """Sonar polar → robot-local Cartesian, accounting for pivot tilt."""
        angle_rad = i * 0.0030739647336304188 + math.pi / 4
        distance  = j * 15.0 / max_bins

        x_local =  distance * math.cos(angle_rad)
        y_local =  distance * math.sin(angle_rad) * math.cos(self.pivot_angle)
        z_local =  distance * math.sin(angle_rad) * math.sin(self.pivot_angle)

        # (forward, -lateral, vertical)
        return y_local, -x_local, z_local

    # ------------------------------------------------------------------ #
    def _normalize_quaternion(self, q):
        n = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        return q.w / n, q.x / n, q.y / n, q.z / n

    def _quaternion_to_rotation_matrix(self, q):
        w, x, y, z = self._normalize_quaternion(q)
        return np.array([
            [1 - 2*(y**2 + z**2),  2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),        1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),        2*(y*z + x*w),   1 - 2*(x**2 + y**2)]])

    def _local_to_global(self, local_points, R, t):
        return np.array([R @ np.array(p) + t for p in local_points])

    # ------------------------------------------------------------------ #
    def process_sonar_data(self, msg):
        data_array  = np.frombuffer(msg.image.data, dtype=np.uint8)
        contour_pts = self.find_contour_points(data_array)

        min_range = self.find_min(contour_pts)
        if min_range is not None:
            self.get_logger().info(
                f'min_range: {min_range:.4f}  pivot: {self.pivot_angle:.4f}')

        if not contour_pts:
            return

        local_pts  = [self.polar_to_cartesian(i, j, 512, 598)
                      for i, j in contour_pts]
        R          = self._quaternion_to_rotation_matrix(self.quat)
        t          = np.array(self.translation_vector)
        global_pts = self._local_to_global(local_pts, R, t)

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
    def find_min(self, points):
        if not points:
            return None
        return min(j for _, j in points)

    def find_contour_points(self, data):
        """
        Original loop structure: iterates every 3rd beam and range bin,
        finds the first range bin above threshold per beam.
        """
        no_of_beams  = 512
        range_bin    = 598
        range_window = 3
        threshold    = 15

        data   = np.array(data).reshape((range_bin, no_of_beams))
        points = []

        for i in range(5, no_of_beams, 3):
            for j in range(100, range_bin - 40, 3):
                if np.mean(data[j:j + range_window, i]) > threshold:
                    points.append((i, j))
                    break   # first hit per beam only

        return points


# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    node = SonarReconstructionOld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
