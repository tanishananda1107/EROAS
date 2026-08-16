#!/usr/bin/env python3
"""
Convert Gazebo GpuLidar PointCloud2 data to ProjectedSonarImage.

The input point cloud is expressed in the sonar sensor frame:
  x = forward, y = lateral, z = vertical.
"""

import math

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Vector3
from marine_acoustic_msgs.msg import ProjectedSonarImage
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


class SonarFrameConverter(Node):
    NUM_BEAMS = 512
    NUM_RANGES = 350
    MAX_RANGE = 15.0
    RANGE_RES = MAX_RANGE / NUM_RANGES
    H_FOV = math.pi / 2.0
    H_FOV_HALF = H_FOV / 2.0

    def __init__(self):
        super().__init__('sonar_frame_converter')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/rexrov2/blueview_p900_point_cloud',
            self.pc_callback,
            sensor_qos,
        )
        self.pub = self.create_publisher(
            ProjectedSonarImage,
            '/rexrov2/blueview_p900/sonar_image_raw',
            10,
        )
        self.get_logger().info(
            'SonarFrameConverter ready. '
            'Converting PointCloud2 -> ProjectedSonarImage'
        )

    def pc_callback(self, msg):
        # The GPU lidar backing this topic publishes horizontal_samples(512)
        # x vertical_samples(fidelity, default 500) = up to ~256k points per
        # scan. A pure-Python per-point loop over that many points regularly
        # took long enough (several hundred ms to >1s) to blow past
        # only_gap.py's 1.0s sonar_timeout, making raw sonar look
        # intermittently or even sustainedly stale near large obstacles
        # (more of the FOV returns valid close-range hits there, adding even
        # more points to iterate) and silently falling back to legacy
        # pose-only navigation. Vectorized with numpy instead.
        intensity = np.zeros(
            (self.NUM_RANGES, self.NUM_BEAMS),
            dtype=np.uint8,
        )

        points = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if points.size:
            x, y, z = points[:, 0], points[:, 1], points[:, 2]
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (x > 0.1)
            x, y, z = x[valid], y[valid], z[valid]

            angle = np.arctan2(y, x)
            in_fov = (angle >= -self.H_FOV_HALF) & (angle <= self.H_FOV_HALF)
            x, y, z, angle = x[in_fov], y[in_fov], z[in_fov], angle[in_fov]

            distance = np.sqrt(x * x + y * y + z * z)
            in_range = distance < self.MAX_RANGE
            angle, distance = angle[in_range], distance[in_range]

            if angle.size:
                beam = (
                    (angle + self.H_FOV_HALF) / self.H_FOV
                    * (self.NUM_BEAMS - 1)
                ).astype(np.int64)
                beam = np.clip(beam, 0, self.NUM_BEAMS - 1)

                range_bin = (distance / self.RANGE_RES).astype(np.int64)
                range_bin = np.clip(range_bin, 0, self.NUM_RANGES - 1)

                offsets = np.arange(-2, 3)
                bins = np.clip(
                    range_bin[:, None] + offsets[None, :],
                    0, self.NUM_RANGES - 1).ravel()
                beams_repeated = np.repeat(beam, offsets.size)
                intensity[bins, beams_repeated] = 255

        out = ProjectedSonarImage()
        out.header = msg.header
        # The incoming point cloud's stamp comes from Gazebo's simulation
        # clock; consumers of this topic run with use_sim_time:=False, so a
        # copied sim-time stamp always looks catastrophically stale against
        # wall time and gets silently dropped. Stamp with wall time instead.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = (
            'rexrov2/rexrov2/sonar_link/blueview_p900_sensor'
        )
        out.beam_directions = [
            Vector3(x=math.cos(angle), y=math.sin(angle), z=0.0)
            for angle in np.linspace(
                -self.H_FOV_HALF,
                self.H_FOV_HALF,
                self.NUM_BEAMS,
            )
        ]
        out.ranges = (
            (np.arange(self.NUM_RANGES, dtype=np.float32) + 0.5)
            * self.RANGE_RES
        ).tolist()
        out.image.is_bigendian = False
        out.image.dtype = out.image.DTYPE_UINT8
        out.image.beam_count = self.NUM_BEAMS
        out.image.data = intensity.reshape(-1).tolist()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SonarFrameConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
