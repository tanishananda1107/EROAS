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
        intensity = np.zeros(
            (self.NUM_RANGES, self.NUM_BEAMS),
            dtype=np.uint8,
        )

        for point in pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if not (
                math.isfinite(x)
                and math.isfinite(y)
                and math.isfinite(z)
            ):
                continue
            if x <= 0.1:
                continue

            angle = math.atan2(y, x)
            if not -self.H_FOV_HALF <= angle <= self.H_FOV_HALF:
                continue

            beam = int(
                (angle + self.H_FOV_HALF)
                / self.H_FOV
                * (self.NUM_BEAMS - 1)
            )
            beam = max(0, min(self.NUM_BEAMS - 1, beam))

            distance = math.sqrt(x * x + y * y)
            if distance >= self.MAX_RANGE:
                continue
            range_bin = int(distance / self.RANGE_RES)
            range_bin = max(0, min(self.NUM_RANGES - 1, range_bin))
            first_bin = max(0, range_bin - 2)
            last_bin = min(self.NUM_RANGES, range_bin + 3)
            intensity[first_bin:last_bin, beam] = 255

        out = ProjectedSonarImage()
        out.header = msg.header
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
