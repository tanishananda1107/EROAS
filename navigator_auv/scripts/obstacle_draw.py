#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
import csv
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py.point_cloud2 import read_points
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        pc_sub  = Subscriber(self, PointCloud2, '/rexrov2/point_cloud')
        pose_sub = Subscriber(self, Odometry,    '/rexrov2/pose_gt')

        self.sync = ApproximateTimeSynchronizer([pc_sub, pose_sub], queue_size=10, slop=1.0)
        self.sync.registerCallback(self.synchronized_callback)

        self.csv_file = '/home/user/data_log.csv'
        self.csv_fh   = open(self.csv_file, mode='a', newline='')
        self.writer   = csv.writer(self.csv_fh)
        if self._is_empty(self.csv_file):
            self.writer.writerow(['Timestamp','Pose_X','Pose_Y','Pose_Z','Point_X','Point_Y','Point_Z'])

    def _is_empty(self, path):
        with open(path, 'r') as f:
            return f.read(1) == ''

    def synchronized_callback(self, pc_msg, pose_msg):
        ts   = self.get_clock().now().nanoseconds / 1e9
        px   = pose_msg.pose.pose.position.x
        py   = pose_msg.pose.pose.position.y
        pz   = pose_msg.pose.pose.position.z
        for pt in read_points(pc_msg, field_names=('x','y','z'), skip_nans=True):
            self.writer.writerow([ts, px, py, pz, pt[0], pt[1], pt[2]])

    def destroy_node(self):
        if hasattr(self, 'csv_fh'):
            self.csv_fh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
