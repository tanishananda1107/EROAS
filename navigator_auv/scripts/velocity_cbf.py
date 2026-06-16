#!/usr/bin/env python3
"""EROAS CBF Safety Filter + Kinematic Driver for Gazebo Harmonic.

Since the gz-sim VelocityControl plugin does not load from URDF spawned models,
this node directly moves the rexrov2 model by computing new poses kinematically
from velocity commands. This matches hover_mode behavior (no gravity/drag).

Architecture:
  eroas_planner.py → /rexrov2/cmd_vel_1 → [THIS NODE] → moves model via gz set_pose
"""
import math
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations as tft


class ObstacleAvoidanceNode(Node):
    """CBF safety filter + kinematic model driver."""

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters
        self.declare_parameter('R_o', 4.0)
        self.declare_parameter('kappa', 0.09)
        self.declare_parameter('radius', 15.0)
        self.declare_parameter('target_depth', -20.0)
        self.declare_parameter('depth_kp', 0.5)
        self.declare_parameter('max_vertical_speed', 0.85)
        self.declare_parameter('control_rate', 20.0)

        self.R_o = float(self.get_parameter('R_o').value)
        self.kappa = float(self.get_parameter('kappa').value)
        self.radius = float(self.get_parameter('radius').value)
        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / self.control_rate

        # State
        self.position = np.array([0.0, 0.0, -20.0])
        self.yaw = 0.0
        self.quaternion = [0.0, 0.0, 0.0, 1.0]
        self.pose_received = False
        self.filtered_points = np.empty((0, 3))
        self.closest_point = None
        self.closest_obstacle_distance = float('inf')
        self.current_h = float('inf')
        self.v_alg = Twist()

        # Subscribers
        self.create_subscription(Twist, '/rexrov2/cmd_vel_1', self._vel_cb, 10)
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self._pose_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rexrov2/blueview_p900_point_cloud',
                                 self._pc_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/rexrov2/sonar/moving', self._sonar_cb, 10)

        # Publishers (for monitoring)
        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)
        self.h_pub = self.create_publisher(Float64, '/rexrov2/current_h', 10)

        # Control timer — moves the model each tick
        self.create_timer(self.dt, self._control_tick)

        self.get_logger().info(f'[CBF] Kinematic driver started at {self.control_rate} Hz')

    def _vel_cb(self, msg):
        self.v_alg = msg

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.position = np.array([p.x, p.y, p.z])
        self.quaternion = [o.x, o.y, o.z, o.w]
        self.yaw = tft.euler_from_quaternion(self.quaternion)[2]
        self.pose_received = True

        # Find closest obstacle
        if len(self.filtered_points) > 0:
            dists = np.linalg.norm(self.filtered_points - self.position, axis=1)
            idx = np.argmin(dists)
            if dists[idx] <= self.radius:
                self.closest_point = self.filtered_points[idx]
                self.closest_obstacle_distance = dists[idx]
                self.current_h = dists[idx]**2 - self.R_o**2
            else:
                self.closest_point = None
                self.closest_obstacle_distance = float('inf')
                self.current_h = float('inf')
        else:
            self.closest_point = None
            self.closest_obstacle_distance = float('inf')
            self.current_h = float('inf')

    def _pc_cb(self, msg):
        if not self.pose_received:
            return
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                        skip_nans=True, reshape_organized_cloud=False)
            new_pts = np.round(np.asarray(pts, dtype=float).reshape((-1, 3)))
        except Exception:
            return
        if new_pts.size == 0:
            return

        if len(self.filtered_points) > 0:
            all_pts = np.vstack((self.filtered_points, new_pts))
        else:
            all_pts = new_pts

        all_pts = np.unique(all_pts, axis=0)
        dists = np.linalg.norm(all_pts - self.position, axis=1)
        self.filtered_points = all_pts[dists <= self.radius]

    def _sonar_cb(self, msg):
        s = msg.data
        if s == 0 or s == 2:
            self.R_o = 4.0
        elif s == 1:
            self.R_o = 2.0

    def _control_tick(self):
        """Apply CBF filter to planner command, then move the model."""
        if not self.pose_received:
            return

        v = self.v_alg
        # Apply CBF projection
        safe_vx, safe_vy = self._cbf_filter_xy(v.linear.x, v.linear.y)

        # Depth hold + vertical command passthrough
        if abs(v.linear.z) > 0.01:
            safe_vz = v.linear.z  # planner commands vertical (escape)
        else:
            error = self.target_depth - self.position[2]
            safe_vz = float(np.clip(self.depth_kp * error,
                                    -self.max_vertical_speed, self.max_vertical_speed))

        # Yaw rate passed through unchanged (paper design)
        wz = v.angular.z

        # Publish for monitoring
        tw = Twist()
        tw.linear.x = safe_vx
        tw.linear.y = safe_vy
        tw.linear.z = safe_vz
        tw.angular.z = wz
        self.cmd_vel_pub.publish(tw)
        self.h_pub.publish(Float64(data=float(self.current_h)))

        # Kinematic integration: compute new pose
        new_yaw = self.yaw + wz * self.dt
        # Velocity in body frame → world frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        vx_world = safe_vx * cos_y - safe_vy * sin_y
        vy_world = safe_vx * sin_y + safe_vy * cos_y
        vz_world = safe_vz

        new_x = self.position[0] + vx_world * self.dt
        new_y = self.position[1] + vy_world * self.dt
        new_z = self.position[2] + vz_world * self.dt

        # Set pose in Gazebo
        self._set_gz_pose(new_x, new_y, new_z, new_yaw)

    def _cbf_filter_xy(self, vx, vy):
        """Single half-plane CBF projection (paper Eq. 33-34)."""
        if self.closest_point is None or self.current_h == float('inf'):
            return vx, vy

        # Transform to global
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        gx = vx * cos_y - vy * sin_y
        gy = vx * sin_y + vy * cos_y

        # Gradient
        grad = np.array([
            2.0 * (self.position[0] - self.closest_point[0]),
            2.0 * (self.position[1] - self.closest_point[1])
        ])
        margin = self.kappa * (self.current_h - 0.5)
        desired = np.array([gx, gy])
        constraint = float(grad @ desired + margin)

        if constraint >= 0.0:
            return vx, vy  # safe

        norm_sq = float(grad @ grad)
        if norm_sq < 1e-9:
            return vx, vy

        safe_global = desired - (constraint / norm_sq) * grad
        # Back to local
        safe_lx = safe_global[0] * cos_y + safe_global[1] * sin_y
        safe_ly = -safe_global[0] * sin_y + safe_global[1] * cos_y
        return float(safe_lx), float(safe_ly)

    def _set_gz_pose(self, x, y, z, yaw):
        """Move rexrov2 model in Gazebo using gz service set_pose."""
        quat = tft.quaternion_from_euler(0, 0, yaw)
        req = (f'name: "rexrov2", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}}}')
        try:
            subprocess.Popen(
                ['gz', 'service', '-s', '/world/oceans_waves/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '100', '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass


def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
