#!/usr/bin/env python3
"""EROAS Complete Navigation Node — single file, everything included.

Combines SPD2C planner + CBF safety + kinematic driver (set_pose).
No external dependencies. Just works.
"""
import math
import subprocess
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations as tft


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class EROASNav(Node):
    def __init__(self):
        super().__init__('eroas_nav')

        self.declare_parameter('waypoints', '0,0,-20;30,0,-20')
        self.declare_parameter('cruise_speed', 0.45)
        self.declare_parameter('max_yaw_rate', 0.30)
        self.declare_parameter('yaw_kp', 0.35)
        self.declare_parameter('target_depth', -20.0)
        self.declare_parameter('goal_tolerance', 5.0)
        self.declare_parameter('R_o', 4.0)
        self.declare_parameter('kappa', 0.09)
        self.declare_parameter('sonar_range', 15.0)
        self.declare_parameter('world_name', 'oceans_waves')

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.target_depth = self.get_parameter('target_depth').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.R_o = self.get_parameter('R_o').value
        self.kappa = self.get_parameter('kappa').value
        self.sonar_range = self.get_parameter('sonar_range').value
        self.world_name = self.get_parameter('world_name').value

        self.waypoints = self._parse_wp(self.get_parameter('waypoints').value)
        self.wp_idx = 0

        # State
        self.pos = np.array([0.0, 0.0, -20.0])
        self.yaw = 0.0
        self.pose_ok = False
        self.obs_points = np.empty((0, 3))

        # Subs
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self._pose_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rexrov2/blueview_p900_point_cloud',
                                 self._pc_cb, qos_profile_sensor_data)

        # Pub for monitoring
        self.cmd_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        # Main loop at 10 Hz
        self.dt = 0.1
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(f'[EROAS] Started. Waypoints: {len(self.waypoints)}')

    def _parse_wp(self, val):
        wps = []
        for s in str(val).split(';'):
            s = s.strip()
            if not s:
                continue
            parts = [float(x) for x in s.split(',')]
            if len(parts) == 3:
                wps.append(np.array(parts))
        return wps or [np.array([30.0, 0.0, -20.0])]

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        self.pose_ok = True

    def _pc_cb(self, msg):
        if not self.pose_ok:
            return
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                        skip_nans=True, reshape_organized_cloud=False)
            pts = np.asarray(pts, dtype=float).reshape((-1, 3))
        except Exception:
            return
        if pts.size == 0:
            return
        # Keep points within sonar_range of vehicle
        pts_round = np.round(pts)
        if len(self.obs_points) > 0:
            all_pts = np.unique(np.vstack((self.obs_points, pts_round)), axis=0)
        else:
            all_pts = np.unique(pts_round, axis=0)
        dists = np.linalg.norm(all_pts - self.pos, axis=1)
        self.obs_points = all_pts[dists <= self.sonar_range]

    def _tick(self):
        if not self.pose_ok:
            return

        # Check waypoint
        if self.wp_idx >= len(self.waypoints):
            self._set_pose(self.pos[0], self.pos[1], self.pos[2], self.yaw)
            return
        goal = self.waypoints[self.wp_idx]
        if np.linalg.norm(self.pos - goal) < self.goal_tolerance:
            self.wp_idx += 1
            self.get_logger().info(f'[EROAS] Waypoint reached! Next: {self.wp_idx}')
            if self.wp_idx >= len(self.waypoints):
                self.get_logger().info('[EROAS] Mission complete!')
                return
            goal = self.waypoints[self.wp_idx]

        # Compute desired heading toward goal
        dx = goal[0] - self.pos[0]
        dy = goal[1] - self.pos[1]
        dz = goal[2] - self.pos[2]
        goal_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(goal_yaw - self.yaw),
                             math.cos(goal_yaw - self.yaw))

        # Desired velocities (SPD2C: go toward goal)
        vx = self.cruise_speed
        vy = 0.0
        vz = clamp(0.3 * dz, -0.3, 0.3)
        wz = clamp(self.yaw_kp * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)

        # CBF filter: check nearest obstacle
        if len(self.obs_points) > 0:
            dists = np.linalg.norm(self.obs_points - self.pos, axis=1)
            idx = np.argmin(dists)
            nearest_dist = dists[idx]
            nearest_pt = self.obs_points[idx]
            h = nearest_dist**2 - self.R_o**2

            if h < float('inf'):
                # Project velocity through CBF
                cos_y = math.cos(self.yaw)
                sin_y = math.sin(self.yaw)
                gx = vx * cos_y - vy * sin_y
                gy = vx * sin_y + vy * cos_y
                grad = 2.0 * (self.pos[:2] - nearest_pt[:2])
                margin = self.kappa * (h - 0.5)
                desired = np.array([gx, gy])
                constraint = float(grad @ desired + margin)

                if constraint < 0:
                    norm_sq = float(grad @ grad)
                    if norm_sq > 1e-9:
                        safe_g = desired - (constraint / norm_sq) * grad
                        vx = float(safe_g[0] * cos_y + safe_g[1] * sin_y)
                        vy = float(-safe_g[0] * sin_y + safe_g[1] * cos_y)

                # If very close, also try to go up/down
                if nearest_dist < self.R_o + 2.0:
                    obs_dz = nearest_pt[2] - self.pos[2]
                    if abs(obs_dz) < 5.0:
                        # Obstacle at same depth — go up
                        vz = 0.3
                        self.get_logger().info(
                            f'[EROAS] Vertical evasion! dist={nearest_dist:.1f} vz={vz:.2f}')

        # Kinematic update
        new_yaw = self.yaw + wz * self.dt
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        new_x = self.pos[0] + (vx * cos_y - vy * sin_y) * self.dt
        new_y = self.pos[1] + (vx * sin_y + vy * cos_y) * self.dt
        new_z = self.pos[2] + vz * self.dt

        # Move model in Gazebo
        self._set_pose(new_x, new_y, new_z, new_yaw)

        # Publish for monitoring
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.linear.z = vz
        tw.angular.z = wz
        self.cmd_pub.publish(tw)

    def _set_pose(self, x, y, z, yaw):
        q = tft.quaternion_from_euler(0, 0, yaw)
        req = (f'name: "rexrov2", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}}}')
        try:
            subprocess.Popen(
                ['gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '50', '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass


def main():
    rclpy.init()
    node = EROASNav()
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
