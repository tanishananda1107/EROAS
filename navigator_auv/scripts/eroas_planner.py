#!/usr/bin/env python3
"""EROAS SPD2C Planner — Full 3D reactive obstacle avoidance.

Implements the complete EROAS pipeline from arXiv:2411.05516:
  1. Gap Finding (horizontal)
  2. Boundedness Check
  3. Convergence Check
  4. Sonar Pivot + Vertical Escape (3D)

Publishes reference velocity to /rexrov2/cmd_vel_1, which is then
filtered by velocity_cbf.py (ST-CBF) before reaching the vehicle.
"""
import math
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


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class EROASPlanner(Node):
    """Full EROAS SPD2C planner with 3D obstacle avoidance."""

    def __init__(self):
        super().__init__('sonar_heading_node')

        # --- Parameters ---
        self.declare_parameter('waypoints', '24,55,-56;28,59,-56;34,62,-56;44,65,-56;55,66,-56;62,75,-56;61,88,-56;55,92,-56')
        self.declare_parameter('cruise_speed', 0.45)
        self.declare_parameter('max_yaw_rate', 0.30)
        self.declare_parameter('yaw_kp', 0.12)
        self.declare_parameter('speed_gain', 0.35)
        self.declare_parameter('sonar_fov_deg', 90.0)
        self.declare_parameter('num_beams', 512)
        self.declare_parameter('gap_min_beams', 150)
        self.declare_parameter('intensity_threshold', 15)
        self.declare_parameter('goal_tolerance', 5.0)
        self.declare_parameter('max_vertical_speed', 0.30)
        self.declare_parameter('pivot_climb_angle_deg', 20.0)
        self.declare_parameter('sonar_max_range', 15.0)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.speed_gain = self.get_parameter('speed_gain').value
        self.sonar_fov = math.radians(self.get_parameter('sonar_fov_deg').value)
        self.num_beams = self.get_parameter('num_beams').value
        self.gap_min_beams = self.get_parameter('gap_min_beams').value
        self.intensity_threshold = self.get_parameter('intensity_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_vz = self.get_parameter('max_vertical_speed').value
        self.pivot_climb_angle = math.radians(self.get_parameter('pivot_climb_angle_deg').value)
        self.sonar_max_range = self.get_parameter('sonar_max_range').value

        # --- Waypoints ---
        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.current_wp_idx = 0

        # --- State ---
        self.pose = None
        self.yaw = 0.0
        self.position = np.zeros(3)
        self.sonar_data = None
        self.sonar_ranges = None
        self.beam_count = 512
        self.range_count = 0
        self.point_cloud_points = None

        # --- Committed gap (prevents oscillation) ---
        self.committed_beam = None
        self.committed_time = 0.0
        self.commit_hold_seconds = 1.5  # Hold a gap for at least this long

        # --- Vertical escape state ---
        self.vertical_escape_active = False
        self.vertical_escape_direction = 0.0  # +1 = up, -1 = down
        self.vertical_escape_start_time = 0.0
        self.vertical_escape_duration = 4.0  # seconds to hold vertical motion

        # --- Sonar pivot state ---
        self.sonar_pivot_active = False

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel_1', 10)
        self.sonar_move_pub = self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)
        self.joint_pub = self.create_publisher(Float64, '/rexrov2/sonar_joint_position_controller/command', 10)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self._pose_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rexrov2/blueview_p900_point_cloud',
                                 self._pc_cb, qos_profile_sensor_data)

        # --- Control loop at 8 Hz ---
        self.create_timer(1.0 / 8.0, self._control_loop)

        self.get_logger().info(f'[EROAS] Planner started with {len(self.waypoints)} waypoints')

    # ==================================================================
    # Parsing
    # ==================================================================

    def _parse_waypoints(self, val):
        wps = []
        for item in str(val).split(';'):
            item = item.strip()
            if not item:
                continue
            parts = [float(x) for x in item.split(',')]
            if len(parts) == 3:
                wps.append(np.array(parts))
        return wps if wps else [np.array([24.0, 55.0, -56.0])]

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.position = np.array([p.x, p.y, p.z])
        self.yaw = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        self.pose = msg.pose.pose

    def _pc_cb(self, msg):
        """Process point cloud from sonar — used for gap finding."""
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                        skip_nans=True, reshape_organized_cloud=False)
            self.point_cloud_points = np.asarray(pts, dtype=float).reshape((-1, 3))
        except Exception:
            self.point_cloud_points = None

    # ==================================================================
    # Main control loop
    # ==================================================================

    def _control_loop(self):
        if self.pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # --- Check waypoint progress ---
        if self.current_wp_idx < len(self.waypoints):
            goal = self.waypoints[self.current_wp_idx]
            dist = np.linalg.norm(self.position - goal)
            if dist < self.goal_tolerance:
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    self.get_logger().info(
                        f'[EROAS] Waypoint {self.current_wp_idx-1} reached, '
                        f'next: {self.waypoints[self.current_wp_idx]}')
                else:
                    self.get_logger().info('[EROAS] All waypoints reached!')

        if self.current_wp_idx >= len(self.waypoints):
            # Mission complete — hover
            self.cmd_pub.publish(Twist())
            return

        goal = self.waypoints[self.current_wp_idx]

        # --- If vertical escape is active, continue it ---
        if self.vertical_escape_active:
            elapsed = now - self.vertical_escape_start_time
            if elapsed < self.vertical_escape_duration:
                cmd = Twist()
                cmd.linear.x = self.cruise_speed * 0.3
                cmd.linear.z = self.vertical_escape_direction * self.max_vz
                self.cmd_pub.publish(cmd)
                self.sonar_move_pub.publish(Float64(data=2.0))
                self.get_logger().info(
                    f'[EROAS] Vertical escape: vz={cmd.linear.z:.2f} elapsed={elapsed:.1f}s')
                return
            else:
                self.vertical_escape_active = False
                self.get_logger().info('[EROAS] Vertical escape complete, resuming horizontal')

        # --- Compute goal heading ---
        dx = goal[0] - self.position[0]
        dy = goal[1] - self.position[1]
        dz = goal[2] - self.position[2]
        goal_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(goal_yaw - self.yaw)

        # --- Gap finding from point cloud ---
        gap_beam, obstacle_detected, no_gap = self._find_gap_from_pc(yaw_error)

        # --- SPD2C decision ---
        cmd = Twist()

        if not obstacle_detected:
            # Free space — navigate directly toward goal
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = clamp(self.yaw_kp * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)
            # Vertical: gently move toward goal depth
            cmd.linear.z = clamp(0.3 * dz, -self.max_vz, self.max_vz)
            self.committed_beam = None
            decision = 'free_space'

        elif gap_beam is not None:
            # Gap found — steer toward it (paper Eq. 24-27)
            # Convert beam to heading offset
            center_beam = self.num_beams / 2.0
            beam_angle = (gap_beam - center_beam) * (self.sonar_fov / self.num_beams)

            # Gap commitment: don't switch gaps rapidly
            if self.committed_beam is not None:
                time_since_commit = now - self.committed_time
                if time_since_commit < self.commit_hold_seconds:
                    # Keep committed gap unless new gap is much better
                    committed_angle = (self.committed_beam - center_beam) * (self.sonar_fov / self.num_beams)
                    if abs(beam_angle - committed_angle) < math.radians(20.0):
                        beam_angle = committed_angle
                    elif abs(beam_angle) < abs(committed_angle) * 0.5:
                        # New gap is significantly better (closer to center)
                        self.committed_beam = gap_beam
                        self.committed_time = now
                else:
                    self.committed_beam = gap_beam
                    self.committed_time = now
            else:
                self.committed_beam = gap_beam
                self.committed_time = now

            # Paper Eq. 24: v_x = K_v * (psi_max - |psi_R|)
            psi_max = self.sonar_fov / 2.0
            cmd.linear.x = clamp(
                self.speed_gain * (psi_max - abs(beam_angle)),
                0.05, self.cruise_speed)

            # Paper Eq. 26: r = K_t * psi_R
            cmd.angular.z = clamp(
                self.yaw_kp * beam_angle,
                -self.max_yaw_rate, self.max_yaw_rate)

            # Vertical: move toward goal depth gently
            cmd.linear.z = clamp(0.2 * dz, -self.max_vz * 0.5, self.max_vz * 0.5)
            decision = 'gap_follow'

        elif no_gap:
            # No horizontal gap — trigger vertical escape (paper Section III-C1d)
            # Decide direction: go UP if goal is above or same level, DOWN if below
            if dz >= 0:
                self.vertical_escape_direction = 1.0  # ascend
            else:
                self.vertical_escape_direction = -1.0  # descend

            self.vertical_escape_active = True
            self.vertical_escape_start_time = now
            self.committed_beam = None

            # Trigger sonar pivot
            self.sonar_move_pub.publish(Float64(data=2.0))
            self.joint_pub.publish(Float64(data=0.5 * self.vertical_escape_direction))

            cmd.linear.x = self.cruise_speed * 0.2
            cmd.linear.z = self.vertical_escape_direction * self.max_vz
            decision = 'vertical_escape'
            self.get_logger().warn(
                f'[EROAS] No horizontal gap! Vertical escape: dir={self.vertical_escape_direction}')

        else:
            # Boundedness: obstacle on one side only — turn toward open side
            cmd.linear.x = self.cruise_speed * 0.3
            cmd.angular.z = self._boundedness_turn(yaw_error)
            cmd.linear.z = clamp(0.15 * dz, -self.max_vz * 0.3, self.max_vz * 0.3)
            decision = 'boundedness_turn'

        # Publish sonar state (0 = normal XY mode)
        if not self.vertical_escape_active:
            self.sonar_move_pub.publish(Float64(data=0.0))
            self.joint_pub.publish(Float64(data=0.0))

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[SPD2C] decision={decision} '
            f'vx={cmd.linear.x:.3f} vy={cmd.linear.y:.3f} vz={cmd.linear.z:.3f} '
            f'wz={cmd.angular.z:.3f} '
            f'pos=({self.position[0]:.1f},{self.position[1]:.1f},{self.position[2]:.1f}) '
            f'yaw={self.yaw:.2f} wp={self.current_wp_idx}')

    # ==================================================================
    # Gap Finding from Point Cloud
    # ==================================================================

    def _find_gap_from_pc(self, yaw_error):
        """Find navigable gaps from point cloud data.

        Returns: (gap_beam, obstacle_detected, no_gap)
          gap_beam: beam index of best gap center (or None)
          obstacle_detected: True if any obstacle within range
          no_gap: True if obstacles everywhere (triggers vertical escape)
        """
        if self.point_cloud_points is None or len(self.point_cloud_points) == 0:
            return None, False, False

        pts = self.point_cloud_points
        # Filter valid forward-facing points
        ranges = np.linalg.norm(pts[:, :2], axis=1)
        valid = (pts[:, 0] > 0.3) & (ranges > 0.5) & (ranges < self.sonar_max_range)

        if not np.any(valid):
            return None, False, False

        pts_valid = pts[valid]
        ranges_valid = ranges[valid]

        # Convert to beam indices (azimuth angle → beam number)
        azimuths = np.arctan2(pts_valid[:, 1], pts_valid[:, 0])
        half_fov = self.sonar_fov / 2.0

        # Filter to FOV
        in_fov = np.abs(azimuths) <= half_fov
        if not np.any(in_fov):
            return None, False, False

        azimuths_fov = azimuths[in_fov]
        ranges_fov = ranges_valid[in_fov]

        # Bin into beams
        beam_indices = ((azimuths_fov + half_fov) / self.sonar_fov * (self.num_beams - 1)).astype(int)
        beam_indices = np.clip(beam_indices, 0, self.num_beams - 1)

        # Build hit map: min range per beam
        hit_ranges = np.full(self.num_beams, self.sonar_max_range)
        np.minimum.at(hit_ranges, beam_indices, ranges_fov)

        # Beams with obstacles closer than threshold
        obstacle_threshold = self.sonar_max_range * 0.8
        has_obstacle = hit_ranges < obstacle_threshold

        if not np.any(has_obstacle):
            return None, False, False  # No obstacles detected

        # Obstacle detected — find free beams
        free_beams = ~has_obstacle

        # Find consecutive runs of free beams (gaps)
        gaps = []
        start = None
        for i in range(self.num_beams + 1):
            if i < self.num_beams and free_beams[i]:
                if start is None:
                    start = i
            else:
                if start is not None:
                    width = i - start
                    if width >= self.gap_min_beams:
                        mid = (start + i) // 2
                        gaps.append((start, i - 1, mid, width))
                    start = None

        if not gaps:
            # Check boundedness
            left_free = np.sum(free_beams[:50]) > 30
            right_free = np.sum(free_beams[-50:]) > 30

            if left_free or right_free:
                # Bounded on one side — return None gap but not no_gap
                return None, True, False
            else:
                # Fully blocked — no horizontal gap exists
                return None, True, True

        # Select gap closest to goal direction
        goal_beam = int(clamp(
            (yaw_error + half_fov) / self.sonar_fov * (self.num_beams - 1),
            0, self.num_beams - 1))

        best_gap = min(gaps, key=lambda g: abs(g[2] - goal_beam))
        return best_gap[2], True, False

    # ==================================================================
    # Boundedness Turn
    # ==================================================================

    def _boundedness_turn(self, yaw_error):
        """When obstacle is unbounded on one side, turn toward open side.
        Paper Section III-C1: LUBO → turn right, RUBO → turn left."""
        if self.point_cloud_points is None or len(self.point_cloud_points) == 0:
            return clamp(self.yaw_kp * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)

        pts = self.point_cloud_points
        # Check which side has more clearance
        left_pts = pts[pts[:, 1] > 0.5]
        right_pts = pts[pts[:, 1] < -0.5]

        left_clear = len(left_pts) == 0 or (len(left_pts) > 0 and np.min(np.linalg.norm(left_pts[:, :2], axis=1)) > 5.0)
        right_clear = len(right_pts) == 0 or (len(right_pts) > 0 and np.min(np.linalg.norm(right_pts[:, :2], axis=1)) > 5.0)

        if left_clear and not right_clear:
            return self.max_yaw_rate * 0.8  # Turn left
        elif right_clear and not left_clear:
            return -self.max_yaw_rate * 0.8  # Turn right
        else:
            # Default: turn toward goal
            return clamp(self.yaw_kp * yaw_error * 2.0, -self.max_yaw_rate, self.max_yaw_rate)


def main():
    rclpy.init()
    node = EROASPlanner()
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
