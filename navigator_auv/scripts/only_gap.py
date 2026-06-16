#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from marine_acoustic_msgs.msg import ProjectedSonarImage
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64


FOV_DEG = 90.0
FOV_RAD = math.radians(FOV_DEG)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class GapCandidate:
    start: int
    end: int
    mid: int
    width: int
    width_deg: float
    width_m: float
    min_clearance: float
    center_clearance: float
    touches_left: bool
    touches_right: bool
    score: float = 0.0


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')

        self.declare_parameter('cmd_vel_topic', '/rexrov2/cmd_vel_1')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('sonar_topic', '/rexrov2/blueview_p900/sonar_image_raw')
        self.declare_parameter('point_cloud_topic', '/rexrov2/blueview_p900_point_cloud')
        self.declare_parameter('use_raw_sonar', False)
        self.declare_parameter('use_point_cloud_sonar', True)
        self.declare_parameter('waypoints', '29,97,-50;31,110,-55;30,90,-90;30,120,-40')
        self.declare_parameter('sonar_timeout', 1.0)
        self.declare_parameter('cruise_speed', 0.55)
        self.declare_parameter('fallback_speed', 0.35)
        self.declare_parameter('fallback_yaw_kp', 0.8)
        self.declare_parameter('fallback_max_yaw_rate', 0.5)
        self.declare_parameter('fallback_lateral_gain', 0.65)
        self.declare_parameter('fallback_min_forward_fraction', 0.20)
        self.declare_parameter('loop_waypoints', False)

        self.declare_parameter('sonar_max_range', 15.0)
        self.declare_parameter('detection_threshold', 2.0)
        self.declare_parameter('min_detection_range', 0.45)
        self.declare_parameter('vehicle_width_m', 1.50)
        self.declare_parameter('gap_safety_margin_m', 0.35)
        self.declare_parameter('preferred_gap_width_m', 3.5)
        self.declare_parameter('free_range_threshold', 6.0)
        self.declare_parameter('min_gap_width_deg', 7.5)
        self.declare_parameter('preferred_gap_width_deg', 18.0)
        self.declare_parameter('collision_distance', 2.2)
        self.declare_parameter('hard_stop_distance', 0.9)
        self.declare_parameter('slowdown_distance', 6.0)
        self.declare_parameter('min_forward_speed', 0.18)
        self.declare_parameter('scan_forward_speed', 0.06)
        self.declare_parameter('scan_lateral_speed', 0.12)
        self.declare_parameter('scan_yaw_rate', 0.35)
        self.declare_parameter('yaw_kp', 0.85)
        self.declare_parameter('max_yaw_rate', 0.65)
        self.declare_parameter('max_yaw_delta', 0.08)
        self.declare_parameter('max_speed_delta', 0.12)
        self.declare_parameter('recovery_lateral_speed', 0.22)
        self.declare_parameter('recovery_yaw_bias', 0.18)
        self.declare_parameter('recovery_speed_threshold', 0.05)
        self.declare_parameter('recovery_timeout', 3.0)
        self.declare_parameter('progress_recovery_distance', 0.25)
        self.declare_parameter('progress_recovery_timeout', 3.0)
        self.declare_parameter('progress_recovery_release_distance', 0.55)
        self.declare_parameter('point_cloud_parse_interval', 0.25)
        self.declare_parameter('stale_sonar_timeout', 3.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        sonar_topic = self.get_parameter('sonar_topic').value
        point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.use_raw_sonar = bool(self.get_parameter('use_raw_sonar').value)
        self.use_point_cloud_sonar = bool(
            self.get_parameter('use_point_cloud_sonar').value)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.img_pub = self.create_publisher(Image, '/rexrov2/detected_objects', 10)
        self.joint_pub = self.create_publisher(
            Float64, '/rexrov2/sonar_joint_position_controller/command', 10)
        self.sonar_move_pub = self.create_publisher(
            Float64, '/rexrov2/sonar/moving', 10)
        self.context_h_pub = self.create_publisher(Float64, '/rexrov2/scg/h', 10)
        self.gap_angle_pub = self.create_publisher(
            Float64, '/rexrov2/scg/selected_gap_angle', 10)
        self.gap_width_pub = self.create_publisher(
            Float64, '/rexrov2/scg/selected_gap_width', 10)
        self.obstacle_count_pub = self.create_publisher(
            Float64, '/rexrov2/scg/obstacle_count', 10)
        self.gap_count_pub = self.create_publisher(
            Float64, '/rexrov2/scg/gap_count', 10)

        self.subscription_handles = [
            self.create_subscription(Odometry, pose_topic, self.pose_callback, 10),
        ]
        if self.use_raw_sonar:
            self.subscription_handles.append(
                self.create_subscription(
                    ProjectedSonarImage, sonar_topic,
                    self.sonar_image_raw_callback, qos_profile_sensor_data))
        if self.use_point_cloud_sonar:
            self.subscription_handles.append(
                self.create_subscription(
                    PointCloud2, point_cloud_topic,
                    self.point_cloud_callback, qos_profile_sensor_data))

        self.create_timer(1.0 / 8.0, self.run_once)

        self.beam_directions = []
        self.ranges = []
        self.data_raw = None
        self.ping_info = None
        self.data_available = False
        self.last_sonar_time = None
        self.pc_free_mask = None
        self.pc_hit_ranges = None
        self.pc_has_hit = None
        self.pc_beam_count = 512
        self.last_point_cloud_time = None
        self.latest_point_cloud_msg = None
        self.latest_point_cloud_time = None
        self.last_point_cloud_parse_time = None
        self.last_processed_point_cloud_stamp = None

        self.latest_pose_msg = None
        self.latest_pose_time = None
        self.last_processed_pose_stamp = None
        self.pose = None
        self.current_planar_speed = 0.0
        self.target_x = self.target_y = self.target_z = None
        self.goal_yaw_error = 0.0
        self.global_angle = math.pi / 2.0

        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.current_goal_index = 0
        self.current_goal = self.waypoints[self.current_goal_index]

        self.cmd_vel_topic = cmd_vel_topic
        self.pose_topic = pose_topic
        self.sonar_topic = sonar_topic
        self.point_cloud_topic = point_cloud_topic

        self.sonar_timeout_ns = int(
            float(self.get_parameter('sonar_timeout').value) * 1e9)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.fallback_speed = float(self.get_parameter('fallback_speed').value)
        self.fallback_yaw_kp = float(self.get_parameter('fallback_yaw_kp').value)
        self.fallback_max_yaw_rate = float(
            self.get_parameter('fallback_max_yaw_rate').value)
        self.fallback_lateral_gain = float(
            self.get_parameter('fallback_lateral_gain').value)
        self.fallback_min_forward_fraction = float(
            self.get_parameter('fallback_min_forward_fraction').value)
        self.loop_waypoints = bool(self.get_parameter('loop_waypoints').value)

        self.sonar_max_range = float(self.get_parameter('sonar_max_range').value)
        self.detection_threshold = float(
            self.get_parameter('detection_threshold').value)
        self.min_detection_range = float(
            self.get_parameter('min_detection_range').value)
        self.vehicle_width_m = float(self.get_parameter('vehicle_width_m').value)
        self.gap_safety_margin_m = float(
            self.get_parameter('gap_safety_margin_m').value)
        self.preferred_gap_width_m = float(
            self.get_parameter('preferred_gap_width_m').value)
        self.min_required_gap_width_m = (
            self.vehicle_width_m + 2.0 * self.gap_safety_margin_m)
        self.free_range_threshold = float(
            self.get_parameter('free_range_threshold').value)
        self.min_gap_width_deg = float(
            self.get_parameter('min_gap_width_deg').value)
        self.preferred_gap_width_deg = float(
            self.get_parameter('preferred_gap_width_deg').value)
        self.collision_distance = float(
            self.get_parameter('collision_distance').value)
        self.hard_stop_distance = float(
            self.get_parameter('hard_stop_distance').value)
        self.slowdown_distance = float(
            self.get_parameter('slowdown_distance').value)
        self.min_forward_speed = float(
            self.get_parameter('min_forward_speed').value)
        self.scan_forward_speed = float(
            self.get_parameter('scan_forward_speed').value)
        self.scan_lateral_speed = float(
            self.get_parameter('scan_lateral_speed').value)
        self.scan_yaw_rate = float(self.get_parameter('scan_yaw_rate').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.max_yaw_delta = float(self.get_parameter('max_yaw_delta').value)
        self.max_speed_delta = float(self.get_parameter('max_speed_delta').value)
        self.recovery_lateral_speed = float(
            self.get_parameter('recovery_lateral_speed').value)
        self.recovery_yaw_bias = float(
            self.get_parameter('recovery_yaw_bias').value)
        self.recovery_speed_threshold = float(
            self.get_parameter('recovery_speed_threshold').value)
        self.recovery_timeout = float(
            self.get_parameter('recovery_timeout').value)
        self.progress_recovery_distance = float(
            self.get_parameter('progress_recovery_distance').value)
        self.progress_recovery_timeout = float(
            self.get_parameter('progress_recovery_timeout').value)
        self.progress_recovery_release_distance = float(
            self.get_parameter('progress_recovery_release_distance').value)
        self.point_cloud_parse_interval = float(
            self.get_parameter('point_cloud_parse_interval').value)
        self.stale_sonar_timeout_ns = int(
            float(self.get_parameter('stale_sonar_timeout').value) * 1e9)

        self.last_cmd = Twist()
        self.last_commanded_speed = 0.0
        self.last_selected_beam = None
        self.low_speed_since = None
        self.recovery_mode = False
        self.recovery_started = None
        self.progress_anchor_xy = None
        self.progress_anchor_time = None
        self.recovery_anchor_xy = None
        self.scan_direction = 1.0
        self.sonar_angle = 0.0
        self.fallback_announced = False
        self.mission_complete = False
        self.last_context_h = float('inf')
        self.gap_history = {}

    def _parse_waypoints(self, value):
        waypoints = []
        for item in str(value).split(';'):
            item = item.strip()
            if not item:
                continue
            fields = [float(v.strip()) for v in item.split(',')]
            if len(fields) != 3:
                raise ValueError(f'Invalid waypoint "{item}", expected x,y,z')
            waypoints.append(tuple(fields))
        return waypoints or [
            (29.0, 97.0, -50.0),
            (31.0, 110.0, -55.0),
            (30.0, 90.0, -90.0),
            (30.0, 120.0, -40.0),
        ]

    def pose_callback(self, pose_msg):
        self.latest_pose_msg = pose_msg
        self.latest_pose_time = self.get_clock().now()

    def _pose_stamp_key(self, msg):
        stamp = msg.header.stamp
        return stamp.sec, stamp.nanosec

    def _refresh_pose_state(self):
        if self.latest_pose_msg is None:
            return

        stamp_key = self._pose_stamp_key(self.latest_pose_msg)
        if stamp_key == self.last_processed_pose_stamp:
            return
        self.last_processed_pose_stamp = stamp_key

        self.target_x, self.target_y, self.target_z = self.current_goal
        pose = self.latest_pose_msg.pose.pose
        self.pose = pose

        x, y, z = pose.position.x, pose.position.y, pose.position.z
        yaw = yaw_from_quaternion(pose.orientation)
        target_yaw = math.atan2(self.target_y - y, self.target_x - x)
        self.goal_yaw_error = wrap_pi(target_yaw - yaw)
        self.global_angle = math.pi / 2.0 + clamp(
            self.goal_yaw_error, -FOV_RAD / 2.0, FOV_RAD / 2.0)

        linear = self.latest_pose_msg.twist.twist.linear
        self.current_planar_speed = math.hypot(linear.x, linear.y)
        self._update_recovery_state()

        d_xy = math.hypot(self.target_x - x, self.target_y - y)
        d = math.hypot(d_xy, self.target_z - z)
        if not self.mission_complete and d < 5.0:
            self.update_goal()

    def sonar_image_raw_callback(self, data):
        if not self._message_stamp_is_fresh(data):
            return
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.ping_info = data.ping_info
        if self.beam_directions and self.ranges and data.image.data and self.ping_info:
            self.data_raw = np.frombuffer(data.image.data, dtype=np.uint8)
            self.data_available = True
            self.last_sonar_time = self.get_clock().now()
            self.fallback_announced = False

    def point_cloud_callback(self, msg):
        self.latest_point_cloud_msg = msg
        self.latest_point_cloud_time = self.get_clock().now()
        self.fallback_announced = False

    def update_goal(self):
        if self.current_goal_index < len(self.waypoints) - 1:
            self.current_goal_index += 1
            self.current_goal = self.waypoints[self.current_goal_index]
            self.get_logger().info(
                f'[NAVIGATION_DECISION] Next waypoint: {self.current_goal}')
        elif self.loop_waypoints:
            self.current_goal_index = 0
            self.current_goal = self.waypoints[self.current_goal_index]
            self.get_logger().info(
                f'[NAVIGATION_DECISION] Restarting waypoint loop: {self.current_goal}')
        else:
            if not self.mission_complete:
                self.get_logger().info('[NAVIGATION_DECISION] Final waypoint reached')
            self.mission_complete = True

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def _message_stamp_is_fresh(self, msg):
        if not hasattr(msg, 'header'):
            return True
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            return True
        now_sec = self.get_clock().now().nanoseconds / 1e9
        age = now_sec - stamp_sec
        return age <= (self.sonar_timeout_ns / 1e9) or age < 0.0

    def _update_recovery_state(self):
        now = self.get_clock().now()
        recovery_command_threshold = max(0.03, self.min_forward_speed * 0.25)
        commanded_motion = self.last_commanded_speed >= recovery_command_threshold
        low_measured_speed = self.current_planar_speed < self.recovery_speed_threshold
        low_progress = self._commanded_progress_stalled(commanded_motion, now)

        if commanded_motion and (low_measured_speed or low_progress):
            if self.low_speed_since is None:
                self.low_speed_since = now
            else:
                timeout = min(self.recovery_timeout, self.progress_recovery_timeout)
                if self._elapsed(self.low_speed_since) > timeout:
                    if not self.recovery_mode:
                        reason = 'pose progress stalled' if low_progress else 'low measured velocity'
                        self.get_logger().warning(
                            '[NAVIGATION_DECISION] Recovery mode triggered: '
                            f'{reason} persisted for more than {timeout:.1f} seconds')
                    self.recovery_mode = True
                    self.recovery_started = self.recovery_started or now
                    if self.recovery_anchor_xy is None and self.pose is not None:
                        self.recovery_anchor_xy = np.array([
                            self.pose.position.x,
                            self.pose.position.y,
                        ], dtype=float)
        else:
            self.low_speed_since = None
            release_by_speed = (
                self.current_planar_speed > self.recovery_speed_threshold * 1.5)
            release_by_progress = self._recovery_made_progress()
            if self.recovery_mode and (release_by_speed or release_by_progress):
                self.recovery_mode = False
                self.recovery_started = None
                self.recovery_anchor_xy = None
                self.get_logger().info(
                    '[NAVIGATION_DECISION] Recovery complete; resuming goal-biased gaps')

    def _commanded_progress_stalled(self, commanded_motion, now):
        if not commanded_motion or self.pose is None:
            self.progress_anchor_xy = None
            self.progress_anchor_time = None
            return False

        xy = np.array([self.pose.position.x, self.pose.position.y], dtype=float)
        if self.progress_anchor_xy is None:
            self.progress_anchor_xy = xy
            self.progress_anchor_time = now
            return False

        moved = float(np.linalg.norm(xy - self.progress_anchor_xy))
        if moved >= self.progress_recovery_distance:
            self.progress_anchor_xy = xy
            self.progress_anchor_time = now
            return False

        if self.progress_anchor_time is None:
            self.progress_anchor_time = now
            return False

        return self._elapsed(self.progress_anchor_time) >= self.progress_recovery_timeout

    def _recovery_made_progress(self):
        if self.pose is None or self.recovery_anchor_xy is None:
            return False

        xy = np.array([self.pose.position.x, self.pose.position.y], dtype=float)
        moved = float(np.linalg.norm(xy - self.recovery_anchor_xy))
        return moved >= self.progress_recovery_release_distance

    def _range_values(self, range_count):
        try:
            values = np.asarray(self.ranges, dtype=float)
        except (TypeError, ValueError):
            values = np.array([])

        if len(values) == range_count and np.isfinite(values).any():
            max_value = float(np.nanmax(values))
            if max_value > 0.0:
                return values

        return np.linspace(0.0, self.sonar_max_range, range_count)

    def _sonar_matrix(self):
        beam_count = len(self.beam_directions) or 512
        range_count = len(self.ranges)
        if self.data_raw is None or beam_count <= 0 or range_count <= 0:
            return None, None, None

        expected_size = beam_count * range_count
        if self.data_raw.size < expected_size:
            self.get_logger().warning(
                '[GAP_FINDING] Sonar image too small for declared beams/ranges: '
                f'{self.data_raw.size} < {expected_size}')
            return None, None, None

        data = self.data_raw[:expected_size].reshape((range_count, beam_count))
        return data, self._range_values(range_count), beam_count

    def _fill_short_blocked_runs(self, free_mask, max_blocked_width=3):
        cleaned = free_mask.copy()
        start = None
        for index, is_free in enumerate(np.append(cleaned, True)):
            if not is_free and start is None:
                start = index
            elif is_free and start is not None:
                end = index - 1
                width = end - start + 1
                if start > 0 and end < len(cleaned) - 1 and width <= max_blocked_width:
                    cleaned[start:end + 1] = True
                start = None
        return cleaned

    def _classify_beams(self, data, ranges):
        range_count, beam_count = data.shape
        start_bin = int(np.searchsorted(ranges, self.min_detection_range))
        start_bin = clamp(start_bin, 1, max(1, range_count - 4))
        end_bin = int(np.searchsorted(ranges, self.sonar_max_range))
        end_bin = clamp(end_bin, start_bin + 3, range_count)

        scan = data[start_bin:end_bin, :].astype(float)
        if scan.shape[0] >= 3:
            averaged = (scan[:-2, :] + scan[1:-1, :] + scan[2:, :]) / 3.0
            offset = 1
        else:
            averaged = scan
            offset = 0

        hits = averaged > self.detection_threshold
        has_hit = hits.any(axis=0)
        first_hit = np.argmax(hits, axis=0) + start_bin + offset
        hit_bins = np.where(has_hit, first_hit, range_count - 1)
        hit_ranges = np.where(has_hit, ranges[hit_bins], self.sonar_max_range)
        free_mask = np.logical_or(~has_hit, hit_ranges >= self.free_range_threshold)
        free_mask = self._fill_short_blocked_runs(free_mask)
        return free_mask, hit_ranges, has_hit

    def _empty_point_cloud_scan(self, beam_count):
        hit_ranges = np.full(beam_count, self.sonar_max_range, dtype=float)
        has_hit = np.zeros(beam_count, dtype=bool)
        free_mask = np.ones(beam_count, dtype=bool)
        return free_mask, hit_ranges, has_hit, beam_count

    def _scan_from_point_cloud(self, msg):
        try:
            points = pc2.read_points_numpy(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
                reshape_organized_cloud=False)
            points = np.asarray(points, dtype=float)
            if points.size == 0:
                return self._empty_point_cloud_scan(self.pc_beam_count)

            points = points.reshape((-1, 3))
            beam_count = self.pc_beam_count
            ranges = np.linalg.norm(points, axis=1)
            azimuth = np.arctan2(points[:, 1], points[:, 0])
            valid = np.logical_and.reduce((
                np.isfinite(ranges),
                points[:, 0] > 0.0,
                ranges >= self.min_detection_range,
                ranges < self.sonar_max_range - 0.05,
                np.abs(azimuth) <= FOV_RAD / 2.0,
            ))
            if not np.any(valid):
                return self._empty_point_cloud_scan(beam_count)

            beam_indices = np.round(
                (azimuth[valid] + FOV_RAD / 2.0) /
                FOV_RAD * (beam_count - 1)).astype(int)
            beam_indices = np.clip(beam_indices, 0, beam_count - 1)
            hit_ranges = np.full(beam_count, self.sonar_max_range, dtype=float)
            np.minimum.at(hit_ranges, beam_indices, ranges[valid])
            has_hit = hit_ranges < self.sonar_max_range - 0.05

            free_mask = np.logical_or(~has_hit, hit_ranges >= self.free_range_threshold)
            free_mask = self._fill_short_blocked_runs(free_mask)
            return free_mask, hit_ranges.astype(float), has_hit, beam_count
        except Exception as exc:
            self.get_logger().warning(
                f'[GAP_FINDING] Failed to parse FLS point cloud: {exc}')
            return None

    def _point_cloud_stamp_key(self, msg):
        stamp = msg.header.stamp
        return stamp.sec, stamp.nanosec

    def _latest_point_cloud_is_fresh(self, allow_stale=False):
        if self.latest_point_cloud_msg is None or self.latest_point_cloud_time is None:
            return False
        now = self.get_clock().now()
        timeout_ns = self.sonar_timeout_ns
        if allow_stale:
            timeout_ns = max(timeout_ns, self.stale_sonar_timeout_ns)
        return (now - self.latest_point_cloud_time).nanoseconds <= timeout_ns

    def _stale_point_cloud_is_available(self):
        if (self.pc_free_mask is None or self.pc_hit_ranges is None or
                self.pc_has_hit is None or self.last_point_cloud_time is None):
            return False
        now = self.get_clock().now()
        return (now - self.last_point_cloud_time).nanoseconds <= self.stale_sonar_timeout_ns

    def _refresh_point_cloud_scan(self, allow_stale=False):
        if not self._latest_point_cloud_is_fresh(allow_stale=allow_stale):
            return False

        now = self.get_clock().now()
        stamp_key = self._point_cloud_stamp_key(self.latest_point_cloud_msg)
        if stamp_key == self.last_processed_point_cloud_stamp:
            return self._point_cloud_is_fresh() or (
                allow_stale and self._stale_point_cloud_is_available())

        if self.last_point_cloud_parse_time is not None:
            elapsed = (now - self.last_point_cloud_parse_time).nanoseconds / 1e9
            if elapsed < self.point_cloud_parse_interval:
                return self._point_cloud_is_fresh() or (
                    allow_stale and self._stale_point_cloud_is_available())

        scan = self._scan_from_point_cloud(self.latest_point_cloud_msg)
        self.last_point_cloud_parse_time = now
        self.last_processed_point_cloud_stamp = stamp_key
        if scan is None:
            return self._point_cloud_is_fresh() or (
                allow_stale and self._stale_point_cloud_is_available())

        self.pc_free_mask, self.pc_hit_ranges, self.pc_has_hit, self.pc_beam_count = scan
        self.last_point_cloud_time = now
        return True

    def _target_beam(self, beam_count):
        center = (beam_count - 1) / 2.0
        beam = center + (clamp(self.goal_yaw_error, -FOV_RAD / 2.0, FOV_RAD / 2.0)
                         / FOV_RAD) * (beam_count - 1)
        return int(clamp(round(beam), 0, beam_count - 1))

    def _beam_to_angle(self, beam, beam_count):
        center = (beam_count - 1) / 2.0
        return (beam - center) * (FOV_RAD / max(1, beam_count - 1))

    def _find_gaps(self, free_mask, hit_ranges):
        beam_count = len(free_mask)
        deg_per_beam = FOV_DEG / max(1, beam_count - 1)
        min_width = max(2, int(math.ceil(self.min_gap_width_deg / deg_per_beam)))
        gaps = []
        start = None

        for index, is_free in enumerate(np.append(free_mask, False)):
            if is_free and start is None:
                start = index
            elif not is_free and start is not None:
                end = index - 1
                width = end - start + 1
                if width >= min_width:
                    mid = (start + end) // 2
                    center_margin = max(1, min(width // 3, min_width // 2))
                    center_start = max(start, mid - center_margin)
                    center_end = min(end, mid + center_margin)
                    gap_ranges = hit_ranges[start:end + 1]
                    center_ranges = hit_ranges[center_start:center_end + 1]
                    width_rad = math.radians(width * deg_per_beam)
                    center_clearance = float(np.min(center_ranges))
                    width_range = (
                        center_clearance if np.isfinite(center_clearance) and center_clearance > 0.0
                        else self.sonar_max_range
                    )
                    width_m = 2.0 * width_range * math.tan(max(1e-6, 0.5 * width_rad))
                    gaps.append(GapCandidate(
                        start=start,
                        end=end,
                        mid=mid,
                        width=width,
                        width_deg=width * deg_per_beam,
                        width_m=width_m,
                        min_clearance=float(np.min(gap_ranges)),
                        center_clearance=center_clearance,
                        touches_left=start == 0,
                        touches_right=end == beam_count - 1,
                    ))
                start = None

        return gaps

    def _count_runs(self, mask):
        count = 0
        in_run = False
        for value in mask:
            if value and not in_run:
                count += 1
                in_run = True
            elif not value:
                in_run = False
        return count

    def _obstacle_boundaries(self, has_hit, hit_ranges, beam_count):
        boundaries = []
        start = None
        for index, blocked in enumerate(np.append(has_hit, False)):
            if blocked and start is None:
                start = index
            elif not blocked and start is not None:
                end = index - 1
                start_angle = self._beam_to_angle(start, beam_count)
                end_angle = self._beam_to_angle(end, beam_count)
                nearest = float(np.min(hit_ranges[start:end + 1]))
                boundaries.append((start_angle, end_angle, nearest))
                start = None
        return boundaries

    def _format_boundaries(self, boundaries, limit=5):
        if not boundaries:
            return 'none'
        parts = [
            f'({start:.2f},{end:.2f},{nearest:.2f}m)'
            for start, end, nearest in boundaries[:limit]
        ]
        if len(boundaries) > limit:
            parts.append(f'+{len(boundaries) - limit}more')
        return '[' + ','.join(parts) + ']'

    def _score_gaps(self, gaps, target_beam, beam_count):
        center = (beam_count - 1) / 2.0
        for gap in gaps:
            target_distance = abs(gap.mid - target_beam) / max(1.0, center)
            target_score = max(0.0, 1.0 - target_distance)
            if gap.start <= target_beam <= gap.end:
                target_score += 0.35
            width_score = min(
                1.35,
                (gap.width_m if np.isfinite(gap.width_m) else self.sonar_max_range)
                / max(self.preferred_gap_width_m, 1e-6))
            clearance_score = min(1.0, gap.center_clearance / self.sonar_max_range)
            edge_penalty = 0.08 if gap.touches_left or gap.touches_right else 0.0

            if self.recovery_mode:
                center_distance = abs(gap.mid - center) / max(1.0, center)
                gap.score = 2.2 * width_score + clearance_score - 0.2 * center_distance
            else:
                gap.score = (
                    3.0 * target_score +
                    0.8 * width_score +
                    0.7 * clearance_score -
                    edge_penalty
                )

        return sorted(gaps, key=lambda g: g.score, reverse=True)

    def _boundedness_check(self, gap):
        if gap is None:
            return 'no_gap', False
        if gap.width_m < self.min_required_gap_width_m:
            return (
                'bounded_unsafe_width_m='
                f'{gap.width_m:.2f}<required={self.min_required_gap_width_m:.2f}',
                False)
        if gap.touches_left and gap.touches_right:
            return 'unbounded_full_fov_traversable', True
        if gap.touches_left:
            return 'open_left_edge_traversable', True
        if gap.touches_right:
            return 'open_right_edge_traversable', True
        return 'bounded_by_obstacles_traversable', True

    def _gap_aim_beam(self, gap, target_beam, beam_count):
        if gap is None:
            return None

        edge_margin = max(1, min(gap.width // 5, int(beam_count * 0.03)))
        inner_start = min(gap.end, gap.start + edge_margin)
        inner_end = max(gap.start, gap.end - edge_margin)
        if inner_start > inner_end:
            inner_start, inner_end = gap.start, gap.end

        return int(clamp(target_beam, inner_start, inner_end))

    def _convergence_check(self, gap, hit_ranges, beam_count, target_beam):
        if gap is None:
            return False, 'no_gap'

        aim_beam = self._gap_aim_beam(gap, target_beam, beam_count)
        center_radius = max(2, min(gap.width // 5, int(beam_count * 0.04)))
        start = max(gap.start, aim_beam - center_radius)
        end = min(gap.end, aim_beam + center_radius)
        forward_clearance = float(np.min(hit_ranges[start:end + 1]))

        if forward_clearance <= self.collision_distance:
            return False, f'collision_imminent_clearance={forward_clearance:.2f}m'
        if gap.width_deg < self.min_gap_width_deg:
            return False, f'gap_too_narrow_width={gap.width_deg:.1f}deg'
        if gap.width_m <= self.min_required_gap_width_m:
            return False, (
                'gap_too_narrow_width_m='
                f'{gap.width_m:.2f} required>{self.min_required_gap_width_m:.2f}')
        if forward_clearance < self.collision_distance + 0.8:
            return False, f'path_not_converging_clearance={forward_clearance:.2f}m'

        history_key = self._gap_history_key(aim_beam, beam_count)
        previous = self.gap_history.get(history_key)
        if previous is not None:
            prev_time, prev_width, prev_clearance = previous
            dt = max(1e-3, self._now_sec() - prev_time)
            width_closing = gap.width_m < prev_width * 0.85
            clearance_closing = forward_clearance < prev_clearance - 0.35
            closing_near_course = forward_clearance < self.slowdown_distance
            if closing_near_course and (width_closing or clearance_closing):
                width_rate = (gap.width_m - prev_width) / dt
                clearance_rate = (forward_clearance - prev_clearance) / dt
                return False, (
                    'gap_converging '
                    f'width_rate={width_rate:.2f}m/s clearance_rate={clearance_rate:.2f}m/s')

        if self.last_selected_beam is not None:
            jump_deg = abs(aim_beam - self.last_selected_beam) * (
                FOV_DEG / max(1, beam_count - 1))
            if jump_deg > 35.0:
                return True, f'feasible_with_smoothed_heading_jump={jump_deg:.1f}deg'

        return True, f'feasible_clearance={forward_clearance:.2f}m'

    def _gap_history_key(self, aim_beam, beam_count):
        angle = self._beam_to_angle(aim_beam, beam_count)
        return int(round(angle / math.radians(5.0)))

    def _record_gap_history(self, gap, hit_ranges, beam_count, target_beam):
        if gap is None:
            return
        aim_beam = self._gap_aim_beam(gap, target_beam, beam_count)
        center_radius = max(2, min(gap.width // 5, int(beam_count * 0.04)))
        start = max(gap.start, aim_beam - center_radius)
        end = min(gap.end, aim_beam + center_radius)
        forward_clearance = float(np.min(hit_ranges[start:end + 1]))
        self.gap_history[self._gap_history_key(aim_beam, beam_count)] = (
            self._now_sec(), gap.width_m, forward_clearance)

        now = self._now_sec()
        self.gap_history = {
            key: value for key, value in self.gap_history.items()
            if now - value[0] <= 4.0
        }

    def _select_gap(self, gaps, hit_ranges, target_beam, beam_count):
        selected_gap = None
        selected_boundedness = ('no_gap', False)
        selected_convergence = (False, 'no_gap')

        for gap in self._score_gaps(gaps, target_beam, beam_count):
            boundedness = self._boundedness_check(gap)
            convergence = self._convergence_check(
                gap, hit_ranges, beam_count, target_beam)
            if selected_gap is None:
                selected_gap = gap
                selected_boundedness = boundedness
                selected_convergence = convergence
            if boundedness[1] and convergence[0]:
                selected_gap = gap
                selected_boundedness = boundedness
                selected_convergence = convergence
                break

        return selected_gap, selected_boundedness, selected_convergence

    def _context_h(self, nearest_obstacle, selected_gap, convergence_ok):
        nearest_margin = nearest_obstacle - self.collision_distance
        if selected_gap is None:
            return nearest_margin

        width_margin = selected_gap.width_m - self.min_required_gap_width_m
        clearance_margin = selected_gap.center_clearance - self.collision_distance
        convergence_margin = 0.0 if convergence_ok else -0.25
        return min(nearest_margin, width_margin, clearance_margin) + convergence_margin

    def _publish_context(
            self, h, selected_angle, selected_width_m,
            obstacle_count=0, gap_count=0):
        self.last_context_h = h
        self.context_h_pub.publish(Float64(data=float(h)))
        self.gap_angle_pub.publish(Float64(data=float(selected_angle)))
        self.gap_width_pub.publish(Float64(data=float(selected_width_m)))
        self.obstacle_count_pub.publish(Float64(data=float(obstacle_count)))
        self.gap_count_pub.publish(Float64(data=float(gap_count)))

    def _slew(self, desired, previous, max_delta):
        return previous + clamp(desired - previous, -max_delta, max_delta)

    def _recovery_side(self, aim_beam=None, beam_count=None):
        if aim_beam is not None and beam_count:
            angle = self._beam_to_angle(aim_beam, beam_count)
            if abs(angle) > 0.03:
                return float(np.sign(angle))
        if self.scan_direction != 0.0:
            return float(np.sign(self.scan_direction))
        return 1.0

    def _gap_is_open_corridor(self, gap):
        if gap is None:
            return False

        clear_range = 0.95 * self.sonar_max_range
        return bool(
            gap.touches_left and
            gap.touches_right and
            gap.center_clearance >= clear_range and
            gap.min_clearance >= clear_range
        )

    def _update_sonar_pivot(self, selected_gap, obstacle_close, no_gap):
        now = self._now_sec()
        sweeping = self.recovery_mode or no_gap or obstacle_close
        if sweeping:
            amplitude = 0.36 if self.recovery_mode or no_gap else 0.18
            desired = amplitude * math.sin(1.4 * now)
        else:
            desired = 0.0

        self.sonar_angle = self._slew(desired, self.sonar_angle, 0.04)
        self.joint_pub.publish(Float64(data=float(self.sonar_angle)))
        self.sonar_move_pub.publish(
            Float64(data=2.0 if abs(self.sonar_angle) > 0.02 else 0.0))

        gap_text = 'none' if selected_gap is None else (
            f'{selected_gap.start}-{selected_gap.end}@{selected_gap.mid}')
        self.get_logger().info(
            '[SONAR_PIVOT] '
            f'sonar_angle={self.sonar_angle:.3f}rad '
            f'sweeping={sweeping} selected_gap={gap_text}')

    def _command_for_gap(self, gap, target_beam, beam_count, convergence_ok):
        cmd = Twist()
        center = (beam_count - 1) / 2.0
        aim_beam = self._gap_aim_beam(gap, target_beam, beam_count)
        open_corridor = self._gap_is_open_corridor(gap)
        heading_error = (aim_beam - center) * (FOV_RAD / max(1, beam_count - 1))
        desired_yaw_rate = clamp(
            self.yaw_kp * heading_error, -self.max_yaw_rate, self.max_yaw_rate)
        recovery_side = self._recovery_side(aim_beam, beam_count)
        recovery_side_step = self.recovery_mode and not open_corridor
        if recovery_side_step:
            desired_yaw_rate = clamp(
                desired_yaw_rate + recovery_side * self.recovery_yaw_bias,
                -self.max_yaw_rate, self.max_yaw_rate)
        elif self.recovery_mode and open_corridor:
            desired_yaw_rate = clamp(
                desired_yaw_rate,
                -0.55 * self.max_yaw_rate,
                0.55 * self.max_yaw_rate)
        cmd.angular.z = self._slew(
            desired_yaw_rate, self.last_cmd.angular.z, self.max_yaw_delta)

        clearance_scale = clamp(
            (gap.center_clearance - self.collision_distance) /
            max(0.1, self.slowdown_distance - self.collision_distance),
            0.35, 1.0)
        turn_scale = clamp(
            1.0 - 0.55 * abs(heading_error) / (FOV_RAD / 2.0),
            0.35, 1.0)
        desired_speed = self.cruise_speed * clearance_scale * turn_scale

        if self.recovery_mode:
            desired_speed = max(self.min_forward_speed, min(desired_speed, self.fallback_speed))
            if open_corridor:
                desired_speed = max(
                    desired_speed,
                    0.75 * min(self.fallback_speed, self.cruise_speed))
        if not convergence_ok:
            desired_speed = self.min_forward_speed
        desired_speed = max(self.min_forward_speed, desired_speed)

        cmd.linear.x = self._slew(
            desired_speed, self.last_cmd.linear.x, self.max_speed_delta)
        desired_lateral = (
            self.recovery_lateral_speed * recovery_side
            if recovery_side_step else 0.0)
        cmd.linear.y = self._slew(
            desired_lateral, self.last_cmd.linear.y, self.max_speed_delta)
        return cmd, aim_beam

    def _command_for_scan(self, hit_ranges, target_beam, beam_count):
        cmd = Twist()
        nearest = float(np.min(hit_ranges)) if hit_ranges is not None else self.sonar_max_range
        open_corridor = nearest >= 0.95 * self.sonar_max_range
        if target_beam < beam_count / 2:
            self.scan_direction = -1.0
        elif target_beam > beam_count / 2:
            self.scan_direction = 1.0

        desired_yaw = self.scan_direction * self.scan_yaw_rate
        if self.recovery_mode and open_corridor:
            desired_yaw = clamp(
                desired_yaw,
                -0.55 * self.max_yaw_rate,
                0.55 * self.max_yaw_rate)
        cmd.angular.z = self._slew(
            desired_yaw, self.last_cmd.angular.z, self.max_yaw_delta)
        if nearest <= self.hard_stop_distance:
            desired_speed = 0.0
        elif nearest <= self.collision_distance:
            desired_speed = max(0.03, self.scan_forward_speed * 0.5)
        else:
            desired_speed = self.scan_forward_speed
        if self.recovery_mode and open_corridor:
            desired_speed = max(
                desired_speed,
                0.60 * min(self.fallback_speed, self.cruise_speed))
        cmd.linear.x = self._slew(
            desired_speed, self.last_cmd.linear.x, self.max_speed_delta)
        desired_lateral = 0.0
        if (self.recovery_mode and not open_corridor) or nearest <= self.collision_distance:
            desired_lateral = self.scan_lateral_speed * self.scan_direction
        cmd.linear.y = self._slew(
            desired_lateral, self.last_cmd.linear.y, self.max_speed_delta)
        return cmd

    def _publish_cmd(self, cmd, decision):
        self.cmd_vel_pub.publish(cmd)
        self.last_cmd = cmd
        self.last_commanded_speed = math.hypot(cmd.linear.x, cmd.linear.y)
        self.get_logger().info(
            '[CMD_VEL] '
            f'decision={decision} '
            f'linear=({cmd.linear.x:.3f},{cmd.linear.y:.3f},{cmd.linear.z:.3f}) '
            f'angular_z={cmd.angular.z:.3f}')

    def _publish_waypoint_fallback(self):
        if self.pose is None:
            return

        if self.mission_complete:
            cmd = Twist()
            self.joint_pub.publish(Float64(data=0.0))
            self.sonar_move_pub.publish(Float64(data=0.0))
            self.get_logger().info(
                '[NAVIGATION_DECISION] mission_complete holding final waypoint')
            self._publish_cmd(cmd, 'mission_complete_hold')
            return

        yaw_error = self.goal_yaw_error
        cmd = Twist()
        cmd.angular.z = clamp(
            self.fallback_yaw_kp * yaw_error,
            -self.fallback_max_yaw_rate,
            self.fallback_max_yaw_rate)
        forward = max(
            self.fallback_min_forward_fraction,
            math.cos(yaw_error))
        lateral_gain = self.fallback_lateral_gain
        if self.recovery_mode:
            lateral_gain = min(1.0, 1.25 * lateral_gain)
        lateral = lateral_gain * math.sin(yaw_error)
        norm = max(1e-6, math.hypot(forward, lateral))
        cmd.linear.x = self.fallback_speed * forward / norm
        cmd.linear.y = self.fallback_speed * lateral / norm
        self.joint_pub.publish(Float64(data=0.0))
        self.sonar_move_pub.publish(Float64(data=0.0))
        self.get_logger().warning(
            '[GAP_FINDING] gaps=0 selected_gap=none reason=no_fresh_sonar')
        self.get_logger().info(
            '[FLS_SENSOR] source=none obstacle_count=0 gap_count=0 '
            'selected_gap_angle=0.000 selected_gap_width=0.00 '
            'nearest_obstacle=inf sonar_profile=fallback')
        self.get_logger().info(
            '[SCG] obstacle_count=0 gap_count=0 selected_gap_angle=0.000 '
            'selected_gap_width=0.00 boundedness=no_gap convergence=false '
            'context_h=inf free_sectors=0 obstacle_boundaries=none')
        self.get_logger().info(
            '[BOUNDEDNESS] result=no_gap traversable=false')
        self.get_logger().info(
            '[CONVERGENCE] result=false reason=no_fresh_sonar')
        self.get_logger().info(
            '[SPD2C] desired_velocity=({:.3f},{:.3f},{:.3f}) desired_yaw_rate={:.3f} '
            'selected_gap_angle=0.000 selected_gap_width=0.00'.format(
                cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z))
        self.get_logger().info(
            '[RECOVERY] mode={} velocity={:.3f} low_speed_since={} action=pose_fallback'.format(
                self.recovery_mode,
                self.current_planar_speed,
                'active' if self.low_speed_since is not None else 'none'))
        self.get_logger().info(
            '[SONAR_PIVOT] sonar_angle=0.000rad sweeping=false selected_gap=none')
        self.get_logger().info(
            '[NAVIGATION_DECISION] pose_fallback no fresh sonar frames')
        self._publish_cmd(cmd, 'pose_fallback')

        if not self.fallback_announced:
            self.get_logger().warning(
                '[NAVIGATION_DECISION] No fresh sonar frames; following waypoints using pose fallback')
            self.fallback_announced = True

    def process_data(self):
        source = 'raw_sonar'
        data, ranges, beam_count = self._sonar_matrix()
        if data is not None and self._raw_sonar_is_fresh():
            free_mask, hit_ranges, has_hit = self._classify_beams(data, ranges)
        elif self._refresh_point_cloud_scan() or self._point_cloud_is_fresh():
            free_mask = self.pc_free_mask
            hit_ranges = self.pc_hit_ranges
            has_hit = self.pc_has_hit
            beam_count = self.pc_beam_count
            source = 'point_cloud'
        elif self._refresh_point_cloud_scan(allow_stale=True) or self._stale_point_cloud_is_available():
            free_mask = self.pc_free_mask
            hit_ranges = self.pc_hit_ranges
            has_hit = self.pc_has_hit
            beam_count = self.pc_beam_count
            source = 'point_cloud_stale'
        else:
            self._publish_waypoint_fallback()
            return

        gaps = self._find_gaps(free_mask, hit_ranges)
        target_beam = self._target_beam(beam_count)
        selected_gap, boundedness, convergence = self._select_gap(
            gaps, hit_ranges, target_beam, beam_count)
        self._record_gap_history(selected_gap, hit_ranges, beam_count, target_beam)

        selected_text = 'none'
        aim_beam = None
        selected_angle = 0.0
        selected_width_deg = 0.0
        selected_width_m = 0.0
        if selected_gap is not None:
            aim_beam = self._gap_aim_beam(selected_gap, target_beam, beam_count)
            selected_angle = self._beam_to_angle(aim_beam, beam_count)
            selected_width_deg = selected_gap.width_deg
            selected_width_m = selected_gap.width_m
            selected_text = (
                f'{selected_gap.start}-{selected_gap.end}@{selected_gap.mid} '
                f'aim={aim_beam} '
                f'width={selected_gap.width_deg:.1f}deg/{selected_gap.width_m:.2f}m '
                f'clearance={selected_gap.center_clearance:.2f}m')

        obstacle_count = self._count_runs(has_hit)
        free_sector_count = self._count_runs(free_mask)
        obstacle_boundaries = self._obstacle_boundaries(
            has_hit, hit_ranges, beam_count)
        nearest_obstacle = float(np.min(hit_ranges[has_hit])) if np.any(has_hit) else self.sonar_max_range
        bounded_result, traversable = boundedness
        convergence_ok, convergence_reason = convergence
        context_h = self._context_h(nearest_obstacle, selected_gap, convergence_ok)
        self._publish_context(
            context_h, selected_angle, selected_width_m,
            obstacle_count, len(gaps))

        self.get_logger().info(
            '[FLS_SENSOR] '
            f'source={source} obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} selected_gap_width={selected_width_m:.2f} '
            f'nearest_obstacle={nearest_obstacle:.2f} free_beams={int(np.count_nonzero(free_mask))} '
            f'hit_beams={int(np.count_nonzero(has_hit))} sonar_profile={beam_count}beams')

        self.get_logger().info(
            '[SCG] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'free_sectors={free_sector_count} selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width_m:.2f} nearest_obstacle={nearest_obstacle:.2f} '
            f'boundedness={bounded_result} traversable={traversable} '
            f'convergence={convergence_ok}:{convergence_reason} context_h={context_h:.3f} '
            f'obstacle_boundaries={self._format_boundaries(obstacle_boundaries)}')

        self.get_logger().info(
            '[GAP_FINDING] '
            f'source={source} obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap={selected_text} selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width_m:.2f} '
            f'target_beam={target_beam} free_beams={int(np.count_nonzero(free_mask))}')

        self.get_logger().info(
            '[BOUNDEDNESS] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} selected_gap_width={selected_width_m:.2f} '
            f'result={bounded_result} traversable={traversable} context_h={context_h:.3f}')

        self.get_logger().info(
            '[CONVERGENCE] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} selected_gap_width={selected_width_m:.2f} '
            f'result={convergence_ok} reason={convergence_reason} context_h={context_h:.3f}')

        obstacle_close = nearest_obstacle < self.slowdown_distance
        no_gap = selected_gap is None
        self._update_sonar_pivot(selected_gap, obstacle_close, no_gap)

        if selected_gap is not None and convergence_ok:
            cmd, aim_beam = self._command_for_gap(
                selected_gap, target_beam, beam_count, convergence_ok)
            self.last_selected_beam = aim_beam
            if self.recovery_mode and self._gap_is_open_corridor(selected_gap):
                decision = 'recovery_open_gap'
            else:
                decision = 'recovery_gap' if self.recovery_mode else 'gap_follow'
        elif selected_gap is not None:
            cmd = self._command_for_scan(hit_ranges, target_beam, beam_count)
            decision = 'convergence_failed_scan'
        else:
            cmd = self._command_for_scan(hit_ranges, target_beam, beam_count)
            decision = 'no_gap_scan'

        self.get_logger().info(
            '[SPD2C] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} selected_gap_width={selected_width_m:.2f} '
            f'boundedness={bounded_result} convergence={convergence_ok}:{convergence_reason} '
            f'context_h={context_h:.3f} '
            f'desired_velocity=({cmd.linear.x:.3f},{cmd.linear.y:.3f},{cmd.linear.z:.3f}) '
            f'desired_yaw_rate={cmd.angular.z:.3f} decision={decision}')

        self.get_logger().info(
            '[RECOVERY] '
            f'mode={self.recovery_mode} obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} selected_gap_width={selected_width_m:.2f} '
            f'current_velocity={self.current_planar_speed:.3f} '
            f'scan_direction={self.scan_direction:.1f} sonar_angle={self.sonar_angle:.3f} '
            f'action={decision}')

        convergence_result = convergence[1]

        self.get_logger().info(
            '[PLANNER_CMD] '
            f'sonar_topic={self.sonar_topic} odom_topic={self.pose_topic} '
            f'cmd_vel_topic={self.cmd_vel_topic} '
            f'obstacles={obstacle_count} gaps={len(gaps)} '
            f'selected_angle={selected_angle:.3f}rad '
            f'selected_width_deg={selected_width_deg:.1f}deg '
            f'selected_width_m={selected_width_m:.2f}m '
            f'boundedness={bounded_result} '
            f'convergence={convergence_result} '
            f'context_h={context_h:.3f} '
            f'planner_linear=({cmd.linear.x:.3f},{cmd.linear.y:.3f},{cmd.linear.z:.3f}) '
            f'planner_angular_z={cmd.angular.z:.3f}')

        self.get_logger().info(
            '[NAVIGATION_DECISION] '
            f'decision={decision} source={source} recovery={self.recovery_mode} '
            f'nearest_obstacle={nearest_obstacle:.2f}m '
            f'sonar_angle={self.sonar_angle:.3f}rad')
        self._publish_cmd(cmd, decision)

    def _raw_sonar_is_fresh(self):
        now = self.get_clock().now()
        return (
            self.data_available and
            self.last_sonar_time is not None and
            (now - self.last_sonar_time).nanoseconds <= self.sonar_timeout_ns
        )

    def _point_cloud_is_fresh(self):
        now = self.get_clock().now()
        return (
            self.pc_free_mask is not None and
            self.pc_hit_ranges is not None and
            self.pc_has_hit is not None and
            self.last_point_cloud_time is not None and
            (now - self.last_point_cloud_time).nanoseconds <= self.sonar_timeout_ns
        )

    def run_once(self):
        self._refresh_pose_state()
        self.process_data()


def main():
    rclpy.init()
    node = SonarHeadingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if 'Unable to convert call argument' not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
