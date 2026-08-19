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

# Eq. 8-9: paper's fixed gap cardinality (L=150 beams out of its N_B=512
# BlueView P900 array).
PAPER_GAP_BEAMS = 150
PAPER_REFERENCE_BEAM_COUNT = 512


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
        self.declare_parameter('prefer_point_cloud_sonar', True)
        self.declare_parameter('waypoints', '29,97,-50;31,110,-55;30,90,-90;30,120,-40')
        self.declare_parameter('sonar_timeout', 1.0)
        self.declare_parameter('cruise_speed', 0.55)
        self.declare_parameter('fallback_speed', 0.35)
        self.declare_parameter('fallback_yaw_kp', 0.8)
        self.declare_parameter('fallback_max_yaw_rate', 0.5)
        self.declare_parameter('fallback_lateral_gain', 0.65)
        self.declare_parameter('fallback_min_forward_fraction', 0.20)
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('waypoint_tolerance', 5.0)
        self.declare_parameter('final_waypoint_tolerance', 5.0)
        self.declare_parameter('planar_waypoint_tolerance', False)

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
        self.declare_parameter('startup_hover_duration', 0.0)
        self.declare_parameter('startup_sensor_wait_timeout', 0.0)
        self.declare_parameter('pivot_min_angle', -0.8)
        self.declare_parameter('pivot_max_angle', 0.8)
        self.declare_parameter('pivot_sample_count', 81)
        self.declare_parameter('pivot_sample_timeout', 0.3)
        self.declare_parameter('pivot_sample_retries', 3)
        self.declare_parameter('vertical_gap_run_length', 30)
        self.declare_parameter('vertical_gap_safety_margin_rad', 0.0)
        self.declare_parameter('vertical_escape_duration', 4.0)
        self.declare_parameter('vertical_depth_tolerance', 0.5)
        self.declare_parameter('vertical_escape_min_duration', 0.0)
        self.declare_parameter('vertical_escape_min_planar_distance', 0.0)
        self.declare_parameter('post_vertical_resume_duration', 35.0)
        self.declare_parameter('vertical_detection_threshold', 15.0)
        # Last-resort fallback for a fully-boxed pocket: gap_follow,
        # boundedness_turn, convexity_turn and even a completed vertical
        # escape can all keep firing without ever producing real net
        # displacement when the CBF's min-norm QP sees obstacle-derived
        # constraints on every bearing at once (observed in World A: a tight
        # cluster where the horizontal scan reports free_beams=0 on every
        # side, so *every* direction the planner tries gets projected down
        # toward zero -- there is no unconstrained direction for a min-norm
        # projection to find). Track real odometry displacement independent
        # of decision state; if the vehicle hasn't covered
        # stuck_recovery_distance_threshold in stuck_recovery_timeout
        # seconds, stop trying more 2D/vertical maneuvers and reverse
        # straight back for stuck_recovery_duration -- directly away from
        # whatever it's been pressed up against is the one direction the
        # CBF constraint gradient (2*(vehicle - obstacle), see
        # velocity_cbf.py's _xy_projection_constraints_with_request) always
        # treats as safe, so it passes through un-throttled and actually
        # opens some real clearance instead of oscillating in place.
        self.declare_parameter('stuck_recovery_timeout', 45.0)
        self.declare_parameter('stuck_recovery_distance_threshold', 2.5)
        self.declare_parameter('stuck_recovery_reverse_speed', 0.3)
        # 6s (~1.8m of backup) raised nearest_obstacle from 2.05m to only
        # 3.28m in World A -- CBF still throttled the subsequent vertical
        # escape's climb rate near that cluster before it reached enough
        # height. ~15s (~4.5m) gives the follow-up climb attempt
        # meaningfully more room to actually work with.
        self.declare_parameter('stuck_recovery_duration', 15.0)
        # Backing straight out and letting gap_follow/convexity_turn re-aim
        # at the goal just re-approaches the identical corridor it was
        # already stuck in -- observed in World A: recovery genuinely opened
        # clearance (nearest_obstacle 2.05m -> 3.9m) but net position was
        # unchanged two minutes later because it drove straight back to the
        # same spot. Yawing while backing away points the vehicle at a
        # different heading before 2D nav resumes, so each attempt actually
        # samples a different direction instead of retrying the one that
        # just failed. Alternates side per attempt so it doesn't overshoot
        # into a mirror-image loop between just two headings.
        self.declare_parameter('stuck_recovery_yaw_bias', 0.5)
        # See _update_stuck_recovery's narrowing_trap comment: catches a
        # corridor that looks open but funnels shut as the vehicle
        # approaches it, before it fully boxes in again.
        self.declare_parameter('narrowing_trap_window', 6.0)
        self.declare_parameter('narrowing_trap_min_width', 60)
        self.declare_parameter('narrowing_trap_ratio', 0.35)
        # See known_bad_headings: excludes headings already confirmed (via
        # a prior trap) to be dead ends, so repeated attempts don't
        # deterministically re-select the same widest-looking-but-wrong
        # opening. ~26deg -- wide enough to cover a bay's whole mouth
        # despite the vehicle's position shifting slightly between
        # attempts, narrow enough not to blot out genuinely distinct
        # nearby headings.
        self.declare_parameter('bad_heading_tolerance', 0.45)
        # A raw 2.5m local-displacement reset was clearing known_bad_headings
        # even when the vehicle was just wandering sideways along the same
        # wall, not actually advancing -- letting it re-discover and re-try
        # openings already confirmed to be dead ends. Require real
        # goal-ward progress (distance-to-goal shrinking by this much)
        # before forgetting them instead.
        self.declare_parameter('bad_heading_clear_progress', 3.0)
        # known_bad_headings was a pure world-frame yaw blacklist with no
        # positional scoping: confirmed via a 15-minute headless run that
        # once loose of the obstacle cluster it can end up blocking the
        # correct goal-ward heading somewhere the vehicle has genuinely
        # moved on to, just because that direction coincidentally shares a
        # bearing with a dead end recorded far away (vehicle diverged 60+m
        # off course, x reaching -30 against a spawn x of ~29, without
        # turning back). Scope each entry to the position it was recorded
        # at: it only excludes candidates while the vehicle is still within
        # this radius of that position, so the blacklist naturally stops
        # applying once the vehicle has moved to different terrain instead
        # of staying globally in effect for the rest of the run.
        self.declare_parameter('bad_heading_position_radius', 20.0)

        # Paper-faithful SPD2C controller (arXiv 2411.05516 Algorithm 1 /
        # AIRLabIISc/EROAS reference only_gap.py). Kept behind a flag so
        # other world configs can keep the pre-existing heuristic.
        self.declare_parameter('paper_controller', False)
        self.declare_parameter('paper_k_t', 0.12)
        self.declare_parameter('paper_k_v', 0.35)
        self.declare_parameter('paper_psi_max', FOV_RAD)
        self.declare_parameter('paper_vx_max', 1.0)
        self.declare_parameter('paper_max_yaw_rate', 0.26)
        self.declare_parameter('paper_convexity_threshold', 0.02)
        # Paper text states Ithr=15, but the AIRLabIISc/EROAS reference
        # only_gap.py that actually produced the paper's figures uses
        # threshold=2 for the horizontal scan (threshold=15 is only used in
        # its vertical process_3d_data). Matching the verified reference.
        self.declare_parameter('paper_intensity_threshold', 2.0)
        # The vertical pivot's process_3d_data equivalent (see
        # _paper_central_sector_free) needs its own threshold per the
        # comment above -- it was reusing paper_intensity_threshold (2.0,
        # the horizontal value) instead, and over that check's much wider
        # ~52deg central band, any weak return (noise/scattering) at that
        # low a threshold marks the whole sector blocked at every sampled
        # elevation, so the pivot search always came back empty even when a
        # real vertical opening existed.
        self.declare_parameter('paper_vertical_intensity_threshold', 15.0)
        # If no horizontal gap_follow beam has been found for this long,
        # escalate straight to the vertical pivot search regardless of the
        # convexity/boundedness classification. Observed in World A: right
        # at a tight pinch (e.g. cube_6_1/cube_7), gap_follow and
        # convexity_turn can alternate forever without either resolving --
        # gap_follow finds a marginal bcl, turning toward it shrinks the
        # corridor further, convexity_turn (whose fitted curvature `a`
        # doesn't reliably read as convex enough) turns back the other way,
        # repeat. That is exactly the situation Sec III-C1's vertical pivot
        # exists for (paper Fig 8b's climb-over "hump"), so once stuck this
        # long we stop trusting the 2D convexity gate and pivot.
        self.declare_parameter('paper_stuck_timeout', 3.0)

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
        # velocity_cbf.py's own depth-hold target was a static per-world
        # value that never tracked waypoint progression -- e.g. a vertical
        # escape that correctly climbs to a shallower waypoint's depth got
        # immediately undone because the CBF kept pulling back toward the
        # original static target. This node is the authority on which
        # waypoint (and therefore which depth) is currently active, so it
        # publishes that live target for velocity_cbf.py to hold instead.
        self.target_depth_pub = self.create_publisher(
            Float64, '/rexrov2/nav/target_depth', 10)

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
        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value)
        self.final_waypoint_tolerance = float(
            self.get_parameter('final_waypoint_tolerance').value)
        self.planar_waypoint_tolerance = bool(
            self.get_parameter('planar_waypoint_tolerance').value)

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

        self.pivot_min_angle = float(self.get_parameter('pivot_min_angle').value)
        self.pivot_max_angle = float(self.get_parameter('pivot_max_angle').value)
        self.pivot_sample_count = int(self.get_parameter('pivot_sample_count').value)
        self.pivot_sample_timeout = float(
            self.get_parameter('pivot_sample_timeout').value)
        self.pivot_sample_retries = int(
            self.get_parameter('pivot_sample_retries').value)
        self.vertical_gap_run_length = int(
            self.get_parameter('vertical_gap_run_length').value)
        self.vertical_gap_safety_margin_rad = float(
            self.get_parameter('vertical_gap_safety_margin_rad').value)
        self.vertical_escape_duration = float(
            self.get_parameter('vertical_escape_duration').value)
        self.vertical_depth_tolerance = float(
            self.get_parameter('vertical_depth_tolerance').value)
        self.vertical_escape_min_duration = float(
            self.get_parameter('vertical_escape_min_duration').value)
        self.vertical_escape_min_planar_distance = float(
            self.get_parameter('vertical_escape_min_planar_distance').value)
        self.post_vertical_resume_duration = float(
            self.get_parameter('post_vertical_resume_duration').value)
        self.vertical_detection_threshold = float(
            self.get_parameter('vertical_detection_threshold').value)
        self.stuck_recovery_timeout = float(
            self.get_parameter('stuck_recovery_timeout').value)
        self.stuck_recovery_distance_threshold = float(
            self.get_parameter('stuck_recovery_distance_threshold').value)
        self.stuck_recovery_reverse_speed = float(
            self.get_parameter('stuck_recovery_reverse_speed').value)
        self.stuck_recovery_duration = float(
            self.get_parameter('stuck_recovery_duration').value)
        self.stuck_recovery_yaw_bias = float(
            self.get_parameter('stuck_recovery_yaw_bias').value)
        self.narrowing_trap_window = float(
            self.get_parameter('narrowing_trap_window').value)
        self.narrowing_trap_min_width = int(
            self.get_parameter('narrowing_trap_min_width').value)
        self.narrowing_trap_ratio = float(
            self.get_parameter('narrowing_trap_ratio').value)
        self.bad_heading_tolerance = float(
            self.get_parameter('bad_heading_tolerance').value)
        self.bad_heading_clear_progress = float(
            self.get_parameter('bad_heading_clear_progress').value)
        self.bad_heading_position_radius = float(
            self.get_parameter('bad_heading_position_radius').value)

        self.paper_controller = bool(self.get_parameter('paper_controller').value)
        self.paper_k_t = float(self.get_parameter('paper_k_t').value)
        self.paper_k_v = float(self.get_parameter('paper_k_v').value)
        self.paper_psi_max = float(self.get_parameter('paper_psi_max').value)
        self.paper_vx_max = float(self.get_parameter('paper_vx_max').value)
        self.paper_max_yaw_rate = float(
            self.get_parameter('paper_max_yaw_rate').value)
        self.paper_convexity_threshold = float(
            self.get_parameter('paper_convexity_threshold').value)
        self.paper_intensity_threshold = float(
            self.get_parameter('paper_intensity_threshold').value)
        self.paper_vertical_intensity_threshold = float(
            self.get_parameter('paper_vertical_intensity_threshold').value)
        self.paper_stuck_timeout = float(
            self.get_parameter('paper_stuck_timeout').value)
        self.paper_beam_stride = 5

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

        # Vertical pivot-search state machine (paper Sec III-C1 "Pivoting the
        # Sonar" / ROS1 reference's navigate_3d+move_sonar+process_3d_data).
        self.vpivot_active = False
        self.vpivot_sample_index = 0
        self.vpivot_angle_commanded_time = None
        self.vpivot_retries_left = 0
        self.vpivot_accepted = []
        self.vertical_escape_active = False
        self.vertical_escape_until = None
        self.vertical_escape_elevation = 0.0
        self.vertical_escape_started_at = None
        self.vertical_escape_start_xy = None
        self.vertical_resume_until = 0.0
        self.no_horizontal_gap_since = None
        self.stuck_recovery_active = False
        self.stuck_recovery_until = None
        self.stuck_recovery_count = 0
        self.stuck_recovery_best_run = -1
        self.stuck_recovery_best_yaw = None
        self.stuck_recovery_start_yaw = 0.0
        self.stuck_recovery_turn_active = False
        self.stuck_recovery_turn_until = None
        self.progress3d_anchor_xyz = None
        self.progress3d_anchor_time = None
        self.gap_width_history = []
        self.known_bad_headings = []
        self.known_bad_headings_goal_anchor = None
        self.stuck_recovery_last_turn_yaw = None

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
        tolerance = (
            self.final_waypoint_tolerance
            if self.current_goal_index >= len(self.waypoints) - 1
            else self.waypoint_tolerance)
        progress_distance = d_xy if self.planar_waypoint_tolerance else d
        if not self.mission_complete and progress_distance < tolerance:
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

    # ------------------------------------------------------------------
    # Paper-faithful SPD2C controller (arXiv 2411.05516 Algorithm 1 /
    # AIRLabIISc/EROAS reference only_gap.py + velocity_cbf.py), active
    # when the 'paper_controller' parameter is true. Operates on the raw
    # sonar intensity matrix (as the reference does), not the elaborate
    # gap-scoring heuristic used by the pre-existing path below.
    # ------------------------------------------------------------------

    def _paper_classify_beams(self, data, stride):
        range_count, beam_count = data.shape
        start_bin = min(10, max(0, range_count - 1))
        end_bin = max(start_bin + 1, range_count - 40)
        band = data[start_bin:end_bin, :].astype(float)
        free_mask = ~np.any(band > self.paper_intensity_threshold, axis=0)
        return [i for i in range(0, beam_count, stride) if free_mask[i]]

    def _paper_gap_candidates(self, free_beams, stride, required_beams):
        # A window of `required_beams` consecutive free_beams entries spans
        # indices i .. i+required_beams-1 (required_beams-1 strides), not
        # i .. i+required_beams (required_beams strides) -- the previous
        # off-by-one silently demanded one extra free beam, so a corridor
        # exactly at the paper's L=150-beam threshold (the common case right
        # at a gap's edge) was rejected as "no gap", falling through to
        # convexity_turn, which doesn't grow the corridor and just
        # oscillates the vehicle in place at that boundary indefinitely.
        mids = []
        n = len(free_beams)
        span = required_beams - 1
        for i in range(n - span):
            if free_beams[i + span] - free_beams[i] == stride * span:
                mids.append(free_beams[i + span // 2])
        return mids

    def _paper_boundedness_turn(self, free_beams, beam_count, stride):
        """Mirrors the reference check_for_boundedness: if the obstacle
        field is open on exactly one FOV edge, aim toward that edge (BO ->
        toward the goal side; LUBO/RUBO -> toward whichever edge is free).
        Beam index 0 is the physically-rightmost beam and beam_count-1 is
        the physically-leftmost beam (see sonar_frame_converter.py's
        atan2(y,x)-based beam assignment), so "aim toward the goal" means
        aiming at the high-index end when the goal is to the left
        (goal_yaw_error >= 0) and the low-index end when it's to the
        right."""
        edge_span = 40
        free_set = set(free_beams)
        right_phys_edge = set(range(0, edge_span, stride))
        left_phys_edge = set(range(max(0, beam_count - edge_span), beam_count, stride))
        right_open = right_phys_edge.issubset(free_set)
        left_open = left_phys_edge.issubset(free_set)
        if not left_open and not right_open:
            return None
        if left_open and not right_open:
            return beam_count - 1 - stride * 2
        if right_open and not left_open:
            return stride * 2
        goal_left = self.goal_yaw_error >= 0.0
        if goal_left:
            return beam_count - 1 - stride * 2
        return stride * 2

    def _paper_contour_points(self, data, ranges, stride):
        range_count, beam_count = data.shape
        start_bin = min(10, max(0, range_count - 1))
        end_bin = max(start_bin + 1, range_count - 40)
        angle_per_beam = FOV_RAD / max(1, beam_count - 1)
        max_range = float(ranges[-1]) if len(ranges) else self.sonar_max_range
        points = []
        for i in range(0, beam_count, stride):
            column = data[start_bin:end_bin, i]
            hit_idx = int(np.argmax(column > self.paper_intensity_threshold))
            if column[hit_idx] <= self.paper_intensity_threshold:
                continue
            j = start_bin + hit_idx
            angle_rad = i * angle_per_beam + math.pi / 4.0
            distance = j * max_range / range_count
            points.append((distance * math.cos(angle_rad), distance * math.sin(angle_rad)))
        return points

    def _paper_convexity(self, points):
        if len(points) < 3:
            return None, 0.0, 0.0
        pts = sorted(points, key=lambda p: p[0])
        xs, ys = [], []
        for x, y in pts:
            if not xs or x > xs[-1] + 1e-6:
                xs.append(x)
                ys.append(y)
        if len(xs) < 3:
            return None, 0.0, 0.0
        try:
            a, b, _ = np.polyfit(xs, ys, 2)
        except (np.linalg.LinAlgError, ValueError):
            return None, 0.0, 0.0
        left_slopes = [2 * a * x + b for x in xs if x < 0]
        right_slopes = [2 * a * x + b for x in xs if x > 0]
        avg_left = float(np.mean(left_slopes)) if left_slopes else 0.0
        avg_right = float(np.mean(right_slopes)) if right_slopes else 0.0
        return float(a), avg_left, avg_right

    def _paper_heading_command_for_angle(self, heading_error):
        cmd = Twist()
        desired_yaw = clamp(
            self.paper_k_t * heading_error,
            -self.paper_max_yaw_rate, self.paper_max_yaw_rate)
        desired_speed = clamp(
            self.paper_k_v * (self.paper_psi_max - abs(heading_error)),
            0.0, self.paper_vx_max)
        cmd.angular.z = self._slew(
            desired_yaw, self.last_cmd.angular.z, self.max_yaw_delta)
        cmd.linear.x = self._slew(
            desired_speed, self.last_cmd.linear.x, self.max_speed_delta)
        cmd.linear.y = 0.0
        return cmd

    def _paper_heading_command(self, bcl, beam_count):
        heading_error = self._beam_to_angle(bcl, beam_count)
        return self._paper_heading_command_for_angle(heading_error)

    def _paper_pivot_angles(self):
        if self.pivot_sample_count <= 1:
            return [self.pivot_min_angle]
        return np.linspace(
            self.pivot_min_angle, self.pivot_max_angle,
            self.pivot_sample_count).tolist()

    def _paper_central_sector_free(self, data):
        """Is there a navigable corridor at this elevation?

        Previously required *zero* pixels above threshold anywhere in the
        ~52deg x range central band (tens of thousands of pixels) -- in any
        realistically cluttered scene that's essentially never true even
        when a genuinely clear corridor exists dead ahead, since a single
        strong return anywhere in that huge area (a different obstacle off
        to the side, seafloor scatter, etc.) fails the whole elevation.
        Matches the horizontal scan's approach instead: classify beams
        free/hit and require a contiguous free run at least as wide as the
        paper's own L=150/512-beam gap cardinality (eq. 8-9), rather than
        demanding the whole swath be pristine.
        """
        range_count, beam_count = data.shape
        lo = int(beam_count * 100 / 512)
        hi = int(beam_count * 400 / 512)
        hi = max(lo + 1, min(hi, beam_count))
        start_bin = min(10, max(0, range_count - 1))
        end_bin = max(start_bin + 1, range_count - 40)
        band = data[start_bin:end_bin, lo:hi]
        free_mask = ~np.any(band > self.paper_vertical_intensity_threshold, axis=0)
        required = max(1, int(round(
            (PAPER_GAP_BEAMS / PAPER_REFERENCE_BEAM_COUNT) * (hi - lo))))
        run = 0
        best_run = 0
        for is_free in free_mask:
            run = run + 1 if is_free else 0
            best_run = max(best_run, run)
        return best_run >= required

    def _goal_elevation_angle(self):
        if self.pose is None or self.target_z is None:
            return 0.0
        x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z
        dz = self.target_z - z
        dxy = math.hypot(self.target_x - x, self.target_y - y)
        return math.atan2(dz, max(1e-3, dxy))

    def _advance_vertical_pivot(self, data):
        """One tick of the elevation sweep. Returns (done, chosen_elevation)."""
        now = self._now_sec()
        angles = self._paper_pivot_angles()

        if self.vpivot_sample_index >= len(angles):
            run_len = max(1, self.vertical_gap_run_length)
            accepted = self.vpivot_accepted
            mids = []
            for i in range(len(accepted) - run_len + 1):
                if all(accepted[i:i + run_len]):
                    mids.append((angles[i] + angles[i + run_len - 1]) / 2.0)
            self.vpivot_active = False
            self.vpivot_sample_index = 0
            self.vpivot_accepted = []
            self.vpivot_angle_commanded_time = None
            if not mids:
                return True, None
            goal_elevation = self._goal_elevation_angle()
            chosen = min(mids, key=lambda a: abs(a - goal_elevation))
            return True, chosen

        angle = angles[self.vpivot_sample_index]
        if self.vpivot_angle_commanded_time is None:
            self.joint_pub.publish(Float64(data=float(angle)))
            self.sonar_move_pub.publish(Float64(data=2.0))
            self.vpivot_angle_commanded_time = now
            return False, None

        if now - self.vpivot_angle_commanded_time < self.pivot_sample_timeout:
            return False, None

        free = self._paper_central_sector_free(data)
        self.vpivot_accepted.append(free)
        range_count, beam_count = data.shape
        lo = int(beam_count * 100 / 512)
        hi = max(lo + 1, min(int(beam_count * 400 / 512), beam_count))
        start_bin = min(10, max(0, range_count - 1))
        end_bin = max(start_bin + 1, range_count - 40)
        band = data[start_bin:end_bin, lo:hi]
        free_mask = ~np.any(band > self.paper_vertical_intensity_threshold, axis=0)
        run = 0
        best_run = 0
        for is_free in free_mask:
            run = run + 1 if is_free else 0
            best_run = max(best_run, run)
        required = max(1, int(round(
            (PAPER_GAP_BEAMS / PAPER_REFERENCE_BEAM_COUNT) * (hi - lo))))
        self.get_logger().info(
            '[VPIVOT_DEBUG] '
            f'angle={angle:.3f} free={free} '
            f'best_run={best_run} required={required} band_width={hi - lo} '
            f'frac_above={float(np.mean(band > self.paper_vertical_intensity_threshold)):.3f}')
        self.vpivot_sample_index += 1
        self.vpivot_angle_commanded_time = None
        return False, None

    def _widest_free_run(self, data):
        """Widest contiguous corridor (in beams) visible in the current
        forward-facing 90deg sonar window, at whatever heading the vehicle
        is at right now."""
        free_beams = self._paper_classify_beams(data, self.paper_beam_stride)
        free_set = set(free_beams)
        beam_count = data.shape[1]
        stride = self.paper_beam_stride
        run = 0
        best = 0
        for i in range(0, beam_count, stride):
            if i in free_set:
                run += 1
                best = max(best, run)
            else:
                run = 0
        return best

    def _update_stuck_recovery(self, now, data):
        """Detect a fully-boxed deadlock, back away while scanning for the
        most open heading, then turn onto it before resuming navigation.

        gap_follow/boundedness_turn/convexity_turn and a completed vertical
        escape can all keep firing, each producing a nonzero commanded
        velocity, without ever moving the vehicle: when the horizontal scan
        reports free_beams=0 on every bearing (observed in World A at a
        tight obstacle cluster), velocity_cbf.py's min-norm CBF-QP has an
        obstacle-derived constraint pushing back from every direction at
        once, so whatever direction the planner tries gets projected down
        toward zero. There is no 2D or vertical-pivot decision that fixes
        this -- the corridor those maneuvers are trying to find may not
        exist within the current safety margin from the vehicle's current
        heading. Reversing straight back is the one direction the CBF's
        constraint gradient (2*(vehicle - obstacle)) always treats as safe,
        so it passes through un-throttled and actually opens real
        clearance -- but backing out and then just letting gap_follow
        re-aim at the goal re-approaches the identical corridor it was
        already stuck in (confirmed: net position unchanged minutes later
        despite real clearance gains). The sonar only ever looks at
        whatever's currently dead ahead within its fixed 90deg FOV, so
        finding out what's actually open in *other* directions requires
        physically yawing the vehicle -- there's no separate yaw-scanning
        joint the way there is for elevation. Turning while backing away
        samples the sonar across a wide arc for free, and remembering the
        heading with the widest corridor seen (not just count of free
        beams, which rewards scattered noise over one real opening) gives
        2D nav something genuinely different to try instead of repeating
        the failed approach.
        """
        if self.stuck_recovery_turn_active:
            current_yaw = (
                yaw_from_quaternion(self.pose.orientation)
                if self.pose is not None else self.stuck_recovery_best_yaw)
            yaw_error = math.atan2(
                math.sin(self.stuck_recovery_best_yaw - current_yaw),
                math.cos(self.stuck_recovery_best_yaw - current_yaw))
            if abs(yaw_error) <= 0.12 or now >= self.stuck_recovery_turn_until:
                self.stuck_recovery_turn_active = False
                self.vertical_resume_until = now + self.post_vertical_resume_duration
                self.progress3d_anchor_xyz = None
                self.progress3d_anchor_time = None
                # Remembered so that *if* this heading also turns out to be
                # a dead end (the next trigger fires), it gets blacklisted
                # below -- see the class docstring note on known_bad_headings.
                self.stuck_recovery_last_turn_yaw = self.stuck_recovery_best_yaw
                self.get_logger().info(
                    '[STUCK_RECOVERY] turned onto best heading '
                    f'(run={self.stuck_recovery_best_run} beams); resuming navigation')
                return False
            cmd = Twist()
            cmd.angular.z = clamp(1.2 * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)
            self._publish_cmd(cmd, 'stuck_recovery_turning_to_best_heading')
            return True

        if self.stuck_recovery_active:
            if self.pose is not None:
                sample_yaw = yaw_from_quaternion(self.pose.orientation)
                # A reading near the heading we were just stuck facing looks
                # clear for the wrong reason -- backing away increases
                # standoff distance from the *same* wall, which alone can
                # push it from "blocked" to "reads free" at longer range,
                # without there being an actual corridor there. Confirmed:
                # the very first post-trigger sample (heading barely
                # changed yet) reported the scan's full width as free, and
                # turning back onto it just re-approached the identical
                # spot. Only trust a candidate once the vehicle has
                # actually turned a meaningful amount away from where it
                # started, so "best" reflects a genuinely different
                # direction, not residual standoff on the old one.
                yaw_delta = abs(math.atan2(
                    math.sin(sample_yaw - self.stuck_recovery_start_yaw),
                    math.cos(sample_yaw - self.stuck_recovery_start_yaw)))
                # "Widest free run" is a proxy for "real through-passage",
                # and a bad one: a large dead-end bay is more open volume
                # than a narrow-but-genuine corridor, so it will *always*
                # out-score the real path on raw width. Confirmed: repeated
                # attempts kept re-selecting the same run=103 heading, which
                # traced (via closing-corridor detection) straight into a
                # dead end every time -- a wider sweep or more attempts
                # alone can't fix this, since picking the single global max
                # is deterministic. Once a heading is confirmed bad (below),
                # exclude it from candidacy so later attempts are forced
                # onto a different, unproven heading instead of re-deriving
                # the same wrong answer.
                run = self._widest_free_run(data)
                sample_xy = (self.pose.position.x, self.pose.position.y)
                if (yaw_delta >= 0.6 and run > self.stuck_recovery_best_run
                        and not self._is_known_bad_heading(sample_yaw, sample_xy)):
                    self.stuck_recovery_best_run = run
                    self.stuck_recovery_best_yaw = sample_yaw
            if now >= self.stuck_recovery_until:
                self.stuck_recovery_active = False
                if self.stuck_recovery_best_yaw is not None:
                    self.stuck_recovery_turn_active = True
                    self.stuck_recovery_turn_until = now + 8.0
                    self.get_logger().info(
                        '[STUCK_RECOVERY] backed away; turning onto widest '
                        f'unproven corridor seen (run={self.stuck_recovery_best_run} '
                        f'beams, {len(self.known_bad_headings)} heading(s) excluded)')
                    return True
                self.vertical_resume_until = now + self.post_vertical_resume_duration
                self.progress3d_anchor_xyz = None
                self.progress3d_anchor_time = None
                self.get_logger().info(
                    '[STUCK_RECOVERY] backing-away complete, no unproven corridor found; '
                    'resuming navigation')
                return False
            cmd = Twist()
            cmd.linear.x = -self.stuck_recovery_reverse_speed
            cmd.angular.z = self._stuck_recovery_yaw_sign() * self.stuck_recovery_yaw_bias
            self._publish_cmd(cmd, 'stuck_recovery_backing_away')
            self.get_logger().warning(
                '[STUCK_RECOVERY] backing away, remaining='
                f'{self.stuck_recovery_until - now:.1f}s best_run={self.stuck_recovery_best_run}')
            return True

        if self.pose is None:
            return False

        # Closing-corridor trap: after a back-away, the frontier scan picks
        # the heading with the widest free run *right now* -- but that only
        # measures how open it looks from the current distance, not whether
        # it stays open closer up. Confirmed via log: turning onto a
        # run=103-beam heading and following it with gap_follow, free_beams
        # shrank continuously and cleanly (103 -> 88 -> 57 -> 9 -> 0) over
        # ~15s of forward travel -- a wide-mouthed pocket funneling shut, not
        # a real through-passage. The existing no-progress timeout only
        # fires *after* it's fully boxed in again (another 45s on top of
        # this one), re-running the whole expensive recovery cycle for a
        # trap that was visible 10+ seconds earlier. Watch for the same
        # signal directly: if the widest run seen in the last
        # narrowing_trap_window seconds was large and it has since collapsed
        # to a small fraction of that, bail immediately instead of driving
        # the rest of the way into the pocket.
        stride = self.paper_beam_stride
        free_now = len(self._paper_classify_beams(data, stride))
        self.gap_width_history.append((now, free_now))
        self.gap_width_history = [
            (t, c) for t, c in self.gap_width_history
            if now - t <= self.narrowing_trap_window]
        recent_max = max((c for _, c in self.gap_width_history), default=0)
        narrowing_trap = (
            recent_max >= self.narrowing_trap_min_width and
            free_now < self.narrowing_trap_min_width and
            free_now <= recent_max * self.narrowing_trap_ratio)

        xyz = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
        if self.progress3d_anchor_xyz is None:
            self.progress3d_anchor_xyz = xyz
            self.progress3d_anchor_time = now

        moved = math.dist(xyz, self.progress3d_anchor_xyz)
        if moved >= self.stuck_recovery_distance_threshold and not narrowing_trap:
            self.progress3d_anchor_xyz = xyz
            self.progress3d_anchor_time = now
            # 2.5m of *raw* displacement resets the no-progress timer
            # correctly (it proves the vehicle isn't frozen), but it's the
            # wrong test for "safe to forget known_bad_headings": confirmed
            # via log -- the vehicle can wander 2.5m sideways along a
            # concave wall face repeatedly, clearing the bad-heading list
            # each time without ever net-advancing, letting it re-discover
            # and re-try the exact same false openings it already ruled
            # out. Only clear once distance to the actual goal has shrunk,
            # so the memory persists for as long as the vehicle is still
            # working the same local pocket, however much it wanders within
            # it.
            if (self.known_bad_headings and self.target_x is not None and
                    self.known_bad_headings_goal_anchor is not None):
                dist_to_goal = math.hypot(
                    self.target_x - xyz[0], self.target_y - xyz[1])
                if dist_to_goal <= (
                        self.known_bad_headings_goal_anchor -
                        self.bad_heading_clear_progress):
                    self.get_logger().info(
                        f'[STUCK_RECOVERY] clearing {len(self.known_bad_headings)} '
                        'bad heading(s), real progress made '
                        f'(dist_to_goal {self.known_bad_headings_goal_anchor:.1f}m '
                        f'-> {dist_to_goal:.1f}m)')
                    self.known_bad_headings = []
                    self.known_bad_headings_goal_anchor = None
            self.stuck_recovery_last_turn_yaw = None
            return False

        no_progress_timeout = now - self.progress3d_anchor_time >= self.stuck_recovery_timeout
        if not (narrowing_trap or no_progress_timeout):
            return False

        # Genuinely stuck (or about to be): abandon whatever 2D/vertical
        # maneuver was in progress and back away instead.
        self.vpivot_active = False
        self.vertical_escape_active = False
        self.joint_pub.publish(Float64(data=0.0))
        self.sonar_move_pub.publish(Float64(data=0.0))
        self.stuck_recovery_active = True
        self.stuck_recovery_until = now + self.stuck_recovery_duration
        self.stuck_recovery_count += 1
        self.stuck_recovery_best_run = -1
        self.stuck_recovery_best_yaw = None
        self.stuck_recovery_start_yaw = yaw_from_quaternion(self.pose.orientation)
        self.gap_width_history = []
        # The heading responsible for *this* trap: if we just turned onto a
        # chosen heading and drove into a dead end, that's
        # stuck_recovery_last_turn_yaw. Otherwise (first attempt in a fresh
        # pocket, nothing turned onto yet) it's wherever we're facing now.
        bad_yaw = (
            self.stuck_recovery_last_turn_yaw
            if self.stuck_recovery_last_turn_yaw is not None
            else self.stuck_recovery_start_yaw)
        if not self.known_bad_headings and self.target_x is not None:
            self.known_bad_headings_goal_anchor = math.hypot(
                self.target_x - xyz[0], self.target_y - xyz[1])
        self.known_bad_headings.append((bad_yaw, xyz[0], xyz[1]))
        self.stuck_recovery_last_turn_yaw = None
        if narrowing_trap:
            self.get_logger().warning(
                '[STUCK_RECOVERY] closing corridor detected '
                f'(free_beams {recent_max}->{free_now} in {self.narrowing_trap_window:.0f}s); '
                f'backing away (attempt {self.stuck_recovery_count}, '
                f'{len(self.known_bad_headings)} bad heading(s) known)')
        else:
            self.get_logger().warning(
                '[STUCK_RECOVERY] no net progress in '
                f'{self.stuck_recovery_timeout:.0f}s (moved {moved:.2f}m); '
                f'backing away (attempt {self.stuck_recovery_count}, '
                f'{len(self.known_bad_headings)} bad heading(s) known)')
        cmd = Twist()
        cmd.linear.x = -self.stuck_recovery_reverse_speed
        cmd.angular.z = self._stuck_recovery_yaw_sign() * self.stuck_recovery_yaw_bias
        self._publish_cmd(cmd, 'stuck_recovery_backing_away')
        return True

    def _stuck_recovery_yaw_sign(self):
        return 1.0 if self.stuck_recovery_count % 2 == 1 else -1.0

    def _is_known_bad_heading(self, yaw, position_xy):
        for bad_yaw, bad_x, bad_y in self.known_bad_headings:
            delta = abs(math.atan2(math.sin(yaw - bad_yaw), math.cos(yaw - bad_yaw)))
            if delta > self.bad_heading_tolerance:
                continue
            if math.dist(position_xy, (bad_x, bad_y)) > self.bad_heading_position_radius:
                continue
            return True
        return False

    def _has_horizontal_gap(self, data, beam_count, stride):
        """Would normal 2D nav (gap_follow) have a real corridor right now?"""
        free_beams = self._paper_classify_beams(data, stride)
        deg_per_beam = FOV_DEG / max(1, beam_count - 1)
        paper_gap_deg = (PAPER_GAP_BEAMS / PAPER_REFERENCE_BEAM_COUNT) * FOV_DEG
        required_beams = max(1, int(round(paper_gap_deg / (deg_per_beam * stride))))
        return bool(self._paper_gap_candidates(free_beams, stride, required_beams))

    def _process_data_paper(self, data, ranges, beam_count):
        now = self._now_sec()
        stride = self.paper_beam_stride

        if self._update_stuck_recovery(now, data):
            return

        if self.vertical_escape_active:
            # Originally exited only via a flat duration timer (world_a:
            # 180s), so nothing stopped it climbing indefinitely once a real
            # escape elevation was found. A "reached target depth" check was
            # added, but target_z is the *current waypoint's* depth, which
            # the vehicle is already sitting at when it gets wedged (a
            # horizontal pinch blocks it at the same depth it's trying to
            # hold, not a different one) -- so that check either fires
            # trivially on tick one (zero climb, self-cancels immediately)
            # or, once gated behind a minimum duration/distance to fix that,
            # never fires again during a real climb, since climbing moves
            # current_z monotonically *away* from target_z. Confirmed via
            # headless log capture: a single escape ran the full 180s,
            # climbing continuously from -50 to -14.7 (nearly surfacing)
            # because it had genuinely cleared the obstacle in the first
            # ~10s and then kept climbing for no reason for another 170.
            #
            # The actual paper intent (Sec III-C1 / Fig 8b) is "climb until
            # you can see a way through again", not "climb to a specific
            # depth" or "climb for a fixed duration" -- so check for that
            # directly: has a real horizontal gap reopened. That's both the
            # correct success signal and an inherent safety bound (it stops
            # climbing the moment it's no longer needed, rather than
            # over-relying on the 180s ceiling). Still requires
            # vertical_escape_min_duration first so it can't fire before any
            # real climbing has happened at all.
            elapsed_since_start = (
                now - self.vertical_escape_started_at
                if self.vertical_escape_started_at is not None else float('inf'))
            min_duration_met = elapsed_since_start >= self.vertical_escape_min_duration
            planar_progress = 0.0
            if self.pose is not None and self.vertical_escape_start_xy is not None:
                planar_progress = math.hypot(
                    self.pose.position.x - self.vertical_escape_start_xy[0],
                    self.pose.position.y - self.vertical_escape_start_xy[1])
            min_planar_met = planar_progress >= self.vertical_escape_min_planar_distance
            cleared_obstacle = (
                min_duration_met and min_planar_met and
                self._has_horizontal_gap(data, beam_count, stride))
            reached_target_depth = (
                min_duration_met and min_planar_met and
                self.pose is not None and self.target_z is not None and
                abs(self.pose.position.z - self.target_z) <= self.vertical_depth_tolerance)
            if cleared_obstacle or reached_target_depth or (
                    self.vertical_escape_until is not None and now >= self.vertical_escape_until):
                self.vertical_escape_active = False
                self.vertical_resume_until = now + self.post_vertical_resume_duration
                z_text = f'{self.pose.position.z:.2f}' if self.pose is not None else 'n/a'
                self.get_logger().info(
                    '[VERTICAL_PIVOT] escape ending: '
                    f'cleared_obstacle={cleared_obstacle} '
                    f'reached_target_depth={reached_target_depth} '
                    f'elapsed={elapsed_since_start:.1f}s z={z_text}')
            else:
                cmd = Twist()
                forward = min(0.5, self.paper_vx_max)
                cmd.linear.x = forward
                cmd.linear.z = forward * math.tan(self.vertical_escape_elevation)
                self._publish_cmd(cmd, 'vertical_escape')
                self.get_logger().info(
                    '[VERTICAL_PIVOT] '
                    f'escaping elevation={self.vertical_escape_elevation:.3f}rad '
                    f'remaining={self.vertical_escape_until - now:.1f}s')
                return

        if self.vpivot_active:
            done, chosen = self._advance_vertical_pivot(data)
            cmd = Twist()
            cmd.linear.x = self.min_forward_speed
            self._publish_cmd(cmd, 'vertical_pivot_scan')
            self.get_logger().info(
                '[VERTICAL_PIVOT] '
                f'sweeping sample={self.vpivot_sample_index}/{self.pivot_sample_count} '
                f'accepted={sum(1 for a in self.vpivot_accepted if a)}')
            if done:
                self.joint_pub.publish(Float64(data=0.0))
                self.sonar_move_pub.publish(Float64(data=0.0))
                if chosen is None:
                    self.get_logger().warning(
                        '[VERTICAL_PIVOT] no vertical gap found; resuming 2D navigation')
                else:
                    self.vertical_escape_active = True
                    self.vertical_escape_elevation = chosen
                    self.vertical_escape_until = now + self.vertical_escape_duration
                    self.vertical_escape_started_at = now
                    self.vertical_escape_start_xy = (
                        (self.pose.position.x, self.pose.position.y)
                        if self.pose is not None else None)
                    self.get_logger().info(
                        f'[VERTICAL_PIVOT] selected elevation={chosen:.3f}rad; beginning escape')
            return

        free_beams = self._paper_classify_beams(data, stride)
        target_beam = self._target_beam(beam_count)
        deg_per_beam = FOV_DEG / max(1, beam_count - 1)
        # Eq. 8-9: fixed gap cardinality L=150 beams out of the paper's
        # N_B=512-beam array (~26.4 deg). Expressed as a fraction of the FOV
        # rather than a hardcoded beam count so it stays correct at this
        # sonar model's fidelity (~500 beams) instead of the paper's 512.
        paper_gap_deg = (PAPER_GAP_BEAMS / PAPER_REFERENCE_BEAM_COUNT) * FOV_DEG
        required_beams = max(1, int(round(paper_gap_deg / (deg_per_beam * stride))))
        mid_beams = self._paper_gap_candidates(free_beams, stride, required_beams)

        # Drawn/observed the actual required route around this obstacle
        # cluster: it goes around the *outside* of the whole structure, not
        # through gaps within it. gap_follow always picks whichever
        # candidate is closest to target_beam (straight at the goal), so
        # the instant stuck-recovery turns it onto the detour heading and
        # backs off even slightly, the very next cycle it re-spots the
        # direct-line gap looking marginally open again and cuts straight
        # back into it -- never committing to the detour long enough to
        # get around. Once a heading is confirmed (via known_bad_headings)
        # to be a dead end, gap_follow itself must not re-target it either,
        # not just stuck-recovery's frontier scan, or the two fight each
        # other indefinitely.
        if mid_beams and self.known_bad_headings and self.pose is not None:
            current_yaw = yaw_from_quaternion(self.pose.orientation)
            current_xy = (self.pose.position.x, self.pose.position.y)
            mid_beams = [
                b for b in mid_beams
                if not self._is_known_bad_heading(
                    current_yaw + self._beam_to_angle(b, beam_count), current_xy)
            ]

        bcl = None
        if mid_beams:
            bcl = min(mid_beams, key=lambda b: abs(b - target_beam))

        if bcl is not None:
            cmd = self._paper_heading_command(bcl, beam_count)
            decision = 'gap_follow'
            self.no_horizontal_gap_since = None
        else:
            if self.no_horizontal_gap_since is None:
                self.no_horizontal_gap_since = now
            in_vertical_cooldown = now < self.vertical_resume_until
            # gap_follow and convexity_turn can otherwise alternate forever
            # right at a tight pinch: gap_follow commits to a marginal bcl,
            # turning toward it shrinks the corridor further, convexity_turn
            # (whose fitted curvature doesn't reliably read as convex there)
            # turns back the other way, repeat -- see paper_stuck_timeout's
            # declare_parameter comment. Once that's gone on too long, stop
            # trying more 2D turns and escalate straight to the vertical
            # pivot search (paper Fig 8b's climb-over "hump").
            stuck_too_long = (
                not in_vertical_cooldown and
                now - self.no_horizontal_gap_since >= self.paper_stuck_timeout)

            edge_beam = None
            if not in_vertical_cooldown and not stuck_too_long:
                edge_beam = self._paper_boundedness_turn(free_beams, beam_count, stride)
            if edge_beam is not None:
                cmd = self._paper_heading_command(edge_beam, beam_count)
                decision = 'boundedness_turn'
            elif stuck_too_long:
                self.vpivot_active = True
                self.vpivot_sample_index = 0
                self.vpivot_accepted = []
                self.vpivot_angle_commanded_time = None
                cmd = Twist()
                cmd.linear.x = self.min_forward_speed
                decision = 'vertical_pivot_triggered_stuck'
                self.no_horizontal_gap_since = None
            else:
                hit_points = self._paper_contour_points(data, ranges, stride)
                a, avg_left, avg_right = self._paper_convexity(hit_points)
                if a is None:
                    _, hit_ranges_full, _ = self._classify_beams(data, ranges)
                    cmd = self._command_for_scan(hit_ranges_full, target_beam, beam_count)
                    decision = 'no_contour_scan'
                elif a < self.paper_convexity_threshold and not in_vertical_cooldown:
                    self.vpivot_active = True
                    self.vpivot_sample_index = 0
                    self.vpivot_accepted = []
                    self.vpivot_angle_commanded_time = None
                    cmd = Twist()
                    cmd.linear.x = self.min_forward_speed
                    decision = 'vertical_pivot_triggered'
                    self.no_horizontal_gap_since = None
                else:
                    # Paper eq. 24-27: every SPD2C heading decision keeps
                    # moving forward (speed scaled by how far off-heading it
                    # is), it doesn't stop dead to turn in place. This branch
                    # previously hardcoded linear.x=0 with an ad-hoc yaw gain,
                    # which -- since convexity_turn fires on essentially every
                    # cycle near a convex obstacle -- meant the vehicle spent
                    # most of its time near an obstacle fully stopped instead
                    # of progressing along the "exploits the narrowing
                    # geometry" path the paper describes.
                    goal_left = self.goal_yaw_error >= 0.0
                    slope = avg_left if goal_left else avg_right
                    heading_error = clamp(
                        math.atan(slope), -self.paper_psi_max, self.paper_psi_max)
                    cmd = self._paper_heading_command_for_angle(heading_error)
                    decision = 'convexity_turn'

        self.last_selected_beam = bcl
        self._publish_cmd(cmd, decision)
        self.get_logger().info(
            '[SPD2C_PAPER] '
            f'decision={decision} bcl={bcl} target_beam={target_beam} '
            f'free_beams={len(free_beams)} required_run={required_beams} '
            f'cmd=({cmd.linear.x:.3f},{cmd.linear.y:.3f},{cmd.linear.z:.3f};{cmd.angular.z:.3f})')

    def process_data(self):
        if self.target_z is not None:
            self.target_depth_pub.publish(Float64(data=float(self.target_z)))

        source = 'raw_sonar'
        data, ranges, beam_count = self._sonar_matrix()
        raw_fresh = data is not None and self._raw_sonar_is_fresh()

        if self.paper_controller and raw_fresh:
            self._process_data_paper(data, ranges, beam_count)
            return

        if raw_fresh:
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
