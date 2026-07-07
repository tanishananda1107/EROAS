#!/usr/bin/env python3
# ROS 2 port
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Accel, Twist, Wrench
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations as tft


OBSTACLE_LINK_POSE = (0.0, 0.0, 0.0, 1.57, 3.14159, 0.0)

OBSTACLE_MODEL_BOXES = {
    'CubeSoft_5': {
        'collision_pose': (4.0, -4.886, 4.996, 0.0, 0.0, 0.0),
        'size': (8.0, 10.522, 11.9),
    },
    'CubeSoft_6': {
        'collision_pose': (4.0, -4.875, 7.944, 0.0, 0.0, 0.0),
        'size': (8.0, 15.506, 6.004),
    },
    'CubeSoft_9': {
        'collision_pose': (4.0, 2.082, 4.423, 0.0, 0.0, 0.0),
        'size': (8.0, 6.597, 6.599),
    },
    'CubeSoft_13': {
        'collision_pose': (-8.985, 0.0, 24.0, 0.0, 0.0, 0.0),
        'size': (17.97, 12.0, 48.0),
    },
}

WORLD_A_OBSTACLE_SPECS = (
    ('cube_5', 'CubeSoft_5', (10.0, 40.0, -60.0, 0.0, 1.57079633, 3.14)),
    ('cube_6', 'CubeSoft_6', (8.0, 26.0, -60.0, -2.0, 1.57079633, 0.25)),
    ('cube_7', 'CubeSoft_5', (32.0, 40.0, -60.0, 0.0, 1.57079633, 3.14)),
    ('cube_9_1', 'CubeSoft_9', (42.0, 66.0, -58.0, -0.12, 1.57079633, -0.75)),
    ('cube_9_2', 'CubeSoft_9', (12.0, 68.0, -58.0, -0.8245, 1.57079633, 0.8245)),
    ('cube_6_1', 'CubeSoft_6', (22.0, 37.0, -60.0, -1.57079633, 1.57079633, -2.15)),
    ('moving_obs', 'CubeSoft_6', (20.0, 58.0, -60.0, -2.1, 1.57079633, 2.1)),
    ('moving_obs_1', 'CubeSoft_6', (47.0, 55.0, -60.0, 2.0, 1.57, 0.0)),
    ('cube_13_1', 'CubeSoft_13', (55.0, 87.0, -54.0, 0.0, 0.0, -1.57079633)),
)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rpy_matrix(roll, pitch, yaw)
    transform[:3, 3] = np.array([x, y, z], dtype=float)
    return transform


def _build_obstacle_box(name, model_key, include_pose):
    model = OBSTACLE_MODEL_BOXES[model_key]
    transform = (
        _pose_matrix(include_pose)
        @ _pose_matrix(OBSTACLE_LINK_POSE)
        @ _pose_matrix(model['collision_pose'])
    )
    return {
        'name': name,
        'center': transform[:3, 3],
        'R': transform[:3, :3],
        'size': np.array(model['size'], dtype=float),
    }


def _build_world_a_obstacle_boxes():
    return [
        _build_obstacle_box(name, model_key, include_pose)
        for name, model_key, include_pose in WORLD_A_OBSTACLE_SPECS
    ]


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('use_point_cloud_obstacles', False)
        self.declare_parameter('use_analytical_obstacles', True)
        self.declare_parameter('planner_cmd_topic', '/rexrov2/cmd_vel_1')
        self.declare_parameter('cmd_vel_topic', '/rexrov2/cmd_vel')
        self.declare_parameter('scg_h_topic', '/rexrov2/scg/h')
        self.declare_parameter(
            'scg_gap_angle_topic', '/rexrov2/scg/selected_gap_angle')
        self.declare_parameter(
            'scg_gap_width_topic', '/rexrov2/scg/selected_gap_width')
        self.declare_parameter(
            'scg_obstacle_count_topic', '/rexrov2/scg/obstacle_count')
        self.declare_parameter('scg_gap_count_topic', '/rexrov2/scg/gap_count')
        self.declare_parameter('scg_fov_deg', 90.0)
        self.declare_parameter('scg_beam_count', 512)
        self.declare_parameter('scg_vehicle_width_m', 1.50)
        self.declare_parameter('scg_gap_safety_margin_m', 0.35)
        self.declare_parameter('scg_collision_distance', 2.2)
        self.declare_parameter('spd2c_cruise_speed', 0.45)
        self.declare_parameter('spd2c_min_forward_speed', 0.12)
        self.declare_parameter('spd2c_recovery_forward_speed', 0.18)
        self.declare_parameter('spd2c_yaw_kp', 0.85)
        self.declare_parameter('spd2c_max_yaw_rate', 0.55)
        self.declare_parameter('spd2c_lateral_gain', 0.45)
        self.declare_parameter('spd2c_max_speed_delta', 0.10)
        self.declare_parameter('spd2c_max_yaw_delta', 0.10)

        use_point_cloud_obstacles = bool(
            self.get_parameter('use_point_cloud_obstacles').value)
        self.use_point_cloud_obstacles = use_point_cloud_obstacles
        self.planner_cmd_topic = self.get_parameter('planner_cmd_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scg_h_topic = self.get_parameter('scg_h_topic').value
        self.scg_gap_angle_topic = self.get_parameter('scg_gap_angle_topic').value
        self.scg_gap_width_topic = self.get_parameter('scg_gap_width_topic').value
        self.scg_obstacle_count_topic = self.get_parameter(
            'scg_obstacle_count_topic').value
        self.scg_gap_count_topic = self.get_parameter('scg_gap_count_topic').value
        self.scg_h = float('inf')
        self.scg_selected_gap_angle = 0.0
        self.scg_selected_gap_width = 0.0
        self.scg_obstacle_count = 0
        self.scg_gap_count = 0
        self.latest_thruster_inputs = [0.0] * 6
        self.latest_pid_output = Accel()
        self.latest_thruster_wrench = Wrench()

        self.subscription_handles = [
            self.create_subscription(Twist, self.planner_cmd_topic, self.vel_callback, 10),
            self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10),
            self.create_subscription(Float64, '/rexrov2/sonar/moving', self.sonar_cb, 10),
            self.create_subscription(Accel, '/rexrov2/cmd_accel', self.pid_output_cb, 10),
            self.create_subscription(
                Wrench, '/rexrov2/thruster_manager/input',
                self.thruster_wrench_cb, 10),
            self.create_subscription(Float64, self.scg_h_topic, self.scg_h_cb, 10),
            self.create_subscription(
                Float64, self.scg_gap_angle_topic, self.scg_gap_angle_cb, 10),
            self.create_subscription(
                Float64, self.scg_gap_width_topic, self.scg_gap_width_cb, 10),
            self.create_subscription(
                Float64, self.scg_obstacle_count_topic,
                self.scg_obstacle_count_cb, 10),
            self.create_subscription(
                Float64, self.scg_gap_count_topic, self.scg_gap_count_cb, 10),
        ]
        for thruster_index in range(len(self.latest_thruster_inputs)):
            self.subscription_handles.append(
                self.create_subscription(
                    FloatStamped,
                    f'/rexrov2/thrusters/thruster_{thruster_index}/input',
                    lambda msg, index=thruster_index: self.thruster_input_cb(msg, index),
                    10))
        if use_point_cloud_obstacles:
            self.subscription_handles.extend([
                self.create_subscription(
                    PointCloud2, '/rexrov2/point_cloud',
                    self.pc_callback, qos_profile_sensor_data),
                self.create_subscription(
                    PointCloud2, '/rexrov2/blueview_p900_point_cloud',
                    self.pc_callback, qos_profile_sensor_data),
            ])

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.h_pub = self.create_publisher(Float64, '/rexrov2/current_h', 10)
        self.sonar_move_pub = self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)

        self.declare_parameter('target_depth', -60.0)
        self.declare_parameter('depth_hold_kp', 0.12)
        self.declare_parameter('depth_deadband', 0.25)
        self.declare_parameter('max_vertical_speed', 0.35)
        self.declare_parameter('max_vertical_delta', 0.05)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('cbf_influence_distance', 8.0)
        self.declare_parameter('cbf_blend_distance', 2.5)
        self.declare_parameter('cbf_gain_xy', 0.18)
        self.declare_parameter('cbf_gain_xz', 0.14)
        self.declare_parameter('max_active_constraints', 8)
        self.declare_parameter('max_xy_speed', 0.95)
        self.declare_parameter('max_xy_delta', 0.18)
        self.declare_parameter('safety_radius_xy', 1.35)
        self.declare_parameter('safety_radius_xz', 1.25)
        self.declare_parameter('safety_radius_sonar_pivot', 1.55)
        self.declare_parameter('minimum_clearance', 0.35)
        self.declare_parameter('imminent_collision_distance', 1.20)
        self.declare_parameter('collision_stop_distance', 0.55)
        self.declare_parameter('hold_depth_without_planner', True)
        self.declare_parameter('point_cloud_points_are_local', True)
        self.declare_parameter('point_cloud_timeout', 0.8)
        self.declare_parameter('point_cloud_min_range', 2.25)
        self.declare_parameter('point_cloud_body_clearance_x', 0.60)
        self.declare_parameter('point_cloud_body_clearance_y', 1.20)
        self.declare_parameter('point_cloud_body_clearance_z', 1.00)
        self.declare_parameter('spatial_memory_timeout', 7.0)
        self.declare_parameter('spatial_memory_radius', 7.0)
        self.declare_parameter('spatial_memory_voxel_size', 0.35)
        self.declare_parameter('spatial_memory_max_points', 3500)
        self.declare_parameter('min_avoid_forward_speed', 0.10)
        self.declare_parameter('max_reverse_speed', 0.0)
        self.declare_parameter('reverse_allowed_distance', 0.75)
        self.declare_parameter('planner_cmd_timeout', 1.0)
        self.declare_parameter('cbf_stall_speed_threshold', 0.06)
        self.declare_parameter('planner_influence_fov_deg', 40.0)
        self.declare_parameter('recovery_trigger_seconds', 3.0)
        self.declare_parameter('recovery_forward_speed', 0.16)
        self.declare_parameter('recovery_side_step_speed', 0.22)
        self.declare_parameter('recovery_yaw_rate', 0.35)
        self.declare_parameter('recovery_gap_fov_deg', 120.0)
        self.declare_parameter('recovery_heading_kp', 0.9)
        self.declare_parameter('recovery_heading_hold_seconds', 2.5)
        self.declare_parameter('recovery_heading_deadband', 0.08)
        self.declare_parameter('safety_escape_speed', 0.35)
        self.declare_parameter('barrier_slide_speed', 0.45)
        self.declare_parameter('barrier_slide_away_weight', 0.45)
        self.declare_parameter('barrier_slide_trigger_h', 0.80)
        self.declare_parameter('free_space_unstick_enabled', True)
        self.declare_parameter('free_space_unstick_timeout', 3.0)
        self.declare_parameter('free_space_unstick_duration', 2.5)
        self.declare_parameter('free_space_unstick_speed', 0.80)
        self.declare_parameter('hover_lock_enabled', True)
        self.declare_parameter('hover_lock_distance', 5.0)
        self.declare_parameter('hover_lock_release_distance', 6.0)
        self.declare_parameter('hover_lock_lateral_min', 0.35)
        self.declare_parameter('hover_lock_squeeze_seconds', 1.0)
        self.declare_parameter('hover_lock_min_seconds', 4.0)
        self.declare_parameter('hover_lock_cooldown_seconds', 1.0)
        self.declare_parameter('hover_lock_oscillation_threshold', 2.0)
        self.declare_parameter('hover_lock_oscillation_decay', 0.82)
        self.declare_parameter('hover_lock_min_lateral_cmd', 0.05)
        self.declare_parameter('hover_lock_min_yaw_cmd', 0.08)

        self.safety_radius_xy = float(
            self.get_parameter('safety_radius_xy').value)
        self.safety_radius_xz = float(
            self.get_parameter('safety_radius_xz').value)
        self.safety_radius_sonar_pivot = float(
            self.get_parameter('safety_radius_sonar_pivot').value)
        self.minimum_clearance = float(
            self.get_parameter('minimum_clearance').value)
        self.imminent_collision_distance = float(
            self.get_parameter('imminent_collision_distance').value)
        self.collision_stop_distance = float(
            self.get_parameter('collision_stop_distance').value)
        self.emergency_collision_distance = self.collision_stop_distance
        self.hold_depth_without_planner = bool(
            self.get_parameter('hold_depth_without_planner').value)
        self.R_o     = self.safety_radius_xy
        self.radius  = 15.0
        self.kappa   = float(self.get_parameter('cbf_gain_xy').value)
        self.kappa1  = float(self.get_parameter('cbf_gain_xz').value)
        self.use_analytical_obstacles = bool(
            self.get_parameter('use_analytical_obstacles').value)
        self.analytical_obstacle_boxes = _build_world_a_obstacle_boxes()
        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_hold_kp = float(self.get_parameter('depth_hold_kp').value)
        self.depth_deadband = float(self.get_parameter('depth_deadband').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.max_vertical_delta = float(self.get_parameter('max_vertical_delta').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.cbf_influence_distance = float(
            self.get_parameter('cbf_influence_distance').value)
        self.cbf_blend_distance = float(
            self.get_parameter('cbf_blend_distance').value)
        self.max_active_constraints = int(
            self.get_parameter('max_active_constraints').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_xy_delta = float(self.get_parameter('max_xy_delta').value)
        self.point_cloud_points_are_local = bool(
            self.get_parameter('point_cloud_points_are_local').value)
        self.point_cloud_timeout = float(
            self.get_parameter('point_cloud_timeout').value)
        self.point_cloud_min_range = float(
            self.get_parameter('point_cloud_min_range').value)
        self.point_cloud_body_clearance_x = float(
            self.get_parameter('point_cloud_body_clearance_x').value)
        self.point_cloud_body_clearance_y = float(
            self.get_parameter('point_cloud_body_clearance_y').value)
        self.point_cloud_body_clearance_z = float(
            self.get_parameter('point_cloud_body_clearance_z').value)
        self.spatial_memory_timeout = float(
            self.get_parameter('spatial_memory_timeout').value)
        self.spatial_memory_radius = float(
            self.get_parameter('spatial_memory_radius').value)
        self.spatial_memory_voxel_size = float(
            self.get_parameter('spatial_memory_voxel_size').value)
        self.spatial_memory_max_points = int(
            self.get_parameter('spatial_memory_max_points').value)
        self.min_avoid_forward_speed = float(
            self.get_parameter('min_avoid_forward_speed').value)
        self.max_reverse_speed = float(
            self.get_parameter('max_reverse_speed').value)
        self.reverse_allowed_distance = float(
            self.get_parameter('reverse_allowed_distance').value)
        self.safety_radius_adjustment = 0.0
        self.planner_cmd_timeout = float(
            self.get_parameter('planner_cmd_timeout').value)
        self.cbf_stall_speed_threshold = float(
            self.get_parameter('cbf_stall_speed_threshold').value)
        self.planner_influence_fov = np.deg2rad(float(
            self.get_parameter('planner_influence_fov_deg').value))
        self.recovery_trigger_seconds = float(
            self.get_parameter('recovery_trigger_seconds').value)
        self.recovery_forward_speed = float(
            self.get_parameter('recovery_forward_speed').value)
        self.recovery_side_step_speed = float(
            self.get_parameter('recovery_side_step_speed').value)
        self.recovery_yaw_rate = float(
            self.get_parameter('recovery_yaw_rate').value)
        self.recovery_gap_fov = np.deg2rad(float(
            self.get_parameter('recovery_gap_fov_deg').value))
        self.recovery_heading_kp = float(
            self.get_parameter('recovery_heading_kp').value)
        self.recovery_heading_hold_seconds = float(
            self.get_parameter('recovery_heading_hold_seconds').value)
        self.recovery_heading_deadband = float(
            self.get_parameter('recovery_heading_deadband').value)
        self.safety_escape_speed = float(
            self.get_parameter('safety_escape_speed').value)
        self.barrier_slide_speed = float(
            self.get_parameter('barrier_slide_speed').value)
        self.barrier_slide_away_weight = float(
            self.get_parameter('barrier_slide_away_weight').value)
        self.barrier_slide_trigger_h = float(
            self.get_parameter('barrier_slide_trigger_h').value)
        self.free_space_unstick_enabled = bool(
            self.get_parameter('free_space_unstick_enabled').value)
        self.free_space_unstick_timeout = float(
            self.get_parameter('free_space_unstick_timeout').value)
        self.free_space_unstick_duration = float(
            self.get_parameter('free_space_unstick_duration').value)
        self.free_space_unstick_speed = float(
            self.get_parameter('free_space_unstick_speed').value)
        self.hover_lock_enabled = bool(
            self.get_parameter('hover_lock_enabled').value)
        self.hover_lock_distance = float(
            self.get_parameter('hover_lock_distance').value)
        self.hover_lock_release_distance = float(
            self.get_parameter('hover_lock_release_distance').value)
        self.hover_lock_lateral_min = float(
            self.get_parameter('hover_lock_lateral_min').value)
        self.hover_lock_squeeze_seconds = float(
            self.get_parameter('hover_lock_squeeze_seconds').value)
        self.hover_lock_min_seconds = float(
            self.get_parameter('hover_lock_min_seconds').value)
        self.hover_lock_cooldown_seconds = float(
            self.get_parameter('hover_lock_cooldown_seconds').value)
        self.hover_lock_oscillation_threshold = float(
            self.get_parameter('hover_lock_oscillation_threshold').value)
        self.hover_lock_oscillation_decay = float(
            self.get_parameter('hover_lock_oscillation_decay').value)
        self.hover_lock_min_lateral_cmd = float(
            self.get_parameter('hover_lock_min_lateral_cmd').value)
        self.hover_lock_min_yaw_cmd = float(
            self.get_parameter('hover_lock_min_yaw_cmd').value)
        self.local_scg_fov = np.deg2rad(float(
            self.get_parameter('scg_fov_deg').value))
        self.local_scg_beam_count = max(
            16, int(self.get_parameter('scg_beam_count').value))
        self.local_scg_vehicle_width = float(
            self.get_parameter('scg_vehicle_width_m').value)
        self.local_scg_gap_safety_margin = float(
            self.get_parameter('scg_gap_safety_margin_m').value)
        self.local_scg_min_gap_width = (
            self.local_scg_vehicle_width +
            2.0 * self.local_scg_gap_safety_margin)
        self.local_scg_collision_distance = float(
            self.get_parameter('scg_collision_distance').value)
        self.spd2c_cruise_speed = float(
            self.get_parameter('spd2c_cruise_speed').value)
        self.spd2c_min_forward_speed = float(
            self.get_parameter('spd2c_min_forward_speed').value)
        self.spd2c_recovery_forward_speed = float(
            self.get_parameter('spd2c_recovery_forward_speed').value)
        self.spd2c_yaw_kp = float(
            self.get_parameter('spd2c_yaw_kp').value)
        self.spd2c_max_yaw_rate = float(
            self.get_parameter('spd2c_max_yaw_rate').value)
        self.spd2c_lateral_gain = float(
            self.get_parameter('spd2c_lateral_gain').value)
        self.spd2c_max_speed_delta = float(
            self.get_parameter('spd2c_max_speed_delta').value)
        self.spd2c_max_yaw_delta = float(
            self.get_parameter('spd2c_max_yaw_delta').value)

        self.filtered_points = np.empty((0,3))
        self.vehicle_pose    = None
        self.quaternion      = None
        self.yaw             = 0.0
        self.current_planar_speed = 0.0
        self.current_h       = float('inf')
        self.closest_obstacle_distance = float('inf')
        self.closest_point   = None
        self.closest_source  = 'none'
        self.obstacle_constraints = []
        self.v_alg           = Twist()
        self.latest_cmd_time = None
        self.last_safe_xy = np.zeros(2)
        self.last_vertical_cmd = 0.0
        self.xy_cbf = True
        self.xz_cbf = self.sonar_moving = False
        self.latest_pose_msg = None
        self.latest_pose_time = None
        self.last_pose_stamp = None
        self.latest_pc_msg = None
        self.latest_pc_time = None
        self.last_pc_process_time = None
        self.last_pc_stamp = None
        self.spatial_memory_points = np.empty((0, 3))
        self.spatial_memory_times = np.empty((0,))
        self.pc_process_period = 0.25
        self.max_point_samples = 5000
        self.rejected_motion_since = None
        self.cbf_recovery_mode = False
        self.last_recovery_heading = 0.0
        self.recovery_heading_hold_until_sec = 0.0
        self.free_space_low_speed_since_sec = 0.0
        self.free_space_unstick_until_sec = 0.0
        self.last_cbf_action = 'startup'
        self.hover_lock_until_sec = 0.0
        self.hover_lock_cooldown_until_sec = 0.0
        self.hover_oscillation_score = 0.0
        self.last_hover_lateral_sign = 0.0
        self.last_hover_yaw_sign = 0.0
        self.hover_squeeze_started_sec = 0.0
        self.hover_lock_reason = 'none'
        self.local_scg_context_h = float('inf')
        self.local_scg_obstacle_count = 0
        self.local_scg_gap_count = 0
        self.local_scg_selected_gap = None
        self.local_scg_nearest_obstacle = float('inf')
        self.local_scg_gap_widths = []
        self.local_scg_obstacle_boundaries = []
        self.local_scg_free_sector_count = 0
        self.local_scg_selected_gap_angle = 0.0
        self.local_scg_selected_gap_width = 0.0
        self.local_scg_boundedness = 'NONE'
        self.local_scg_traversable = False
        self.local_scg_boundedness_reason = 'startup'
        self.local_scg_convergence = 'STABLE'
        self.local_scg_convergence_reason = 'startup'
        self.local_scg_gap_history = {}
        self.last_spd2c_cmd = Twist()
        self.last_spd2c_decision = 'startup'
        self.final_velocity_near_zero_since_sec = 0.0
        self.final_velocity_recovery_active = False
        self.final_velocity_recovery_elapsed = 0.0
        self.final_velocity_recovery_heading = 0.0
        self.final_velocity_recovery_forward_safe = False
        self.final_velocity_recovery_scan_requested = False
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop)

    # ---- state callbacks ----
    def sonar_cb(self, msg):
        s = msg.data
        self.xy_cbf = self.xz_cbf = self.sonar_moving = False
        self.R_o = self.safety_radius_xy
        if   s == 1:
            self.xz_cbf = True
            self.R_o = self.safety_radius_xz
        elif s == 2:
            # Sonar pivoting refines the estimate, but it must not interrupt
            # horizontal obstacle avoidance or stop command publication.
            self.xy_cbf = True
            self.sonar_moving = True
            self.R_o = self.safety_radius_sonar_pivot
        elif s == 0:
            self.xy_cbf = True
            self.R_o = self.safety_radius_xy

    def scg_h_cb(self, msg):
        self.scg_h = float(msg.data)

    def scg_gap_angle_cb(self, msg):
        self.scg_selected_gap_angle = float(msg.data)

    def scg_gap_width_cb(self, msg):
        self.scg_selected_gap_width = float(msg.data)

    def scg_obstacle_count_cb(self, msg):
        self.scg_obstacle_count = int(round(msg.data))

    def scg_gap_count_cb(self, msg):
        self.scg_gap_count = int(round(msg.data))

    def thruster_input_cb(self, msg, index):
        if 0 <= index < len(self.latest_thruster_inputs):
            self.latest_thruster_inputs[index] = float(msg.data)

    def pid_output_cb(self, msg):
        self.latest_pid_output = msg

    def thruster_wrench_cb(self, msg):
        self.latest_thruster_wrench = msg

    def vel_callback(self, msg):
        self.v_alg = msg
        self.latest_cmd_time = self.get_clock().now()

    def pc_callback(self, msg):
        self.latest_pc_msg = msg
        self.latest_pc_time = self.get_clock().now()

    def _pc_stamp_key(self, msg):
        stamp = msg.header.stamp
        return stamp.sec, stamp.nanosec

    def _stamp_age(self, stamp):
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            return 0.0
        now_sec = self.get_clock().now().nanoseconds / 1e9
        return now_sec - stamp_sec

    def _point_cloud_stamp_is_fresh(self):
        return self.latest_pc_msg is not None

    def _planner_cmd_is_fresh(self):
        if self.latest_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        return age <= self.planner_cmd_timeout

    def _log_planner_cmd(self):
        if self.v_alg is None or self.latest_cmd_time is None:
            return
        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        self.get_logger().info(
            '[PLANNER_CMD] '
            f'planner_topic={self.planner_cmd_topic} age={age:.3f}s '
            f'raw_linear=({self.v_alg.linear.x:.3f},{self.v_alg.linear.y:.3f},{self.v_alg.linear.z:.3f}) '
            f'raw_angular_z={self.v_alg.angular.z:.3f}')

    def _update_point_cloud_obstacles(self):
        if self.vehicle_pose is None or self.latest_pc_msg is None:
            return
        if not self._point_cloud_stamp_is_fresh():
            self.filtered_points = np.empty((0, 3))
            return

        now = self.get_clock().now()
        stamp_key = self._pc_stamp_key(self.latest_pc_msg)
        if stamp_key == self.last_pc_stamp:
            return

        if self.last_pc_process_time is not None:
            elapsed = (now - self.last_pc_process_time).nanoseconds / 1e9
            if elapsed < self.pc_process_period:
                return

        self.last_pc_process_time = now
        self.last_pc_stamp = stamp_key

        try:
            pts = pc2.read_points_numpy(
                self.latest_pc_msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
                reshape_organized_cloud=False)
        except Exception as exc:
            self.get_logger().warning(f'[NAVIGATION_DECISION] Failed to parse CBF point cloud: {exc}')
            return

        pts = np.asarray(pts, dtype=float)
        if pts.size == 0:
            self.filtered_points = np.empty((0, 3))
            return

        pts = pts.reshape((-1, 3))
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if pts.size == 0:
            self.filtered_points = np.empty((0, 3))
            return

        if self.point_cloud_points_are_local:
            body_mask = np.logical_and.reduce((
                np.abs(pts[:, 0]) <= self.point_cloud_body_clearance_x,
                np.abs(pts[:, 1]) <= self.point_cloud_body_clearance_y,
                np.abs(pts[:, 2]) <= self.point_cloud_body_clearance_z,
            ))
            pts = pts[np.logical_not(body_mask)]
            if pts.size == 0:
                self.filtered_points = np.empty((0, 3))
                return

        ranges = np.linalg.norm(pts, axis=1)
        pts = pts[np.logical_and(
            ranges >= self.point_cloud_min_range,
            ranges <= self.radius)]
        if pts.size == 0:
            self.filtered_points = np.empty((0, 3))
            return

        if len(pts) > self.max_point_samples:
            step = int(np.ceil(len(pts) / self.max_point_samples))
            pts = pts[::step]

        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        if self.point_cloud_points_are_local:
            pts = (self._R() @ pts.T).T + vp

        pts = np.round(pts / 0.25) * 0.25
        d = np.linalg.norm(pts - vp, axis=1)
        pts = pts[np.logical_and(d >= self.point_cloud_min_range, d <= self.radius)]
        if len(pts):
            self.filtered_points = np.unique(pts, axis=0)
            self._update_spatial_memory(self.filtered_points)
        else:
            self.filtered_points = np.empty((0, 3))
            self._prune_spatial_memory()

    def _analytical_obstacles(self):
        return self.analytical_obstacle_boxes

    def _analytical_constraints(self, vp):
        constraints = []
        if not self.use_analytical_obstacles:
            return constraints

        for obs in self._analytical_obstacles():
            local_vp = obs['R'].T @ (vp - obs['center'])
            half_size = obs['size'] / 2.0
            local_closest = np.clip(local_vp, -half_size, half_size)
            world_closest = obs['R'] @ local_closest + obs['center']
            dist = np.linalg.norm(vp - world_closest)
            constraints.append({
                'source': obs['name'],
                'point': world_closest,
                'distance': float(dist),
            })

        return constraints

    def _spatial_memory_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_spatial_memory(self):
        if len(self.spatial_memory_points) == 0:
            return

        now = self._spatial_memory_now()
        ages = now - self.spatial_memory_times
        keep = ages <= self.spatial_memory_timeout

        if self.vehicle_pose is not None:
            vp = np.array([
                self.vehicle_pose.x,
                self.vehicle_pose.y,
                self.vehicle_pose.z,
            ], dtype=float)
            dists = np.linalg.norm(self.spatial_memory_points - vp, axis=1)
            keep = np.logical_and(keep, dists <= self.spatial_memory_radius)

        self.spatial_memory_points = self.spatial_memory_points[keep]
        self.spatial_memory_times = self.spatial_memory_times[keep]

    def _update_spatial_memory(self, points):
        if self.spatial_memory_timeout <= 0.0 or len(points) == 0:
            return

        self._prune_spatial_memory()
        now = self._spatial_memory_now()
        points = np.asarray(points, dtype=float).reshape((-1, 3))
        if self.spatial_memory_voxel_size > 0.0:
            points = np.round(points / self.spatial_memory_voxel_size) * self.spatial_memory_voxel_size

        if len(self.spatial_memory_points):
            all_points = np.vstack((self.spatial_memory_points, points))
            all_times = np.concatenate((
                self.spatial_memory_times,
                np.full(len(points), now),
            ))
        else:
            all_points = points
            all_times = np.full(len(points), now)

        if self.spatial_memory_voxel_size > 0.0 and len(all_points):
            order = np.argsort(all_times)
            all_points = all_points[order]
            all_times = all_times[order]
            keys = np.round(all_points / self.spatial_memory_voxel_size).astype(np.int64)
            _, reverse_keep = np.unique(keys[::-1], axis=0, return_index=True)
            keep_indices = len(keys) - 1 - reverse_keep
            all_points = all_points[keep_indices]
            all_times = all_times[keep_indices]

        if len(all_points) > self.spatial_memory_max_points:
            order = np.argsort(all_times)
            keep = order[-self.spatial_memory_max_points:]
            all_points = all_points[keep]
            all_times = all_times[keep]

        self.spatial_memory_points = all_points
        self.spatial_memory_times = all_times

    def _spatial_memory_has_points(self):
        self._prune_spatial_memory()
        return len(self.spatial_memory_points) > 0

    def _point_cloud_candidate_points(self):
        clouds = []
        if self._point_cloud_is_fresh() and len(self.filtered_points):
            clouds.append(np.asarray(self.filtered_points, dtype=float))

        if self._spatial_memory_has_points():
            clouds.append(np.asarray(self.spatial_memory_points, dtype=float))

        if not clouds:
            return np.empty((0, 3))

        points = np.vstack(clouds)
        if self.vehicle_pose is not None:
            vp = np.array([
                self.vehicle_pose.x,
                self.vehicle_pose.y,
                self.vehicle_pose.z,
            ], dtype=float)
            dists = np.linalg.norm(points - vp, axis=1)
            points = points[np.logical_and(
                dists >= self.point_cloud_min_range,
                dists <= self.radius)]

        if len(points) == 0:
            return np.empty((0, 3))

        return np.unique(points, axis=0)

    def _point_cloud_constraints(self, vp):
        points = self._point_cloud_candidate_points()
        if len(points) == 0:
            return []

        vectors = points - vp
        dists = np.linalg.norm(vectors, axis=1)
        if dists.size == 0:
            return []

        order = np.argsort(dists)
        bearing_bin_width = np.deg2rad(8.0)
        selected = []
        used_bins = set()
        for index in order:
            if dists[index] > self.radius:
                continue

            bearing = np.arctan2(vectors[index, 1], vectors[index, 0])
            bearing_bin = int(np.round(bearing / bearing_bin_width))
            if bearing_bin in used_bins:
                continue

            used_bins.add(bearing_bin)
            selected.append({
                'source': f'point_cloud_bin_{bearing_bin}',
                'point': points[index],
                'distance': float(dists[index]),
            })
            if len(selected) >= self.max_active_constraints * 3:
                break

        return selected

    def _point_cloud_is_fresh(self):
        if not self.use_point_cloud_obstacles or self.latest_pc_time is None:
            return False

        age = (self.get_clock().now() - self.latest_pc_time).nanoseconds / 1e9
        return age <= self.point_cloud_timeout and self._point_cloud_stamp_is_fresh()

    def _count_mask_runs(self, mask):
        count = 0
        in_run = False
        for value in mask:
            if value and not in_run:
                count += 1
                in_run = True
            elif not value:
                in_run = False
        return count

    def _scg_constraints(self, vp):
        """Build CBF constraints from FLS-based SCG data (only_gap.py output).
        
        When scg_obstacle_count > 0, we have obstacles detected by sonar.
        Convert the SCG gap information into constraint points at the
        obstacle boundaries near the selected gap.
        """
        constraints = []
        
        if self.scg_obstacle_count == 0 or not np.isfinite(self.scg_h):
            return constraints
        
        # SCG h value is the CBF barrier function from only_gap.py
        # If h >= 0, no constraint violation. If h < 0, obstacle too close.
        # Map scg_h to a pseudo-distance for constraint building.
        scg_distance = np.sqrt(self.scg_h + self.R_o**2) if self.scg_h >= -self.R_o**2 else self.R_o
        
        # If too far away, don't add constraint
        if scg_distance > self.radius:
            return constraints
        
        # Create a synthetic constraint point in the direction of the
        # selected gap angle (where the obstacle is relative to us)
        # The gap_angle points to the free space; obstacle is perpendicular.
        obstacle_angle = self.scg_selected_gap_angle + np.pi / 2.0
        
        # Generate 2-3 constraint points around the obstacle direction
        # to properly blockade this direction
        for angle_offset in [-0.3, 0.0, 0.3]:
            angle = obstacle_angle + angle_offset
            cp_xy = vp[:2] + scg_distance * np.array([np.cos(angle), np.sin(angle)])
            cp_z = vp[2]  # Keep obstacle at vehicle Z for XY plane constraint
            
            constraints.append({
                'source': 'scg_fls',
                'point': np.array([cp_xy[0], cp_xy[1], cp_z]),
                'distance': scg_distance,
            })
        
        return constraints

    def _scg_beam_to_angle(self, beam):
        center = (self.local_scg_beam_count - 1) / 2.0
        return (beam - center) * (
            self.local_scg_fov / max(1, self.local_scg_beam_count - 1))

    def _format_scg_list(self, values, unit='', limit=6):
        if not values:
            return 'none'
        parts = [f'{value:.2f}{unit}' for value in values[:limit]]
        if len(values) > limit:
            parts.append(f'+{len(values) - limit}more')
        return '[' + ','.join(parts) + ']'

    def _format_scg_boundaries(self, boundaries, limit=5):
        if not boundaries:
            return 'none'
        parts = [
            f'({start:.2f},{end:.2f},{nearest:.2f}m)'
            for start, end, nearest in boundaries[:limit]
        ]
        if len(boundaries) > limit:
            parts.append(f'+{len(boundaries) - limit}more')
        return '[' + ','.join(parts) + ']'

    def _format_scg_rejections(self, rejected, limit=4):
        if not rejected:
            return 'none'
        parts = [
            f'{item["boundedness"]}/{item["convergence"]}:'
            f'{item["width_m"]:.2f}m:{item["reason"]}'
            for item in rejected[:limit]
        ]
        if len(rejected) > limit:
            parts.append(f'+{len(rejected) - limit}more')
        return '[' + ','.join(parts) + ']'

    def _scg_local_points(self):
        point_sets = []
        source_parts = []

        if self._point_cloud_is_fresh() and len(self.filtered_points):
            point_sets.append(np.asarray(self.filtered_points, dtype=float))
            source_parts.append('point_cloud')
        if len(self.spatial_memory_points):
            point_sets.append(np.asarray(self.spatial_memory_points, dtype=float))
            source_parts.append('spatial_memory')
        if not point_sets and self.obstacle_constraints:
            point_sets.append(np.asarray([
                obstacle['point'] for obstacle in self.obstacle_constraints
            ], dtype=float))
            source_parts.append('active_constraints')

        if not point_sets:
            return np.empty((0, 3)), 'none'

        points = np.vstack(point_sets)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if points.size == 0:
            return np.empty((0, 3)), 'none'

        vp = np.array([
            self.vehicle_pose.x,
            self.vehicle_pose.y,
            self.vehicle_pose.z,
        ], dtype=float)
        local_points = (self._R().T @ (points - vp).T).T
        planar_ranges = np.linalg.norm(local_points[:, :2], axis=1)
        bearings = np.arctan2(local_points[:, 1], local_points[:, 0])
        half_fov = self.local_scg_fov / 2.0
        valid = np.logical_and.reduce((
            np.isfinite(planar_ranges),
            planar_ranges > 0.05,
            planar_ranges <= self.radius,
            local_points[:, 0] > 0.05,
            np.abs(bearings) <= half_fov,
        ))
        return local_points[valid], '+'.join(source_parts)

    def _scg_profile_from_points(self, local_points):
        beam_count = self.local_scg_beam_count
        hit_ranges = np.full(beam_count, self.radius, dtype=float)
        has_hit = np.zeros(beam_count, dtype=bool)
        if local_points.size == 0:
            return np.logical_not(has_hit), hit_ranges, has_hit

        planar_ranges = np.linalg.norm(local_points[:, :2], axis=1)
        bearings = np.arctan2(local_points[:, 1], local_points[:, 0])
        half_fov = self.local_scg_fov / 2.0
        beam_scale = (beam_count - 1) / max(self.local_scg_fov, 1e-6)
        inflated_radius = max(0.05, self.R_o + self.minimum_clearance)

        for bearing, distance in zip(bearings, planar_ranges):
            if not np.isfinite(distance) or distance <= 0.05:
                continue
            beam = int(round((bearing + half_fov) * beam_scale))
            beam = int(np.clip(beam, 0, beam_count - 1))
            half_width = float(np.clip(
                math.atan2(inflated_radius, max(distance, 1e-3)),
                np.deg2rad(2.0),
                np.deg2rad(22.0)))
            beam_radius = max(1, int(math.ceil(half_width * beam_scale)))
            start = max(0, beam - beam_radius)
            end = min(beam_count - 1, beam + beam_radius)
            has_hit[start:end + 1] = True
            hit_ranges[start:end + 1] = np.minimum(
                hit_ranges[start:end + 1], distance)

        return np.logical_not(has_hit), hit_ranges, has_hit

    def _scg_gaps(self, free_mask, hit_ranges):
        gaps = []
        beam_count = len(free_mask)
        min_width = max(
            2,
            int(math.ceil(np.deg2rad(7.5) /
                          (self.local_scg_fov / max(1, beam_count - 1)))))
        start = None

        for index, is_free in enumerate(np.append(free_mask, False)):
            if is_free and start is None:
                start = index
            elif not is_free and start is not None:
                end = index - 1
                width = end - start + 1
                if width >= min_width:
                    mid = (start + end) // 2
                    width_rad = width * (
                        self.local_scg_fov / max(1, beam_count - 1))
                    center_margin = max(1, min(width // 3, min_width // 2))
                    center_start = max(start, mid - center_margin)
                    center_end = min(end, mid + center_margin)
                    center_clearance = float(
                        np.min(hit_ranges[center_start:center_end + 1]))
                    width_range = (
                        center_clearance
                        if np.isfinite(center_clearance) and center_clearance > 0.0
                        else self.radius
                    )
                    gaps.append({
                        'start': start,
                        'end': end,
                        'mid': mid,
                        'width_m': float(
                            2.0 * width_range *
                            math.tan(max(1e-6, 0.5 * width_rad))),
                        'width_deg': float(np.rad2deg(width_rad)),
                        'center_clearance': center_clearance,
                        'touches_left': start == 0,
                        'touches_right': end == beam_count - 1,
                    })
                start = None

        return gaps

    def _scg_gap_boundedness(self, gap):
        if gap is None:
            return 'NONE', False, 'no_gap'

        if gap['touches_left'] and gap['touches_right']:
            boundedness = 'OPEN'
        elif gap['touches_left']:
            boundedness = 'RIGHT_BOUNDED'
        elif gap['touches_right']:
            boundedness = 'LEFT_BOUNDED'
        else:
            boundedness = 'BOTH_BOUNDED'

        width_safe = gap['width_m'] >= self.local_scg_min_gap_width
        clearance_safe = (
            np.isfinite(gap['center_clearance']) and
            gap['center_clearance'] > self.local_scg_collision_distance)
        safe = bool(width_safe and clearance_safe)
        if safe:
            reason = 'safe_width_and_clearance'
        elif not width_safe:
            reason = (
                f'unsafe_width={gap["width_m"]:.2f}m'
                f'<required={self.local_scg_min_gap_width:.2f}m')
        else:
            reason = (
                f'unsafe_clearance={gap["center_clearance"]:.2f}m'
                f'<=required={self.local_scg_collision_distance:.2f}m')
        return boundedness, safe, reason

    def _scg_ranked_gaps(self, gaps):
        center = (self.local_scg_beam_count - 1) / 2.0

        def score(gap):
            center_offset = abs(gap['mid'] - center) / max(1.0, center)
            clearance = (
                gap['center_clearance']
                if np.isfinite(gap['center_clearance']) else self.radius)
            width_score = gap['width_m'] / max(self.local_scg_min_gap_width, 1e-6)
            clearance_score = clearance / max(self.local_scg_collision_distance, 1e-6)
            return 1.8 * width_score + 0.7 * clearance_score - 0.4 * center_offset

        return sorted(gaps, key=score, reverse=True)

    def _scg_gap_history_key(self, gap):
        center_angle = self._scg_beam_to_angle(gap['mid'])
        return int(round(center_angle / np.deg2rad(5.0)))

    def _prune_scg_gap_history(self):
        now = self._now_sec()
        self.local_scg_gap_history = {
            key: value for key, value in self.local_scg_gap_history.items()
            if now - value['time'] <= 5.0
        }

    def _scg_convergence_analysis(self, gap, nearest_obstacle):
        if gap is None:
            return 'NARROWING_UNSAFE', False, 'no_gap'

        self._prune_scg_gap_history()
        now = self._now_sec()
        center_angle = self._scg_beam_to_angle(gap['mid'])
        key = self._scg_gap_history_key(gap)
        previous = self.local_scg_gap_history.get(key)

        if previous is None:
            classification = 'STABLE'
            safe = True
            reason = 'new_gap_history'
        else:
            width_delta = gap['width_m'] - previous['width_m']
            angle_delta = abs(center_angle - previous['center_angle'])
            prev_nearest = previous['nearest_obstacle']
            if np.isfinite(nearest_obstacle) and np.isfinite(prev_nearest):
                nearest_delta = nearest_obstacle - prev_nearest
            else:
                nearest_delta = 0.0

            width_closing = (
                width_delta < -0.25 or
                gap['width_m'] < previous['width_m'] * 0.85)
            obstacle_closing = nearest_delta < -0.35
            width_margin = gap['width_m'] - self.local_scg_min_gap_width
            clearance_margin = (
                gap['center_clearance'] - self.local_scg_collision_distance)
            margin_low = width_margin < 0.35 or clearance_margin < 0.80
            closing_near_vehicle = (
                np.isfinite(nearest_obstacle) and
                nearest_obstacle < self.local_scg_collision_distance + 0.80)
            unsafe = (
                width_closing and margin_low or
                obstacle_closing and closing_near_vehicle or
                width_closing and obstacle_closing)

            if unsafe:
                classification = 'NARROWING_UNSAFE'
                safe = False
            elif width_delta > 0.20 or nearest_delta > 0.35:
                classification = 'WIDENING'
                safe = True
            else:
                classification = 'STABLE'
                safe = True

            reason = (
                f'width_delta={width_delta:.2f}m '
                f'angle_delta={angle_delta:.2f}rad '
                f'nearest_delta={nearest_delta:.2f}m '
                f'width_margin={width_margin:.2f}m '
                f'clearance_margin={clearance_margin:.2f}m')

        self.local_scg_gap_history[key] = {
            'time': now,
            'width_m': gap['width_m'],
            'center_angle': center_angle,
            'nearest_obstacle': nearest_obstacle,
        }
        return classification, safe, reason

    def _scg_select_gap_after_convergence(self, gaps, nearest_obstacle):
        ranked_gaps = self._scg_ranked_gaps(gaps)
        rejected = []

        for gap in ranked_gaps:
            boundedness, safe, reason = self._scg_gap_boundedness(gap)
            convergence, convergence_safe, convergence_reason = (
                self._scg_convergence_analysis(gap, nearest_obstacle))
            if safe and convergence_safe:
                return (
                    gap, boundedness, True, reason,
                    convergence, True, convergence_reason, rejected)
            reject_reason = reason if not safe else convergence_reason
            rejected.append({
                'boundedness': boundedness,
                'convergence': convergence,
                'width_m': gap['width_m'],
                'reason': reject_reason,
            })

        if ranked_gaps:
            return (
                None, 'NONE', False, 'no_valid_gap',
                'NARROWING_UNSAFE', False, 'all_ranked_gaps_rejected',
                rejected)
        return (
            None, 'NONE', False, 'no_gap',
            'NARROWING_UNSAFE', False, 'no_gap',
            rejected)

    def _scg_obstacle_boundaries(self, has_hit, hit_ranges):
        boundaries = []
        start = None
        for index, blocked in enumerate(np.append(has_hit, False)):
            if blocked and start is None:
                start = index
            elif not blocked and start is not None:
                end = index - 1
                boundaries.append((
                    self._scg_beam_to_angle(start),
                    self._scg_beam_to_angle(end),
                    float(np.min(hit_ranges[start:end + 1])),
                ))
                start = None
        return boundaries

    def _update_scg_stage(self):
        local_points, source = self._scg_local_points()
        free_mask, hit_ranges, has_hit = self._scg_profile_from_points(local_points)
        gaps = self._scg_gaps(free_mask, hit_ranges)
        boundaries = self._scg_obstacle_boundaries(has_hit, hit_ranges)

        obstacle_count = self._count_mask_runs(has_hit)
        free_sector_count = self._count_mask_runs(free_mask)
        nearest = (
            float(np.min(hit_ranges[has_hit]))
            if np.any(has_hit) else float('inf')
        )
        gap_widths = [gap['width_m'] for gap in gaps]
        (
            selected_gap,
            boundedness,
            traversable,
            boundedness_reason,
            convergence,
            convergence_safe,
            convergence_reason,
            rejected,
        ) = self._scg_select_gap_after_convergence(gaps, nearest)
        selected_angle = (
            self._scg_beam_to_angle(selected_gap['mid'])
            if selected_gap is not None else 0.0)
        selected_width = selected_gap['width_m'] if selected_gap is not None else 0.0
        recovery_scan_requested = selected_gap is None

        if selected_gap is None:
            context_h = (
                nearest - self.local_scg_collision_distance
                if np.isfinite(nearest) else float('inf'))
        else:
            nearest_margin = (
                nearest - self.local_scg_collision_distance
                if np.isfinite(nearest) else float('inf'))
            width_margin = selected_gap['width_m'] - self.local_scg_min_gap_width
            clearance_margin = (
                selected_gap['center_clearance'] - self.local_scg_collision_distance)
            context_h = min(nearest_margin, width_margin, clearance_margin)

        self.local_scg_context_h = context_h
        self.local_scg_obstacle_count = obstacle_count
        self.local_scg_gap_count = len(gaps)
        self.local_scg_selected_gap = selected_gap.copy() if selected_gap is not None else None
        self.local_scg_nearest_obstacle = nearest
        self.local_scg_gap_widths = gap_widths
        self.local_scg_obstacle_boundaries = boundaries
        self.local_scg_free_sector_count = free_sector_count
        self.local_scg_selected_gap_angle = selected_angle
        self.local_scg_selected_gap_width = selected_width
        self.local_scg_boundedness = boundedness
        self.local_scg_traversable = traversable
        self.local_scg_boundedness_reason = boundedness_reason
        self.local_scg_convergence = convergence
        self.local_scg_convergence_reason = convergence_reason

        if recovery_scan_requested:
            self.sonar_move_pub.publish(Float64(data=2.0))

        nearest_text = f'{nearest:.2f}' if np.isfinite(nearest) else 'inf'
        context_text = f'{context_h:.3f}' if np.isfinite(context_h) else 'inf'
        self.get_logger().info(
            '[FLS_SENSOR] '
            f'source={source} obstacle_count={obstacle_count} '
            f'gap_count={len(gaps)} selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width:.2f} nearest_obstacle={nearest_text} '
            f'free_beams={int(np.count_nonzero(free_mask))} '
            f'hit_beams={int(np.count_nonzero(has_hit))} '
            f'sonar_profile={self.local_scg_beam_count}beams')
        self.get_logger().info(
            '[SCG] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'free_sectors={free_sector_count} selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width:.2f} '
            f'gap_widths_m={self._format_scg_list(gap_widths, "m")} '
            f'nearest_obstacle={nearest_text} context_h={context_text} '
            f'obstacle_boundaries={self._format_scg_boundaries(boundaries)}')
        self.get_logger().info(
            '[BOUNDEDNESS] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width:.2f} '
            f'classification={boundedness} traversable={traversable} '
            f'reason={boundedness_reason} '
            f'rejected_gaps={self._format_scg_rejections(rejected)} '
            f'context_h={context_text}')
        self.get_logger().info(
            '[CONVERGENCE] '
            f'obstacle_count={obstacle_count} gap_count={len(gaps)} '
            f'selected_gap_angle={selected_angle:.3f} '
            f'selected_gap_width={selected_width:.2f} '
            f'classification={convergence} feasible={convergence_safe} '
            f'reason={convergence_reason} '
            f'history_size={len(self.local_scg_gap_history)} '
            f'nearest_obstacle={nearest_text} '
            f'recovery_scan_requested={recovery_scan_requested} '
            f'context_h={context_text}')

    def _update_obstacle_state(self):
        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        constraints = []
        if self.use_analytical_obstacles:
            constraints.extend(self._analytical_constraints(vp))
        if self._point_cloud_is_fresh() or self._spatial_memory_has_points():
            constraints.extend(self._point_cloud_constraints(vp))
        # Integrate FLS-based SCG constraints when available
        if self.scg_obstacle_count > 0 and np.isfinite(self.scg_h):
            constraints.extend(self._scg_constraints(vp))

        if not constraints:
            self.closest_point = None
            self.closest_source = 'none'
            self.closest_obstacle_distance = float('inf')
            self.current_h = float('inf')
            self.obstacle_constraints = []
            return

        for constraint in constraints:
            constraint['h'] = constraint['distance']**2 - self.R_o**2

        constraints.sort(key=lambda item: item['h'])
        closest = constraints[0]
        max_active_distance = min(
            self.radius,
            max(self.R_o + self.cbf_influence_distance,
                closest['distance'] + self.cbf_blend_distance))
        active = [
            constraint for constraint in constraints
            if constraint['distance'] <= max_active_distance
        ]
        if not active:
            active = [closest]

        self.obstacle_constraints = active[:self.max_active_constraints]
        self.closest_point = closest['point']
        self.closest_source = closest.get('source', 'unknown')
        self.closest_obstacle_distance = closest['distance']
        self.current_h = closest['h']

    def pose_callback(self, msg):
        self.latest_pose_msg = msg
        self.latest_pose_time = self.get_clock().now()

    def _pose_stamp_key(self, msg):
        stamp = msg.header.stamp
        return stamp.sec, stamp.nanosec

    def _refresh_pose_state(self):
        if self.latest_pose_msg is None:
            return

        stamp_key = self._pose_stamp_key(self.latest_pose_msg)
        if stamp_key == self.last_pose_stamp:
            return
        self.last_pose_stamp = stamp_key

        if not (self.xy_cbf or self.xz_cbf or self.sonar_moving): return
        msg = self.latest_pose_msg
        self.vehicle_pose = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        q   = [ori.x, ori.y, ori.z, ori.w]
        self.quaternion = q
        _, _, self.yaw = tft.euler_from_quaternion(q)
        linear = msg.twist.twist.linear
        self.current_planar_speed = float(np.hypot(linear.x, linear.y))

    # ---- CBF helpers ----
    def _R(self):
        x,y,z,w = self.quaternion
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

    def _global_to_local(self, gp):
        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        return self._R().T @ (np.array(gp) - vp)

    def _h_dot_xy(self, vx, vy):
        g = np.array([2*(self.vehicle_pose.x - self.closest_point[0]),
                      2*(self.vehicle_pose.y - self.closest_point[1])])
        return g @ np.array([vx, vy])

    def _h_dot_xz(self, vx, vz):
        lc = self._global_to_local(self.closest_point)
        g  = np.array([-2*lc[0], -2*lc[2]])
        return g @ np.array([vx, vz])

    def _tf_g2l_xy(self, gx, gy):
        lx =  gx*np.cos(self.yaw) + gy*np.sin(self.yaw)
        ly = -gx*np.sin(self.yaw) + gy*np.cos(self.yaw)
        return lx, ly

    def _tf_l2g_xy(self, lx, ly):
        gx = lx*np.cos(self.yaw) - ly*np.sin(self.yaw)
        gy = lx*np.sin(self.yaw) + ly*np.cos(self.yaw)
        return gx, gy

    def _project_to_cbf_constraint(self, desired, gradient, margin):
        norm_sq = float(gradient @ gradient)
        if norm_sq < 1e-9:
            return desired

        constraint_value = float(gradient @ desired + margin)
        if constraint_value >= 0.0:
            return desired

        return desired + (-constraint_value / norm_sq) * gradient

    def _constraint_value(self, candidate, constraint):
        gradient, margin = constraint
        return float(gradient @ candidate + margin)

    def _constraints_satisfied(self, candidate, constraints, tolerance=1e-6):
        if not np.isfinite(candidate).all():
            return False

        if float(np.linalg.norm(candidate)) > self.max_xy_speed + tolerance:
            return False

        return all(
            self._constraint_value(candidate, constraint) >= -tolerance
            for constraint in constraints
        )

    def _project_to_cbf_constraints(self, desired, constraints):
        if not constraints:
            return desired

        desired = desired.astype(float)
        speed = float(np.linalg.norm(desired))
        if speed > self.max_xy_speed:
            desired = desired * (self.max_xy_speed / speed)

        candidates = [desired]

        for gradient, margin in constraints:
            norm_sq = float(gradient @ gradient)
            if norm_sq < 1e-9:
                continue
            line_shift = (-margin - float(gradient @ desired)) / norm_sq
            candidates.append(desired + line_shift * gradient)

        for left_index in range(len(constraints)):
            left_gradient, left_margin = constraints[left_index]
            for right_index in range(left_index + 1, len(constraints)):
                right_gradient, right_margin = constraints[right_index]
                matrix = np.vstack((left_gradient, right_gradient))
                det = float(np.linalg.det(matrix))
                if abs(det) < 1e-9:
                    continue
                rhs = -np.array([left_margin, right_margin], dtype=float)
                try:
                    candidates.append(np.linalg.solve(matrix, rhs))
                except np.linalg.LinAlgError:
                    continue

        feasible = [
            candidate for candidate in candidates
            if self._constraints_satisfied(candidate, constraints)
        ]
        if feasible:
            return min(
                feasible,
                key=lambda candidate: float(np.linalg.norm(candidate - desired)))

        # Fallback to iterative half-plane projection if numerical degeneracy
        # prevents the small active-set solve from finding a feasible point.
        safe = desired.copy()
        for _ in range(max(12, len(constraints) * 2)):
            changed = False
            for gradient, margin in constraints:
                previous = safe.copy()
                safe = self._project_to_cbf_constraint(safe, gradient, margin)
                changed = changed or np.linalg.norm(safe - previous) > 1e-6
            if not changed:
                break

        speed = float(np.linalg.norm(safe))
        if speed > self.max_xy_speed:
            safe = safe * (self.max_xy_speed / speed)
        return safe

    def _slew_xy(self, desired):
        delta = desired - self.last_safe_xy
        norm = float(np.linalg.norm(delta))
        if norm > self.max_xy_delta and self.current_h > 0.0:
            desired = self.last_safe_xy + delta * (self.max_xy_delta / norm)
        self.last_safe_xy = desired
        return desired

    def _shape_local_xy(self, local_safe, requested):
        requested_planar = float(np.hypot(requested.linear.x, requested.linear.y))
        if requested_planar <= 1e-4:
            return np.zeros(2)

        local_safe[0] = max(local_safe[0], -self.max_reverse_speed)
        if self.closest_obstacle_distance > self.reverse_allowed_distance:
            local_safe[0] = max(
                local_safe[0],
                min(self.min_avoid_forward_speed, requested.linear.x))

        speed = float(np.linalg.norm(local_safe))
        if speed > self.max_xy_speed:
            local_safe = local_safe * (self.max_xy_speed / speed)
        return local_safe

    def _elapsed_sec(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _xy_projection_constraints(self):
        if not self.obstacle_constraints:
            return []

        # Backwards-compatible: if called without requested, behave as before.
        return self._xy_projection_constraints_with_request(None)

    def _xy_projection_constraints_with_request(self, requested):
        vp_xy = np.array([self.vehicle_pose.x, self.vehicle_pose.y], dtype=float)
        constraints = []

        for obstacle in self.obstacle_constraints:
            cp_xy = np.asarray(obstacle['point'][:2], dtype=float)
            gradient = 2.0 * (vp_xy - cp_xy)
            margin = self.kappa * (obstacle['h'] - self.minimum_clearance)
            constraints.append((gradient, margin))
        return constraints

    def _local_recovery_points(self):
        point_sets = []
        if self._point_cloud_is_fresh() and len(self.filtered_points):
            point_sets.append(np.asarray(self.filtered_points, dtype=float))
        if self.obstacle_constraints:
            point_sets.append(np.asarray([
                obstacle['point'] for obstacle in self.obstacle_constraints
            ], dtype=float))
        if not point_sets:
            return np.empty((0, 3))

        points = np.vstack(point_sets)
        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        return (self._R().T @ (points - vp).T).T

    def _opposing_obstacles_nearby(self):
        points = self._local_recovery_points()
        if points.size == 0:
            return False

        planar_ranges = np.linalg.norm(points[:, :2], axis=1)
        near = np.logical_and.reduce((
            np.isfinite(planar_ranges),
            planar_ranges <= self.hover_lock_distance,
            points[:, 0] >= -1.0,
            np.abs(points[:, 1]) >= self.hover_lock_lateral_min,
        ))
        if not np.any(near):
            return False

        near_points = points[near]
        left_blocked = np.any(near_points[:, 1] > self.hover_lock_lateral_min)
        right_blocked = np.any(near_points[:, 1] < -self.hover_lock_lateral_min)
        return bool(left_blocked and right_blocked)

    def _update_hover_oscillation_score(self, local_safe, requested):
        lateral = float(local_safe[1])
        yaw = float(requested.angular.z)
        lateral_sign = (
            float(np.sign(lateral))
            if abs(lateral) >= self.hover_lock_min_lateral_cmd else 0.0
        )
        yaw_sign = (
            float(np.sign(yaw))
            if abs(yaw) >= self.hover_lock_min_yaw_cmd else 0.0
        )

        flipped = (
            lateral_sign != 0.0 and
            self.last_hover_lateral_sign != 0.0 and
            lateral_sign != self.last_hover_lateral_sign
        ) or (
            yaw_sign != 0.0 and
            self.last_hover_yaw_sign != 0.0 and
            yaw_sign != self.last_hover_yaw_sign
        )

        self.hover_oscillation_score *= self.hover_lock_oscillation_decay
        if flipped:
            self.hover_oscillation_score += 1.0

        if lateral_sign != 0.0:
            self.last_hover_lateral_sign = lateral_sign
        if yaw_sign != 0.0:
            self.last_hover_yaw_sign = yaw_sign

    def _hover_lock_active(self):
        if self.hover_lock_until_sec <= 0.0:
            return False

        now = self._now_sec()
        still_holding = now < self.hover_lock_until_sec
        still_squeezed = (
            self.closest_obstacle_distance <= self.hover_lock_release_distance and
            self._opposing_obstacles_nearby()
        )
        if still_holding or still_squeezed:
            return True

        self.hover_lock_until_sec = 0.0
        self.hover_lock_cooldown_until_sec = now + self.hover_lock_cooldown_seconds
        self.hover_oscillation_score = 0.0
        self.hover_lock_reason = 'none'
        self.get_logger().info('[CBF_HOVER] hover lock released; retrying navigation')
        return False

    def _maybe_hover_lock(self, local_safe, requested):
        if not self.hover_lock_enabled:
            return False
        if self._hover_lock_active():
            self.last_recovery_heading = self._stable_recovery_heading(requested)
            self.last_cbf_action = 'hover_recovery'
            self.cbf_recovery_mode = True
            return True

        now = self._now_sec()
        if now < self.hover_lock_cooldown_until_sec:
            return False

        requested_planar = float(np.hypot(requested.linear.x, requested.linear.y))
        projected_planar = float(np.linalg.norm(local_safe))
        squeezed = self._opposing_obstacles_nearby()
        close = self.closest_obstacle_distance <= self.hover_lock_distance
        if squeezed and close:
            if self.hover_squeeze_started_sec <= 0.0:
                self.hover_squeeze_started_sec = now
        else:
            self.hover_squeeze_started_sec = 0.0

        squeeze_dwell = (
            self.hover_squeeze_started_sec > 0.0 and
            now - self.hover_squeeze_started_sec >= self.hover_lock_squeeze_seconds
        )
        oscillating = self.hover_oscillation_score >= self.hover_lock_oscillation_threshold
        stalled_by_cbf = (
            requested_planar > self.cbf_stall_speed_threshold and
            projected_planar < self.cbf_stall_speed_threshold
        )

        if squeezed and close and (
                squeeze_dwell or oscillating or stalled_by_cbf or self.cbf_recovery_mode):
            self.hover_lock_until_sec = now + self.hover_lock_min_seconds
            self.hover_lock_reason = (
                'squeeze_hover' if squeeze_dwell else
                'oscillation' if oscillating else
                'stalled_cbf' if stalled_by_cbf else
                'recovery_squeeze'
            )
            self.last_recovery_heading = self._stable_recovery_heading(requested)
            self.last_cbf_action = 'hover_recovery'
            self.cbf_recovery_mode = True
            self.get_logger().warning(
                '[CBF_HOVER] entering hover lock: '
                f'reason={self.hover_lock_reason} '
                f'nearest={self.closest_obstacle_distance:.2f}m '
                f'constraints={len(self.obstacle_constraints)} '
                f'oscillation_score={self.hover_oscillation_score:.2f}')
            return True

        return False

    def _fallback_recovery_heading(self, requested):
        half_fov = self.recovery_gap_fov / 2.0
        if abs(requested.angular.z) > 0.03:
            return float(np.clip(
                requested.angular.z / max(self.recovery_heading_kp, 1e-3),
                -half_fov,
                half_fov))

        if self.closest_point is not None:
            local_closest = self._global_to_local(self.closest_point)
            side = -np.sign(local_closest[1])
            if side == 0.0:
                side = 1.0
            return float(np.clip(side * half_fov * 0.65, -half_fov, half_fov))

        return 0.0

    def _largest_free_gap_heading(self, requested):
        half_fov = self.recovery_gap_fov / 2.0
        points = self._local_recovery_points()
        if points.size == 0:
            return self._fallback_recovery_heading(requested)

        planar_ranges = np.linalg.norm(points[:, :2], axis=1)
        bearings = np.arctan2(points[:, 1], points[:, 0])
        valid = np.logical_and.reduce((
            np.isfinite(planar_ranges),
            points[:, 0] > 0.05,
            planar_ranges >= self.point_cloud_min_range,
            planar_ranges <= self.radius,
            np.abs(bearings) <= half_fov,
        ))
        if not np.any(valid):
            return self._fallback_recovery_heading(requested)

        inflated_radius = max(0.05, self.R_o + self.minimum_clearance)
        blocked_half_width = np.clip(
            np.arctan2(inflated_radius, planar_ranges[valid]),
            np.deg2rad(4.0),
            np.deg2rad(28.0))
        blocked = sorted(zip(
            np.clip(bearings[valid] - blocked_half_width, -half_fov, half_fov),
            np.clip(bearings[valid] + blocked_half_width, -half_fov, half_fov),
        ))

        merged = []
        for start, end in blocked:
            if not merged or start > merged[-1][1]:
                merged.append([float(start), float(end)])
            else:
                merged[-1][1] = max(merged[-1][1], float(end))

        gaps = []
        cursor = -half_fov
        for start, end in merged:
            if start > cursor:
                gaps.append((cursor, start))
            cursor = max(cursor, end)
        if cursor < half_fov:
            gaps.append((cursor, half_fov))

        if not gaps:
            return self._fallback_recovery_heading(requested)

        preferred = self._fallback_recovery_heading(requested)
        def score(gap):
            center = 0.5 * (gap[0] + gap[1])
            width = gap[1] - gap[0]
            return width - 0.15 * abs(center - preferred)

        best = max(gaps, key=score)
        return float(np.clip(0.5 * (best[0] + best[1]), -half_fov, half_fov))

    def _stable_recovery_heading(self, requested):
        candidate = self._largest_free_gap_heading(requested)
        now = self._now_sec()
        deadband = max(0.0, self.recovery_heading_deadband)

        if abs(candidate) < deadband and abs(self.last_recovery_heading) >= deadband:
            candidate = self.last_recovery_heading

        holding = (
            now < self.recovery_heading_hold_until_sec and
            abs(self.last_recovery_heading) >= deadband
        )
        if holding:
            same_side = (
                abs(candidate) < deadband or
                np.sign(candidate) == np.sign(self.last_recovery_heading)
            )
            if not same_side:
                return self.last_recovery_heading

        self.last_recovery_heading = float(candidate)
        if abs(self.last_recovery_heading) >= deadband:
            self.recovery_heading_hold_until_sec = (
                now + max(0.0, self.recovery_heading_hold_seconds))
        return self.last_recovery_heading

    def _update_cbf_recovery_state(self, raw_planar, projected_planar):
        now = self.get_clock().now()
        scg_context_rejected = (
            np.isfinite(self.scg_h) and
            self.scg_h < -0.05 and
            raw_planar > self.cbf_stall_speed_threshold and
            self.closest_obstacle_distance > self.emergency_collision_distance
        )
        rejected_forward = (
            raw_planar > self.cbf_stall_speed_threshold and
            (
                projected_planar < self.cbf_stall_speed_threshold or
                self.current_planar_speed < self.cbf_stall_speed_threshold
            ) and
            self.closest_obstacle_distance > self.emergency_collision_distance
        ) or scg_context_rejected

        if rejected_forward:
            if self.rejected_motion_since is None:
                self.rejected_motion_since = now
            elif self._elapsed_sec(self.rejected_motion_since) >= self.recovery_trigger_seconds:
                if not self.cbf_recovery_mode:
                    self.get_logger().warning(
                        '[CBF_RECOVERY] filtered velocity near zero for more than '
                        f'{self.recovery_trigger_seconds:.1f}s; rotating toward largest free gap')
                self.cbf_recovery_mode = True
                try:
                    # Request sonar pivoting to re-scan environment while recovering
                    self.sonar_move_pub.publish(Float64(data=2.0))
                except Exception:
                    pass
                # Explicit recovery diagnostic
                heading = self._stable_recovery_heading(self.v_alg if self.v_alg is not None else Twist())
                self.get_logger().info(
                    '[RECOVERY] entering recovery mode: heading={:.3f} rad '
                    'recovery_forward_speed={:.3f} context_h={:.3f}'.format(
                        heading, self.recovery_forward_speed, self.scg_h))
        else:
            self.rejected_motion_since = None
            if self.cbf_recovery_mode and projected_planar > self.cbf_stall_speed_threshold * 1.5:
                self.cbf_recovery_mode = False
                self.get_logger().info('[CBF_RECOVERY] planner command passes CBF again; resuming normal filtering')

    def _recovery_xy_command(self, requested, heading):
        speed = min(
            self.max_xy_speed,
            max(self.recovery_forward_speed, self.min_avoid_forward_speed))
        if requested.linear.x > 0.0:
            speed = min(speed, max(requested.linear.x, self.recovery_forward_speed))

        local = np.array([
            speed * max(0.25, np.cos(heading)),
            speed * np.sin(heading),
        ], dtype=float)

        if abs(local[1]) < self.recovery_side_step_speed * 0.5 and abs(heading) > 0.05:
            local[1] = np.sign(heading) * self.recovery_side_step_speed * 0.5

        norm = float(np.linalg.norm(local))
        if norm > self.max_xy_speed:
            local = local * (self.max_xy_speed / norm)
        return local

    def _safety_escape_xy_command(self):
        if self.current_h >= 0.0 or self.closest_point is None:
            return None
        if self.closest_obstacle_distance <= self.emergency_collision_distance:
            self.last_cbf_action = 'collision_stop'
            return np.zeros(2)

        local_closest = self._global_to_local(self.closest_point)
        away = -np.asarray(local_closest[:2], dtype=float)
        norm = float(np.linalg.norm(away))
        if norm < 1e-6:
            return None

        speed = min(
            self.max_xy_speed,
            max(self.safety_escape_speed, self.recovery_side_step_speed))
        self.last_cbf_action = 'safety_escape'
        return away * (speed / norm)

    def _project_local_xy(self, local_xy, constraints):
        if not constraints:
            return local_xy

        candidate_global = np.array(
            self._tf_l2g_xy(local_xy[0], local_xy[1]),
            dtype=float)
        safe_global = self._project_to_cbf_constraints(candidate_global, constraints)
        return np.array(self._tf_g2l_xy(safe_global[0], safe_global[1]), dtype=float)

    def _barrier_sliding_xy_command(self, requested, constraints):
        if self.closest_point is None or not np.isfinite(self.current_h):
            return None
        if self.closest_obstacle_distance <= self.emergency_collision_distance:
            return None
        near_barrier = (
            self.current_h <= self.barrier_slide_trigger_h or
            self.closest_obstacle_distance <= max(
                self.imminent_collision_distance,
                self.R_o + self.cbf_influence_distance)
        )
        if not near_barrier:
            return None

        local_closest = self._global_to_local(self.closest_point)
        away = -np.asarray(local_closest[:2], dtype=float)
        away_norm = float(np.linalg.norm(away))
        if away_norm < 1e-6:
            return None
        away = away / away_norm

        requested_xy = np.array([requested.linear.x, requested.linear.y], dtype=float)
        requested_norm = float(np.linalg.norm(requested_xy))
        requested_dir = requested_xy / requested_norm if requested_norm > 1e-6 else np.array([1.0, 0.0])

        tangent = np.array([-away[1], away[0]], dtype=float)
        if float(tangent @ requested_dir) < float((-tangent) @ requested_dir):
            tangent = -tangent

        away_weight = float(np.clip(self.barrier_slide_away_weight, 0.05, 1.5))
        blended = tangent + away_weight * away
        blended_norm = float(np.linalg.norm(blended))
        if blended_norm < 1e-6:
            return None

        speed = min(
            self.max_xy_speed,
            max(
                self.barrier_slide_speed,
                self.recovery_side_step_speed,
                min(self.recovery_forward_speed, max(self.min_avoid_forward_speed, requested_norm)),
            ))
        local = blended * (speed / blended_norm)
        local = self._project_local_xy(local, constraints)

        if float(np.linalg.norm(local)) < self.cbf_stall_speed_threshold:
            return None

        return local

    def _avoid_zero_xy(self, local_safe, requested):
        raw_planar = float(np.hypot(requested.linear.x, requested.linear.y))
        projected_planar = float(np.linalg.norm(local_safe))
        self._update_cbf_recovery_state(raw_planar, projected_planar)
        self.last_cbf_action = 'filtered'

        safety_escape = self._safety_escape_xy_command()
        if safety_escape is not None:
            return safety_escape

        if raw_planar <= self.cbf_stall_speed_threshold:
            self.last_cbf_action = 'planner_idle'
            return local_safe

        unstick_local = self._free_space_unstick_local(requested)
        if unstick_local is not None:
            return unstick_local

        projected_ok = projected_planar >= self.cbf_stall_speed_threshold
        if projected_ok and not self.cbf_recovery_mode:
            constraints = self._xy_projection_constraints_with_request(requested)
            slide_local = self._barrier_sliding_xy_command(requested, constraints)
            if slide_local is not None:
                slide_planar = float(np.linalg.norm(slide_local))
                if slide_planar > max(projected_planar * 1.15, self.cbf_stall_speed_threshold):
                    self.last_cbf_action = 'barrier_slide'
                    return slide_local
            return local_safe

        heading = self._stable_recovery_heading(requested)

        if self.closest_obstacle_distance <= self.emergency_collision_distance:
            self.last_cbf_action = 'collision_stop'
            return np.zeros(2)

        candidate_local = self._recovery_xy_command(requested, heading)
        # pass the requested planner velocity into the constraint helper
        # (was referencing undefined variable `v` previously)
        constraints = self._xy_projection_constraints_with_request(requested)
        if constraints:
            candidate_local = self._project_local_xy(candidate_local, constraints)

        slide_local = self._barrier_sliding_xy_command(requested, constraints)
        if slide_local is not None:
            slide_planar = float(np.linalg.norm(slide_local))
            candidate_planar = float(np.linalg.norm(candidate_local))
            if self.cbf_recovery_mode or slide_planar > max(projected_planar, candidate_planar):
                self.last_cbf_action = 'barrier_slide'
                return slide_local

        if (np.linalg.norm(candidate_local) < self.cbf_stall_speed_threshold and
                self.closest_obstacle_distance > self.emergency_collision_distance):
            self.last_cbf_action = 'rotate_in_place'
            return np.zeros(2)

        if np.linalg.norm(candidate_local) > projected_planar or self.cbf_recovery_mode:
            self.last_cbf_action = 'recovery_gap' if self.cbf_recovery_mode else 'escape_side_step'
            return candidate_local

        return local_safe

    def _free_space_unstick_local(self, requested):
        if not self.free_space_unstick_enabled:
            return None
        far_from_active_barrier = (
            not self.obstacle_constraints or
            self.closest_obstacle_distance > max(
                self.imminent_collision_distance,
                self.R_o + self.cbf_influence_distance)
        )
        if not far_from_active_barrier:
            self.free_space_low_speed_since_sec = 0.0
            return None
        if len(self.spatial_memory_points) > 0:
            self.free_space_low_speed_since_sec = 0.0
            return None

        raw_planar = float(np.hypot(requested.linear.x, requested.linear.y))
        if raw_planar <= self.cbf_stall_speed_threshold:
            self.free_space_low_speed_since_sec = 0.0
            return None

        now = self._now_sec()
        active = now < self.free_space_unstick_until_sec
        moving = self.current_planar_speed >= self.cbf_stall_speed_threshold

        if moving and not active:
            self.free_space_low_speed_since_sec = 0.0
            return None

        if not active:
            if self.free_space_low_speed_since_sec <= 0.0:
                self.free_space_low_speed_since_sec = now
                return None
            if now - self.free_space_low_speed_since_sec < self.free_space_unstick_timeout:
                return None

            self.free_space_unstick_until_sec = (
                now + max(0.0, self.free_space_unstick_duration))
            active = True
            self.get_logger().warning(
                '[CBF_RECOVERY] free-space unstick: '
                f'commanded={raw_planar:.3f}m/s measured={self.current_planar_speed:.3f}m/s '
                f'for {self.free_space_unstick_timeout:.1f}s; sending forward-only burst')

        if not active:
            return None

        speed = min(
            self.max_xy_speed,
            max(self.free_space_unstick_speed, requested.linear.x, self.min_avoid_forward_speed))
        self.last_cbf_action = 'free_space_unstick'
        return np.array([speed, 0.0], dtype=float)

    def _depth_hold_velocity(self):
        if self.vehicle_pose is None:
            return 0.0
        error = self.target_depth - self.vehicle_pose.z
        if abs(error) <= self.depth_deadband:
            desired = 0.0
        else:
            corrected_error = error - np.sign(error) * self.depth_deadband
            # Gazebo's odometry and the ROS 2 velocity controller use ENU z:
            # positive velocity raises the vehicle, negative velocity dives.
            desired = self.depth_hold_kp * corrected_error

        desired = float(np.clip(desired, -self.max_vertical_speed, self.max_vertical_speed))
        delta = np.clip(
            desired - self.last_vertical_cmd,
            -self.max_vertical_delta,
            self.max_vertical_delta)
        self.last_vertical_cmd = float(self.last_vertical_cmd + delta)
        return self.last_vertical_cmd

    def _opt_xy(self, v):
        dgx, dgy = self._tf_l2g_xy(v.linear.x, v.linear.y)
        desired = np.array([dgx, dgy], dtype=float)
        # Project the planner's nominal command against every active barrier.
        constraints = self._xy_projection_constraints_with_request(v)
        if not constraints:
            safe = desired
        else:
            try:
                safe = self._project_to_cbf_constraints(desired, constraints)
            except Exception as exc:
                self.get_logger().error(
                    f'[CBF] XY projection failed, falling back to recovery: {exc}')
                self.last_cbf_action = 'projection_error'
                safe = np.zeros(2, dtype=float)

        safe = self._slew_xy(safe)
        local_safe = np.array(self._tf_g2l_xy(safe[0], safe[1]))
        local_safe = self._avoid_zero_xy(local_safe, v)
        if self.last_cbf_action not in ('collision_stop', 'safety_escape', 'barrier_slide'):
            local_safe = self._shape_local_xy(local_safe, v)
            local_safe = self._project_local_xy(local_safe, constraints)
        self._update_hover_oscillation_score(local_safe, v)
        if self._maybe_hover_lock(local_safe, v):
            self.cbf_recovery_mode = True
            local_safe = self._recovery_xy_command(v, self.last_recovery_heading)
            local_safe = self._project_local_xy(local_safe, constraints)
        shaped_global = np.array(self._tf_l2g_xy(local_safe[0], local_safe[1]))
        self.last_safe_xy = shaped_global
        return local_safe

    def _opt_xz(self, v, vertical_cmd=None):
        desired_z = v.linear.z if vertical_cmd is None else vertical_cmd
        desired = np.array([v.linear.x, desired_z], dtype=float)
        local_closest = self._global_to_local(self.closest_point)
        gradient = np.array([-2*local_closest[0], -2*local_closest[2]], dtype=float)
        margin = self.kappa1 * (self.current_h - 0.5)
        return self._project_to_cbf_constraint(desired, gradient, margin)

    def _filtered_angular_z(self, requested):
        if self.last_cbf_action in ('collision_stop', 'safety_escape'):
            return 0.0
        if self.last_cbf_action == 'free_space_unstick':
            return 0.0
        if self.last_cbf_action not in (
                'escape_side_step', 'recovery_gap', 'collision_rotate',
                'rotate_in_place', 'hover_recovery'):
            return requested.angular.z

        desired = float(np.clip(
            self.recovery_heading_kp * self.last_recovery_heading,
            -self.recovery_yaw_rate,
            self.recovery_yaw_rate))
        if abs(desired) < 0.05:
            desired = np.sign(requested.angular.z) * min(
                self.recovery_yaw_rate, max(0.05, abs(requested.angular.z)))
        if desired == 0.0:
            desired = self.recovery_yaw_rate

        if abs(requested.angular.z) > abs(desired) and self.last_cbf_action != 'collision_rotate':
            return requested.angular.z
        return desired

    def _slew_scalar(self, desired, previous, max_delta):
        delta = float(np.clip(desired - previous, -max_delta, max_delta))
        return float(previous + delta)

    def _spd2c_reference(self):
        planner_fresh = self._planner_cmd_is_fresh()
        if planner_fresh:
            return self.v_alg, 'external_reference'

        reference = Twist()
        reference.linear.x = self.spd2c_cruise_speed
        return reference, 'forward_default'

    def _spd2c_reference_heading(self, reference):
        planar = float(np.hypot(reference.linear.x, reference.linear.y))
        if planar > 1e-4:
            angle = math.atan2(
                reference.linear.y,
                max(abs(reference.linear.x), 1e-3))
        elif abs(reference.angular.z) > 0.02:
            angle = reference.angular.z / max(self.spd2c_yaw_kp, 1e-3)
        else:
            angle = 0.0

        return float(np.clip(angle, -self.local_scg_fov / 2.0, self.local_scg_fov / 2.0))

    def _spd2c_reference_speed(self, reference):
        speed = float(np.hypot(reference.linear.x, reference.linear.y))
        if speed < self.spd2c_min_forward_speed:
            speed = self.spd2c_cruise_speed
        return float(np.clip(speed, self.spd2c_min_forward_speed, self.max_xy_speed))

    def _spd2c_command(self):
        reference, reference_source = self._spd2c_reference()
        reference_heading = self._spd2c_reference_heading(reference)
        reference_speed = self._spd2c_reference_speed(reference)
        selected_gap = self.local_scg_selected_gap
        half_fov = self.local_scg_fov / 2.0

        valid_gap = (
            selected_gap is not None and
            self.local_scg_traversable and
            self.local_scg_convergence != 'NARROWING_UNSAFE')

        cmd = Twist()
        if valid_gap:
            gap_angle = float(self.local_scg_selected_gap_angle)
            if self.local_scg_boundedness == 'OPEN':
                target_angle = 0.75 * gap_angle + 0.25 * reference_heading
            elif self.local_scg_boundedness in ('LEFT_BOUNDED', 'RIGHT_BOUNDED'):
                target_angle = 0.85 * gap_angle + 0.15 * reference_heading
            else:
                target_angle = gap_angle
            target_angle = float(np.clip(target_angle, -half_fov, half_fov))

            width_scale = float(np.clip(
                selected_gap['width_m'] / max(self.local_scg_min_gap_width * 1.8, 1e-6),
                0.45,
                1.0))
            clearance = (
                selected_gap['center_clearance']
                if np.isfinite(selected_gap['center_clearance'])
                else self.radius)
            clearance_scale = float(np.clip(
                (clearance - self.local_scg_collision_distance) /
                max(0.5, self.radius - self.local_scg_collision_distance),
                0.35,
                1.0))
            turn_scale = float(np.clip(
                1.0 - 0.45 * abs(target_angle) / max(half_fov, 1e-6),
                0.45,
                1.0))
            bounded_scale = (
                0.82 if self.local_scg_boundedness == 'BOTH_BOUNDED' else
                0.92 if self.local_scg_boundedness in ('LEFT_BOUNDED', 'RIGHT_BOUNDED') else
                1.0)
            convergence_scale = (
                1.05 if self.local_scg_convergence == 'WIDENING' else 0.90)
            speed = (
                reference_speed *
                width_scale *
                clearance_scale *
                turn_scale *
                bounded_scale *
                convergence_scale)
            speed = float(np.clip(
                speed, self.spd2c_min_forward_speed, self.max_xy_speed))
            decision = 'gap_follow'
        else:
            self.sonar_move_pub.publish(Float64(data=2.0))
            target_angle = self._stable_recovery_heading(reference)
            if abs(target_angle) < 0.05:
                sweep_sign = np.sign(reference.angular.z)
                if sweep_sign == 0.0:
                    sweep_sign = 1.0
                target_angle = float(sweep_sign * min(0.45, 0.65 * half_fov))
            target_angle = float(np.clip(target_angle, -half_fov, half_fov))
            speed = float(np.clip(
                self.spd2c_recovery_forward_speed,
                0.0,
                self.max_xy_speed))
            decision = 'recovery_scan'

        desired_yaw = float(np.clip(
            self.spd2c_yaw_kp * target_angle,
            -self.spd2c_max_yaw_rate,
            self.spd2c_max_yaw_rate))
        desired_forward = max(0.0, speed * math.cos(target_angle))
        desired_lateral = (
            self.spd2c_lateral_gain * speed * math.sin(target_angle))

        cmd.linear.x = self._slew_scalar(
            desired_forward,
            self.last_spd2c_cmd.linear.x,
            self.spd2c_max_speed_delta)
        cmd.linear.y = self._slew_scalar(
            desired_lateral,
            self.last_spd2c_cmd.linear.y,
            self.spd2c_max_speed_delta)
        cmd.linear.z = 0.0
        cmd.angular.z = self._slew_scalar(
            desired_yaw,
            self.last_spd2c_cmd.angular.z,
            self.spd2c_max_yaw_delta)

        self.last_spd2c_cmd = cmd
        self.last_spd2c_decision = decision
        self.get_logger().info(
            '[SPD2C] '
            f'decision={decision} reference_source={reference_source} '
            f'obstacle_count={self.local_scg_obstacle_count} '
            f'gap_count={self.local_scg_gap_count} '
            f'free_sectors={self.local_scg_free_sector_count} '
            f'sonar_profile={self.local_scg_beam_count}beams '
            f'selected_gap_angle={self.local_scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.local_scg_selected_gap_width:.2f} '
            f'boundedness={self.local_scg_boundedness} '
            f'convergence={self.local_scg_convergence} '
            f'context_h={self.local_scg_context_h:.3f} '
            f'nearest_obstacle={self.local_scg_nearest_obstacle:.3f} '
            f'current_position=({self.vehicle_pose.x:.3f},{self.vehicle_pose.y:.3f},{self.vehicle_pose.z:.3f}) '
            f'current_yaw={self.yaw:.3f} current_velocity={self.current_planar_speed:.3f} '
            f'mission_reference=({reference.linear.x:.3f},{reference.linear.y:.3f},{reference.angular.z:.3f}) '
            f'reference_heading={reference_heading:.3f} target_heading={target_angle:.3f} '
            f'desired_forward_velocity={cmd.linear.x:.3f} '
            f'desired_lateral_velocity={cmd.linear.y:.3f} '
            f'desired_yaw_rate={cmd.angular.z:.3f}')
        return cmd

    def _copy_twist(self, msg):
        copied = Twist()
        copied.linear.x = msg.linear.x
        copied.linear.y = msg.linear.y
        copied.linear.z = msg.linear.z
        copied.angular.x = msg.angular.x
        copied.angular.y = msg.angular.y
        copied.angular.z = msg.angular.z
        return copied

    def _recovery_yaw_rate_for_heading(self, heading, requested):
        yaw = float(np.clip(
            self.recovery_heading_kp * heading,
            -self.recovery_yaw_rate,
            self.recovery_yaw_rate))
        if abs(yaw) < 0.08:
            fallback_sign = np.sign(heading)
            if fallback_sign == 0.0:
                fallback_sign = np.sign(requested.angular.z)
            if fallback_sign == 0.0:
                fallback_sign = np.sign(self.local_scg_selected_gap_angle)
            if fallback_sign == 0.0:
                fallback_sign = 1.0
            yaw = float(fallback_sign * min(self.recovery_yaw_rate, 0.18))
        return yaw

    def _final_cbf_recovery_filter(self, raw, cbf_tw):
        filtered = self._copy_twist(cbf_tw)
        raw_planar = float(np.hypot(raw.linear.x, raw.linear.y))
        cbf_planar = float(np.hypot(cbf_tw.linear.x, cbf_tw.linear.y))
        emergency = self.closest_obstacle_distance <= self.emergency_collision_distance
        now = self._now_sec()

        near_zero = (
            raw_planar > self.cbf_stall_speed_threshold and
            (
                cbf_planar < self.cbf_stall_speed_threshold or
                self.current_planar_speed < self.cbf_stall_speed_threshold
            )
        )

        if emergency:
            self.final_velocity_near_zero_since_sec = 0.0
            self.final_velocity_recovery_active = False
            self.final_velocity_recovery_elapsed = 0.0
            self.final_velocity_recovery_forward_safe = False
            self.final_velocity_recovery_scan_requested = False
            self.last_cbf_action = 'collision_stop'
            filtered.linear.x = 0.0
            filtered.linear.y = 0.0
            filtered.angular.z = 0.0
            return filtered

        if not near_zero:
            self.final_velocity_near_zero_since_sec = 0.0
            self.final_velocity_recovery_elapsed = 0.0
            self.final_velocity_recovery_forward_safe = False
            self.final_velocity_recovery_scan_requested = False
            if self.final_velocity_recovery_active and cbf_planar > self.cbf_stall_speed_threshold * 1.5:
                self.get_logger().info('[RECOVERY] final CBF velocity recovered; leaving final-command recovery')
            self.final_velocity_recovery_active = False
            return filtered

        if self.final_velocity_near_zero_since_sec <= 0.0:
            self.final_velocity_near_zero_since_sec = now

        elapsed = now - self.final_velocity_near_zero_since_sec
        self.final_velocity_recovery_elapsed = elapsed
        heading = self._stable_recovery_heading(raw)
        self.final_velocity_recovery_heading = heading
        self.final_velocity_recovery_scan_requested = True
        self.sonar_move_pub.publish(Float64(data=2.0))

        # Keep the command nonzero while waiting for the recovery dwell timer:
        # rotate/sweep instead of publishing an all-zero cmd_vel.
        if abs(filtered.angular.z) < 0.05:
            filtered.angular.z = self._recovery_yaw_rate_for_heading(heading, raw)

        if elapsed < self.recovery_trigger_seconds:
            self.final_velocity_recovery_active = False
            self.final_velocity_recovery_forward_safe = False
            if self.last_cbf_action not in ('safety_escape', 'barrier_slide'):
                self.last_cbf_action = 'pre_recovery_scan'
            return filtered

        if not self.final_velocity_recovery_active:
            self.get_logger().warning(
                '[RECOVERY] final/CBF velocity near zero for more than '
                f'{self.recovery_trigger_seconds:.1f}s; entering recovery scan')
        self.final_velocity_recovery_active = True
        self.cbf_recovery_mode = True
        self.last_cbf_action = 'final_velocity_recovery'

        forward_safe = (
            self.closest_obstacle_distance > self.imminent_collision_distance and
            (
                not self.obstacle_constraints or
                not np.isfinite(self.current_h) or
                self.current_h > 0.0
            )
        )
        self.final_velocity_recovery_forward_safe = bool(forward_safe)

        if forward_safe:
            candidate = self._recovery_xy_command(raw, heading)
            constraints = self._xy_projection_constraints_with_request(raw)
            if constraints:
                candidate = self._project_local_xy(candidate, constraints)
            if float(np.linalg.norm(candidate)) >= self.cbf_stall_speed_threshold:
                filtered.linear.x = candidate[0]
                filtered.linear.y = candidate[1]
            else:
                filtered.linear.x = 0.0
                filtered.linear.y = 0.0
        else:
            filtered.linear.x = 0.0
            filtered.linear.y = 0.0

        filtered.angular.z = self._recovery_yaw_rate_for_heading(heading, raw)
        return filtered

    def process_data(self, v):
        published = False
        if self.xy_cbf:
            if self.current_h != float('inf'):
                try:
                    safe = self._opt_xy(v)
                except Exception as exc:
                    self.get_logger().error(
                        f'[CBF] XY processing failed, using recovery rotate: {exc}')
                    self.last_cbf_action = 'projection_error'
                    safe = np.zeros(2, dtype=float)
            else:
                safe = self._free_space_unstick_local(v)
                if safe is None:
                    self.last_cbf_action = 'no_obstacle'
                    safe = np.array([v.linear.x, v.linear.y])
            cbf_tw = Twist()
            cbf_tw.angular.z = self._filtered_angular_z(v)
            cbf_tw.linear.x = safe[0]
            cbf_tw.linear.y = safe[1]
            cbf_tw.linear.z = v.linear.z
            cbf_tw = self._final_cbf_recovery_filter(v, cbf_tw)
            tw = Twist()
            tw.angular.z = cbf_tw.angular.z
            tw.linear.x = cbf_tw.linear.x
            tw.linear.y = cbf_tw.linear.y
            tw.linear.z = self._depth_hold_velocity()
            self.h_pub.publish(Float64(data=float(self.current_h)))
            self._publish_cmd(
                v, cbf_tw, tw,
                'xy_cbf_sonar_pivot' if self.sonar_moving else 'xy_cbf')
            published = True
        if self.xz_cbf:
            depth_cmd = self._depth_hold_velocity()
            try:
                safe = (
                    self._opt_xz(v, depth_cmd)
                    if self.current_h != float('inf')
                    else np.array([v.linear.x, depth_cmd])
                )
            except Exception as exc:
                self.get_logger().error(
                    f'[CBF] XZ processing failed, falling back to no z projection: {exc}')
                safe = np.array([v.linear.x, depth_cmd])
            cbf_tw = Twist()
            cbf_tw.angular.z = v.angular.z
            cbf_tw.linear.x = safe[0]
            cbf_tw.linear.z = safe[1]
            cbf_tw = self._final_cbf_recovery_filter(v, cbf_tw)
            tw = Twist()
            tw.angular.z = cbf_tw.angular.z
            tw.linear.x = cbf_tw.linear.x
            tw.linear.z = cbf_tw.linear.z
            self.h_pub.publish(Float64(data=float(self.current_h)))
            self._publish_cmd(v, cbf_tw, tw, 'xz_cbf')
            published = True
        if not published:
            cbf_tw = Twist()
            cbf_tw.angular.z = v.angular.z
            cbf_tw.linear.x = v.linear.x
            cbf_tw.linear.y = v.linear.y
            cbf_tw.linear.z = v.linear.z
            cbf_tw = self._final_cbf_recovery_filter(v, cbf_tw)
            tw = Twist()
            tw.angular.z = cbf_tw.angular.z
            tw.linear.x = cbf_tw.linear.x
            tw.linear.y = cbf_tw.linear.y
            tw.linear.z = self._depth_hold_velocity()
            self.h_pub.publish(Float64(data=float(self.current_h)))
            self.get_logger().warning(
                '[CBF_FILTER] no_cbf_active fallback publishes planner velocities with depth hold')
            self._publish_cmd(v, cbf_tw, tw, 'depth_hold_only')

    def _publish_cmd(self, raw, cbf_tw, tw, mode):
        self.cmd_pub.publish(tw)
        thrusters = ','.join(f'{value:.2f}' for value in self.latest_thruster_inputs)
        position_text = 'unknown'
        yaw_text = 'unknown'
        if self.vehicle_pose is not None:
            position_text = (
                f'({self.vehicle_pose.x:.3f},{self.vehicle_pose.y:.3f},'
                f'{self.vehicle_pose.z:.3f})')
            yaw_text = f'{self.yaw:.3f}'
        self.get_logger().info(
            '[CBF_FILTER] '
            f'cmd_topic={self.cmd_vel_topic} planner_topic={self.planner_cmd_topic} mode={mode} '
            f'action={self.last_cbf_action} h={self.current_h:.3f} '
            f'constraints={len(self.obstacle_constraints)} source={self.closest_source} '
            f'nearest={self.closest_obstacle_distance:.3f} safety_radius={self.R_o:.2f} '
            f'spatial_memory={len(self.spatial_memory_points)} '
            f'min_clearance={self.minimum_clearance:.2f} imminent_distance={self.imminent_collision_distance:.2f} '
            f'emergency_distance={self.emergency_collision_distance:.2f} hover_reason={self.hover_lock_reason} '
            f'hover_score={self.hover_oscillation_score:.2f} '
            f'measured_planar={self.current_planar_speed:.3f} '
            f'planner_cmd=({raw.linear.x:.3f},{raw.linear.y:.3f},{raw.linear.z:.3f};{raw.angular.z:.3f}) '
            f'cbf_cmd=({cbf_tw.linear.x:.3f},{cbf_tw.linear.y:.3f},{cbf_tw.linear.z:.3f};{cbf_tw.angular.z:.3f}) '
            f'final_cmd=({tw.linear.x:.3f},{tw.linear.y:.3f},{tw.linear.z:.3f};{tw.angular.z:.3f}) '
            f'recovery={self.cbf_recovery_mode}')
        self.get_logger().info(
            '[CBF] '
            f'obstacle_count={self.scg_obstacle_count} gap_count={self.scg_gap_count} '
            f'selected_gap_angle={self.scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.scg_selected_gap_width:.2f} '
            f'context_parameter_h={self.scg_h:.3f} cbf_h={self.current_h:.3f} '
            f'planner_cmd=({raw.linear.x:.3f},{raw.linear.y:.3f},{raw.linear.z:.3f};{raw.angular.z:.3f}) '
            f'CBF_cmd=({cbf_tw.linear.x:.3f},{cbf_tw.linear.y:.3f},{cbf_tw.linear.z:.3f};{cbf_tw.angular.z:.3f}) '
            f'final_cmd=({tw.linear.x:.3f},{tw.linear.y:.3f},{tw.linear.z:.3f};{tw.angular.z:.3f}) '
            f'action={self.last_cbf_action} '
            f'nearest_obstacle={self.closest_obstacle_distance:.3f} recovery={self.cbf_recovery_mode}')
        self.get_logger().info(
            '[PID] '
            f'obstacle_count={self.scg_obstacle_count} gap_count={self.scg_gap_count} '
            f'selected_gap_angle={self.scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.scg_selected_gap_width:.2f} '
            f'context_parameter_h={self.scg_h:.3f} '
            f'safe_velocity_input=({tw.linear.x:.3f},{tw.linear.y:.3f},{tw.linear.z:.3f}) '
            f'safe_yaw_rate={tw.angular.z:.3f} '
            f'current_velocity={self.current_planar_speed:.3f} '
            f'hover_depth_target={self.target_depth:.3f} '
            f'PID_output_linear=({self.latest_pid_output.linear.x:.3f},'
            f'{self.latest_pid_output.linear.y:.3f},{self.latest_pid_output.linear.z:.3f}) '
            f'PID_output_angular_z={self.latest_pid_output.angular.z:.3f}')
        self.get_logger().info(
            '[THRUSTER_COMMANDS] '
            f'obstacle_count={self.scg_obstacle_count} gap_count={self.scg_gap_count} '
            f'selected_gap_angle={self.scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.scg_selected_gap_width:.2f} '
            f'context_parameter_h={self.scg_h:.3f} '
            f'wrench=({self.latest_thruster_wrench.force.x:.3f},'
            f'{self.latest_thruster_wrench.force.y:.3f},'
            f'{self.latest_thruster_wrench.force.z:.3f};'
            f'{self.latest_thruster_wrench.torque.x:.3f},'
            f'{self.latest_thruster_wrench.torque.y:.3f},'
            f'{self.latest_thruster_wrench.torque.z:.3f}) '
            f'thruster_commands=[{thrusters}] current_position={position_text} '
            f'current_yaw={yaw_text} current_velocity={self.current_planar_speed:.3f}')
        self.get_logger().info(
            '[REXROV_DYNAMICS] '
            f'obstacle_count={self.scg_obstacle_count} gap_count={self.scg_gap_count} '
            f'selected_gap_angle={self.scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.scg_selected_gap_width:.2f} '
            f'context_parameter_h={self.scg_h:.3f} current_position={position_text} '
            f'current_yaw={yaw_text} current_velocity={self.current_planar_speed:.3f} '
            f'safe_velocity=({tw.linear.x:.3f},{tw.linear.y:.3f},{tw.linear.z:.3f})')
        self.get_logger().info(
            '[RECOVERY] '
            f'obstacle_count={self.scg_obstacle_count} gap_count={self.scg_gap_count} '
            f'selected_gap_angle={self.scg_selected_gap_angle:.3f} '
            f'selected_gap_width={self.scg_selected_gap_width:.2f} '
            f'context_parameter_h={self.scg_h:.3f} mode={self.cbf_recovery_mode} '
            f'action={self.last_cbf_action} current_velocity={self.current_planar_speed:.3f} '
            f'cbf_velocity=({cbf_tw.linear.x:.3f},{cbf_tw.linear.y:.3f},{cbf_tw.linear.z:.3f};{cbf_tw.angular.z:.3f}) '
            f'final_velocity=({tw.linear.x:.3f},{tw.linear.y:.3f},{tw.linear.z:.3f};{tw.angular.z:.3f}) '
            f'near_zero_elapsed={self.final_velocity_recovery_elapsed:.2f} '
            f'final_recovery_active={self.final_velocity_recovery_active} '
            f'recovery_heading={self.final_velocity_recovery_heading:.3f} '
            f'sonar_sweep={self.final_velocity_recovery_scan_requested} '
            f'forward_safe={self.final_velocity_recovery_forward_safe} '
            f'retry_gap_selection=True '
            f'nearest_obstacle={self.closest_obstacle_distance:.3f}')

    def control_loop(self):
        self._refresh_pose_state()
        if self.vehicle_pose is None:
            return

        hover_error = self.target_depth - self.vehicle_pose.z
        self.get_logger().info(
            f'[HOVER_CONTROL] target_depth={self.target_depth:.3f} current_z={self.vehicle_pose.z:.3f} '
            f'error={hover_error:.3f} vertical_cmd={self.last_vertical_cmd:.3f}')

        if self._planner_cmd_is_fresh():
            self._log_planner_cmd()
        else:
            self.get_logger().info(
                '[PLANNER_CMD] planner_reference=stale_or_missing '
                'external planner is reference/fallback only; SPD2C will plan from SCG')

        self._update_point_cloud_obstacles()
        self._update_obstacle_state()
        self._update_scg_stage()
        desired = self._spd2c_command()
        self.process_data(desired)


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
