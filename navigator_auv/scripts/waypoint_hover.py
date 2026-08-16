#!/usr/bin/env python3
"""waypoint_hover.py — sequential state-machine navigation script with FLS FOV visualization.

Executes a fixed sequence of FORWARD and TURN steps to trace the S-curve
path through the World A obstacle field.  Each step transitions either on
heading tolerance (TURN) or elapsed wall-time (FORWARD).

Additionally publishes RViz markers showing the FLS field-of-view cone in real-time,
and logs which obstacles enter the FOV as the vehicle moves.

Spawn at: x=8, y=6, z=-59, yaw=1.5708rad/90deg (south of cube_6, the tall
pillar, pointing north/+y so it's dead ahead within sonar range). Launch
with (the launch file's `yaw` arg is radians, matching WORLD_A's own
spawn dict -- passing a bare "90" spawns at ~116.6deg, i.e. 90rad mod
2*pi, not 90deg):
  ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py \
      world_name:=world_a start_waypoint_hover:=true \
      x:=8 y:=6 z:=-59 yaw:=1.5708

The previous x≈5,y≈15,z=-50,yaw≈0 docstring spawn was never validated: at
z=-50 the vehicle flies ~9-10m above the real obstacle field (obstacles
sit at z=-58..-60 per obstacle_avoidance.world), so no obstacle ever
entered the FLS FOV (confirmed via headless run: zero [FOV] detections
over a full run). The STEPS below were re-derived offline against the
obstacles' real oriented-box geometry (ported from velocity_cbf.py's
OBSTACLE_MODEL_BOXES/WORLD_A_OBSTACLE_SPECS) to thread past cube_6,
cube_5 and cube_6_1 with a few meters of real clearance, tracing an
S-curve comparable to the paper's Figure 8.

FLS Sensor Parameters (Blueview P900):
  - Horizontal FOV: 90°, Vertical FOV: 90°
  - Min range: 0.2m, Max range: 12m
  - Sensor link: blueview_p900_link
  - Optical frame: blueview_p900_forward_sonar_optical_link
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer

# ---------------------------------------------------------------------------
# FLS Sensor Parameters (Blueview P900)
# ---------------------------------------------------------------------------
FLS_HORZ_FOV = math.radians(90.0)      # 90° horizontal FOV
FLS_VERT_FOV = math.radians(90.0)      # 90° vertical FOV
FLS_MIN_RANGE = 0.2                    # meters
FLS_MAX_RANGE = 12.0                   # meters (10m default + 2m margin)
FLS_SENSOR_FRAME = "blueview_p900_link"
FLS_OPTICAL_FRAME = "blueview_p900_forward_sonar_optical_link"

# Known obstacle positions in World A (from obstacle_avoidance.world)
# Format: (name, x, y, z, radius_approx)
OBSTACLES = [
    ("cube_5", 10.0, 40.0, -60.0, 1.5),
    ("cube_6", 8.0, 26.0, -60.0, 2.0),       # tall pillar (spatial anchor)
    ("cube_6_1", 22.0, 37.0, -60.0, 1.5),
    ("cube_7", 32.0, 40.0, -60.0, 1.5),
    ("cube_9_1", 42.0, 66.0, -58.0, 1.0),
    ("cube_9_2", 12.0, 68.0, -58.0, 1.0),
    ("moving_obs", 20.0, 58.0, -60.0, 1.5),
    ("moving_obs_1", 47.0, 55.0, -60.0, 1.5),
]

# ---------------------------------------------------------------------------
# Step definitions.  Each step is a dict:
#   type   : 'fwd'  — publish linear.x = SPEED for DURATION seconds
#            'turn' — rotate by ANGLE_DEG at YAW_RATE rad/s, then stop
#   For 'fwd' :  duration (s)
#   For 'turn': angle_deg (+CCW / -CW),  yaw_rate (rad/s)
# ---------------------------------------------------------------------------
SPEED = 0.40          # m/s forward speed for all FWD steps
YAW_RATE = 0.20       # rad/s rotation rate for all TURN steps

STEPS = [
    # 1. Close in on cube_6 (the tall pillar) from the south, straight ahead
    {'type': 'fwd',  'duration': 30.0, 'label': 'fwd — approach cube_6 pillar from the south'},
    # 2. Turn left ~62 deg to curve around the pillar's west side
    {'type': 'turn', 'angle_deg': 62.0, 'label': 'turn L 62deg — curve west around the pillar'},
    # 3. Forward past the pillar, climbing over its top (Fig. 8b hump, rising leg)
    {'type': 'fwd',  'duration': 15.0, 'vz': 0.15,
     'label': 'fwd — past pillar, climbing over its top'},
    # 4. Turn right ~75 deg to resume heading north
    {'type': 'turn', 'angle_deg': -75.0, 'label': 'turn R 75deg — resume heading north'},
    # 5. Push north toward the cube_5 / cube_6_1 pair, still climbing
    {'type': 'fwd',  'duration': 18.0, 'vz': 0.10,
     'label': 'fwd — north toward cube_5/cube_6_1, still climbing'},
    # 6. Turn left ~25 deg to thread the gap between them
    {'type': 'turn', 'angle_deg': 25.0, 'label': 'turn L 25deg — thread the gap'},
    # 7. Forward through the gap, descending back down (Fig. 8b hump, falling leg)
    {'type': 'fwd',  'duration': 12.0, 'vz': -0.15,
     'label': 'fwd — through gap, descending'},
    # 8. Turn right ~20 deg for the final approach
    {'type': 'turn', 'angle_deg': -20.0, 'label': 'turn R 20deg — final approach'},
    # 9. Forward to final resting position, leveling off depth
    {'type': 'fwd',  'duration': 8.0, 'vz': -0.20,
     'label': 'fwd — final stop position, leveling off'},
]

CMD_TOPIC  = "/rexrov2/cmd_vel"
POSE_TOPIC = "/rexrov2/pose_gt"
# ---------------------------------------------------------------------------


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SequentialNav(Node):
    def __init__(self):
        super().__init__('waypoint_hover',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.pub  = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.fov_marker_pub = self.create_publisher(MarkerArray, "/fls_fov_marker", 10)
        
        self.pose = None
        self.yaw  = 0.0
        self.pose_logged = False
        
        # TF listener for sensor frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, POSE_TOPIC, self._on_pose, qos_profile_sensor_data)

        self.step_idx   = 0
        self.step_start = None   # wall time float
        self.turn_start_yaw  = None
        self.turn_target_yaw = None
        
        # FOV visualization tracking
        self.last_fov_update = 0.0
        self.fov_update_interval = 0.1  # Update FOV marker every 100ms
        self.obstacles_in_fov_log = set()  # Track which obstacles we've already logged

        self.create_timer(0.05, self._loop)   # 20 Hz main navigation loop
        self.create_timer(self.fov_update_interval, self._update_fov_marker)  # FOV visualization
        
        self.get_logger().info(
            f"SequentialNav ready — {len(STEPS)} steps loaded.\n"
            f"FLS FOV: {math.degrees(FLS_HORZ_FOV):.1f}° H x {math.degrees(FLS_VERT_FOV):.1f}° V\n"
            f"FLS Range: {FLS_MIN_RANGE}-{FLS_MAX_RANGE}m")

    # ------------------------------------------------------------------
    def _on_pose(self, msg):
        self.pose = msg.pose.pose
        self.yaw  = yaw_from_quat(msg.pose.pose.orientation)

    # ------------------------------------------------------------------
    def _log_pose(self, label):
        if self.pose is None:
            return
        p = self.pose.position
        self.get_logger().info(
            f"[STEP] {label} | "
            f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f} "
            f"yaw={math.degrees(self.yaw):.1f}°")

    # ------------------------------------------------------------------
    def _is_point_in_fov_cone(self, obs_pos, vehicle_pos, vehicle_yaw):
        """Check if obstacle center falls within FLS FOV cone."""
        # Vector from vehicle to obstacle (in world frame)
        dx = obs_pos[0] - vehicle_pos[0]
        dy = obs_pos[1] - vehicle_pos[1]
        dz = obs_pos[2] - vehicle_pos[2]
        
        # Distance to obstacle
        dist_xy = math.sqrt(dx*dx + dy*dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Check range
        if dist_3d < FLS_MIN_RANGE or dist_3d > FLS_MAX_RANGE:
            return False
        
        # Bearing to obstacle relative to vehicle heading (yaw)
        bearing = math.atan2(dy, dx)
        heading_diff = wrap(bearing - vehicle_yaw)
        
        # Check horizontal FOV
        if abs(heading_diff) > FLS_HORZ_FOV / 2.0:
            return False
        
        # Check vertical FOV (simple: elevation angle must be within FOV)
        if dist_xy < 0.1:
            dist_xy = 0.1  # Avoid division by zero
        elevation = math.atan2(dz - 0.6, dist_xy)  # 0.6m is approx sonar height above center
        
        if abs(elevation) > FLS_VERT_FOV / 2.0:
            return False
        
        return True

    # ------------------------------------------------------------------
    def _check_obstacles_in_fov(self):
        """Check which obstacles are currently in FLS FOV and log new detections."""
        if self.pose is None:
            return
        
        vehicle_pos = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
        
        for obs_name, obs_x, obs_y, obs_z, _ in OBSTACLES:
            obs_pos = (obs_x, obs_y, obs_z)
            
            if self._is_point_in_fov_cone(obs_pos, vehicle_pos, self.yaw):
                # Obstacle is in FOV
                if obs_name not in self.obstacles_in_fov_log:
                    self.obstacles_in_fov_log.add(obs_name)
                    p = self.pose.position
                    self.get_logger().info(
                        f"[FOV] {obs_name} DETECTED in FLS cone at "
                        f"vehicle pos (x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f})")
            else:
                # Obstacle left FOV
                if obs_name in self.obstacles_in_fov_log:
                    self.obstacles_in_fov_log.discard(obs_name)

    # ------------------------------------------------------------------
    def _build_fov_cone_marker(self):
        """Build a TRIANGLE_LIST marker representing the FLS FOV cone."""
        marker = Marker()
        marker.header.frame_id = FLS_OPTICAL_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fls_fov"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Semi-transparent cyan for FOV visualization
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.3  # 30% transparency
        
        # Generate cone points
        # Approximate cone as a pyramid with circular base
        num_segments = 16  # Number of segments around the cone
        
        # Apex of cone (at sensor, origin of optical frame)
        apex = Point(x=0.0, y=0.0, z=0.0)
        
        # Radii at near and far ranges
        near_radius_h = FLS_MIN_RANGE * math.tan(FLS_HORZ_FOV / 2.0)
        near_radius_v = FLS_MIN_RANGE * math.tan(FLS_VERT_FOV / 2.0)
        far_radius_h = FLS_MAX_RANGE * math.tan(FLS_HORZ_FOV / 2.0)
        far_radius_v = FLS_MAX_RANGE * math.tan(FLS_VERT_FOV / 2.0)
        
        # Generate near and far circle points
        near_points = []
        far_points = []
        
        for i in range(num_segments):
            angle = 2.0 * math.pi * i / num_segments
            # Near circle
            near_x = near_radius_h * math.cos(angle)
            near_y = near_radius_v * math.sin(angle)
            near_z = FLS_MIN_RANGE
            near_points.append(Point(x=near_x, y=near_y, z=near_z))
            
            # Far circle
            far_x = far_radius_h * math.cos(angle)
            far_y = far_radius_v * math.sin(angle)
            far_z = FLS_MAX_RANGE
            far_points.append(Point(x=far_x, y=far_y, z=far_z))
        
        # Build triangles
        # 1. Apex to near circle (cone tip)
        for i in range(num_segments):
            next_i = (i + 1) % num_segments
            marker.points.append(apex)
            marker.points.append(near_points[i])
            marker.points.append(near_points[next_i])
        
        # 2. Side surface (near to far)
        for i in range(num_segments):
            next_i = (i + 1) % num_segments
            # First triangle
            marker.points.append(near_points[i])
            marker.points.append(far_points[i])
            marker.points.append(near_points[next_i])
            # Second triangle
            marker.points.append(near_points[next_i])
            marker.points.append(far_points[i])
            marker.points.append(far_points[next_i])
        
        # 3. Far circle cap
        for i in range(num_segments):
            next_i = (i + 1) % num_segments
            marker.points.append(far_points[i])
            marker.points.append(far_points[next_i])
            marker.points.append(apex)
        
        # Set scale (marker mesh points don't use scale, but set for consistency)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        return marker

    # ------------------------------------------------------------------
    def _update_fov_marker(self):
        """Update and publish the FOV marker."""
        if self.pose is None:
            return
        
        marker_array = MarkerArray()
        
        try:
            # Try to build marker in FLS optical frame
            marker = self._build_fov_cone_marker()
            marker_array.markers.append(marker)
        except Exception as e:
            self.get_logger().warn(f"Failed to build FOV marker: {e}")
        
        # Check for obstacles in current FOV
        self._check_obstacles_in_fov()
        
        # Publish marker array
        self.fov_marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    def _loop(self):
        if self.pose is None:
            return

        # Log initial spawned pose on first available data
        if not self.pose_logged:
            self.pose_logged = True
            p = self.pose.position
            self.get_logger().info(
                f"[SPAWNED] x={p.x:.2f} y={p.y:.2f} z={p.z:.2f} "
                f"yaw={math.degrees(self.yaw):.1f}°")

        if self.step_idx >= len(STEPS):
            self.pub.publish(Twist())
            return

        step = STEPS[self.step_idx]
        now  = time.monotonic()

        # ---- initialise step ----
        if self.step_start is None:
            self.step_start = now
            self._log_pose(f"START step {self.step_idx}: {step['label']}")
            if step['type'] == 'turn':
                self.turn_start_yaw  = self.yaw
                delta = math.radians(step['angle_deg'])
                self.turn_target_yaw = wrap(self.yaw + delta)

        cmd = Twist()

        # ---- FORWARD step ----
        if step['type'] == 'fwd':
            elapsed = now - self.step_start
            if elapsed >= step['duration']:
                self._advance_step()
            else:
                cmd.linear.x = SPEED
                cmd.linear.z = step.get('vz', 0.0)

        # ---- TURN step ----
        elif step['type'] == 'turn':
            remaining = wrap(self.turn_target_yaw - self.yaw)
            tol = math.radians(2.0)   # 2° tolerance

            if abs(remaining) <= tol:
                self._advance_step()
            else:
                direction = 1.0 if remaining > 0 else -1.0
                cmd.angular.z = direction * YAW_RATE

        self.pub.publish(cmd)

    # ------------------------------------------------------------------
    def _advance_step(self):
        self._log_pose(f"END   step {self.step_idx}: {STEPS[self.step_idx]['label']}")
        self.step_idx   += 1
        self.step_start  = None
        self.turn_start_yaw  = None
        self.turn_target_yaw = None
        if self.step_idx < len(STEPS):
            self.get_logger().info(
                f"[NEXT] → step {self.step_idx}: {STEPS[self.step_idx]['label']}")
        else:
            self.get_logger().info("[DONE] all steps complete — holding position")


def main():
    rclpy.init()
    node = SequentialNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
