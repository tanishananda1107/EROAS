#!/usr/bin/env python3
"""waypoint_hover.py
Standalone controller: flies the REXROV2 to each waypoint in WAYPOINTS,
in order, and HOVERS at each one for HOVER_SECONDS before moving to the
next. No SPD2C/CBF dependency -- this is a pure point-to-point + hover
shortcut.
Includes attitude hold (roll/pitch -> 0) baked in from the start, because
without it the vehicle drifts and can flip over during long runs.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---- EDIT THESE ----------------------------------------------------------
# World A waypoints from the old repo (eroas_world_a path, Figure 5)
# Spawn: x=24, y=55, z=-56, yaw=1.5708
# WP0 is skipped (it's the spawn point itself) — start from WP1 onwards
WAYPOINTS = [
    (28.0,  59.0, -56.0),
    (34.0,  62.0, -56.0),
    (44.0,  65.0, -56.0),
    (55.0,  66.0, -56.0),
    (62.0,  75.0, -56.0),
    (61.0,  88.0, -56.0),
    (55.0,  92.0, -56.0),
]
HOVER_SECONDS = 8.0
ARRIVAL_TOLERANCE = 2.5   # matches old repo waypoint_tolerance
CMD_TOPIC = "/rexrov2/cmd_vel"
POSE_TOPIC = "/rexrov2/pose_gt"
# ---------------------------------------------------------------------------
KP_X, MAX_X = 0.5, 1.0
KP_Z, MAX_Z = 0.6, 1.0
KP_YAW, MAX_YAW = 0.8, 0.4
KP_ATT, MAX_ATT = 0.5, 0.3


def clip(v, lo_or_mag, hi=None):
    if hi is None:
        hi = lo_or_mag
        lo_or_mag = -lo_or_mag
    return max(lo_or_mag, min(hi, v))


def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class WaypointHover(Node):
    def __init__(self):
        super().__init__(
            'waypoint_hover',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.sub = self.create_subscription(Odometry, POSE_TOPIC, self.on_pose, 10)
        self.pose = None
        self.wp_index = 0
        self.hover_until = None
        self.timer = self.create_timer(0.1, self.control_loop)

        # Waypoints: load from ROS param if available, else use module-level default
        default_waypoints = list(WAYPOINTS)
        waypoint_text = (
            self.get_parameter('waypoints').value
            if self.has_parameter('waypoints') else '')
        if waypoint_text:
            self.waypoints = [
                tuple(float(v) for v in wp.split(','))
                for wp in waypoint_text.split(';')
            ]
        else:
            self.waypoints = default_waypoints

        self.get_logger().info(f"WaypointHover started. {len(self.waypoints)} waypoints loaded.")

    def on_pose(self, msg):
        self.pose = msg.pose.pose

    def control_loop(self):
        if self.pose is None:
            return
        if self.wp_index >= len(self.waypoints):
            self.pub.publish(Twist())
            return
        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z
        roll, pitch, yaw = euler_from_quaternion(self.pose.orientation)
        tx, ty, tz = self.waypoints[self.wp_index]
        dx, dy, dz = tx - x, ty - y, tz - z
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        twist = Twist()
        if dist_3d <= ARRIVAL_TOLERANCE:
            now = self.get_clock().now()
            if self.hover_until is None:
                self.hover_until = now.nanoseconds + int(HOVER_SECONDS * 1e9)
                self.get_logger().info(
                    f"[ARRIVED] waypoint {self.wp_index} ({tx},{ty},{tz}) -- hovering {HOVER_SECONDS}s"
                )
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            if now.nanoseconds >= self.hover_until:
                self.wp_index += 1
                self.hover_until = None
                self.get_logger().info(f"[NEXT] advancing to waypoint index {self.wp_index}")
        else:
            target_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
            speed_factor = max(0.0, math.cos(yaw_err))
            twist.linear.x = clip(KP_X * dist_xy * speed_factor, MAX_X)
            twist.linear.z = clip(KP_Z * dz, MAX_Z)
            twist.angular.z = clip(KP_YAW * yaw_err, MAX_YAW)
            self.get_logger().info(
                f"[WP {self.wp_index}] target=({tx:.1f},{ty:.1f},{tz:.1f}) "
                f"current=({x:.2f},{y:.2f},{z:.2f}) dist={dist_3d:.2f} yaw_err={math.degrees(yaw_err):.1f}deg",
                throttle_duration_sec=1.0,
            )
        # Attitude hold: always active
        twist.angular.x = clip(-KP_ATT * roll, MAX_ATT)
        twist.angular.y = clip(-KP_ATT * pitch, MAX_ATT)
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = WaypointHover()
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
