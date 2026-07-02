#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf_transformations as tft
import time


class HoverPatrolNode(Node):
    def __init__(self):
        super().__init__('hover_patrol_node')

        # Declare parameters
        self.declare_parameter('target_depth', -56.0)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('max_vertical_speed', 0.6)
        self.declare_parameter('cruise_speed', 0.4)
        self.declare_parameter('turn_rate', 0.45)
        self.declare_parameter('obstacle_dist', 4.5)
        self.declare_parameter('clear_dist', 5.5)
        self.declare_parameter('min_obstacle_x', 1.0)
        self.declare_parameter('max_obstacle_x', 15.0)
        self.declare_parameter('obstacle_half_width', 1.8)
        self.declare_parameter('obstacle_half_height', 1.0)

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)
        self.obstacle_dist = float(self.get_parameter('obstacle_dist').value)
        self.clear_dist = float(self.get_parameter('clear_dist').value)
        self.min_obstacle_x = float(self.get_parameter('min_obstacle_x').value)
        self.max_obstacle_x = float(self.get_parameter('max_obstacle_x').value)
        self.obstacle_half_width = float(
            self.get_parameter('obstacle_half_width').value)
        self.obstacle_half_height = float(
            self.get_parameter('obstacle_half_height').value)

        # Subscribers
        self.pose_sub = self.create_subscription(
            Odometry,
            '/rexrov2/pose_gt',
            self.pose_callback,
            10
        )
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/rexrov2/blueview_p900_point_cloud',
            self.pc_callback,
            qos_profile_sensor_data
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        # State variables
        self.pose = None
        self.points = np.empty((0, 3))
        self.obstacle_detected = False
        self.closest_obstacle_dist = float('inf')
        self.turn_direction = 1.0  # 1 for left, -1 for right

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        # Debug logger timer (1 Hz)
        self.log_timer = self.create_timer(1.0, self.publish_debug_logs)

        self.get_logger().info("Hover Patrol Node initialized.")

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def pc_callback(self, msg):
        # Convert PointCloud2 to 2D numpy array
        pts_list = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts_list.append([float(p[0]), float(p[1]), float(p[2])])
        
        self.points = np.array(pts_list, dtype=float)

        # Process obstacle distance in front of the vehicle
        if len(self.points) > 0:
            # Ignore very close returns from the vehicle/sensor body, then look
            # for obstacles in the forward corridor.
            in_front = self.points[
                (self.points[:, 0] > self.min_obstacle_x) &
                (self.points[:, 0] < self.max_obstacle_x) &
                (self.points[:, 1] > -self.obstacle_half_width) &
                (self.points[:, 1] < self.obstacle_half_width) &
                (self.points[:, 2] > -self.obstacle_half_height) &
                (self.points[:, 2] < self.obstacle_half_height)
            ]
            if len(in_front) > 0:
                dists = np.linalg.norm(in_front, axis=1)
                self.closest_obstacle_dist = float(np.min(dists))
                
                # Check if closest point is on the left or right of the vehicle to choose turn direction
                closest_idx = np.argmin(dists)
                closest_y = in_front[closest_idx][1]
                # If obstacle is more to the left, turn right. If to the right, turn left.
                if closest_y > 0:
                    self.turn_direction = -1.0
                else:
                    self.turn_direction = 1.0

                if self.closest_obstacle_dist < self.obstacle_dist:
                    self.obstacle_detected = True
                elif self.closest_obstacle_dist > self.clear_dist:
                    self.obstacle_detected = False
            else:
                self.closest_obstacle_dist = float('inf')
                self.obstacle_detected = False
        else:
            self.closest_obstacle_dist = float('inf')
            self.obstacle_detected = False

    def control_loop(self):
        cmd = Twist()

        # 1. Height / Depth Hold
        if self.pose is not None:
            current_z = self.pose.position.z
            z_error = self.target_depth - current_z
            z_vel = self.depth_kp * z_error
            cmd.linear.z = float(np.clip(z_vel, -self.max_vertical_speed, self.max_vertical_speed))
        else:
            cmd.linear.z = 0.0

        # 2. Forward movement & obstacle avoidance patrol behavior
        if self.obstacle_detected:
            # Slow down forward movement and rotate to avoid
            cmd.linear.x = 0.08
            cmd.angular.z = self.turn_direction * self.turn_rate
        else:
            # Move forward at cruise speed
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = 0.0

        # Publish the command
        self.cmd_pub.publish(cmd)

    def publish_debug_logs(self):
        if self.pose is not None:
            pos = self.pose.position
            ori = self.pose.orientation
            _, _, yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            
            # Print debug information as requested
            self.get_logger().info(
                f"\n=== DEBUG LOG ===\n"
                f"Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}\n"
                f"Target Hover Height: {self.target_depth:.2f}\n"
                f"Velocity Command: linear.x={self.cruise_speed if not self.obstacle_detected else 0.08:.2f}, "
                f"linear.z={np.clip(self.depth_kp * (self.target_depth - pos.z), -self.max_vertical_speed, self.max_vertical_speed):.2f}, "
                f"angular.z={self.turn_direction * self.turn_rate if self.obstacle_detected else 0.0:.2f}\n"
                f"Publishing cmd_vel: True\n"
                f"Obstacle Detected: {self.obstacle_detected} (Distance: {self.closest_obstacle_dist:.2f}m)\n"
                f"================="
            )
        else:
            self.get_logger().info("Waiting for RexROV pose data...")


def main():
    rclpy.init()
    node = HoverPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
