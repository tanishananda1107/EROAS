#!/usr/bin/env python3
"""
just_cbf_full.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
Uses scipy.optimize.minimize (SLSQP) instead of cvxpy.
"""

from collections import deque

import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry

import message_filters


class CBFControlNode(Node):
    def __init__(self):
        super().__init__('just_cbf')

        self.vehicle_pose = None
        self.radius = 20.0
        self.filtered_points = np.empty((0, 3))

        self.closest_obstacle_distance = float('inf')
        self.kappa = 10.0
        self.R_o = 3.0
        self.mass = 1863
        self.I_z = 691.23

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.last_h = float('inf')
        self.dh = 0.0
        self.last_u = self.last_v = self.last_w = self.last_r = 0.001
        self.consts_1 = None
        self.h_dot_t = None

        self.current_Th = None
        self.prev_Th = None
        self.current_h = float('inf')
        self.current_u = self.current_v = self.current_w = self.current_r = 0.0

        # Dynamic model coefficients
        self.X_dot_u = 779.79;  self.Y_dot_v = 1222.0; self.Z_dot_w = 3959.9; self.N_dot_r = 224.32
        self.X_u = -74.82;      self.Y_v = -69.48;      self.Z_w = -782.4;     self.N_r = -105.0
        self.X_uu = -748.22;    self.Y_vv = -992.53;    self.Z_ww = -1821.01;  self.N_rr = -523.27

        self.T_alg = np.zeros(3)   # [Tx, Ty, Tpsi]
        self.T_z = self.T_roll = self.T_pitch = 0.0

        # Publishers
        self.optimized_control_pub = self.create_publisher(
            WrenchStamped, '/rexrov2/thruster_manager/input_stamped', 10)

        # Synchronized subscribers
        thrust_sub = message_filters.Subscriber(self, WrenchStamped, '/rexrov2/thruster_manager/input_stamped_1')
        pose_sub   = message_filters.Subscriber(self, Odometry, '/rexrov2/pose_gt')
        self.at_sync = message_filters.ApproximateTimeSynchronizer(
            [thrust_sub, pose_sub], queue_size=10, slop=0.03)
        self.at_sync.registerCallback(self.callback_all)

        self.create_subscription(PointCloud2, '/rexrov2/point_cloud', self.point_cloud_callback, 10)

        self.get_logger().info('just_cbf_full Node started')

    # ------------------------------------------------------------------
    def callback_all(self, thrust_input: WrenchStamped, pose: Odometry):
        self.get_logger().info('all Data received..')
        self.pose_callback(pose)
        self.control_input_callback(thrust_input)
        self.process_data()

    def pose_callback(self, msg: Odometry):
        self.last_u, self.last_v, self.last_w, self.last_r = (
            self.current_u, self.current_v, self.current_w, self.current_r)
        self.vehicle_pose = msg.pose.pose.position
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z

        vxx, vyy, vzz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
        if len(self.filtered_points) > 0:
            d = np.linalg.norm(self.filtered_points - np.array([vxx, vyy, vzz]), axis=1)
            s = float(np.min(d)) if d.size > 0 else float('inf')
            if s > self.radius:
                s = float('inf')
        else:
            s = float('inf')
        self.closest_obstacle_distance = s
        self.current_h = s - self.R_o

    def point_cloud_callback(self, msg: PointCloud2):
        self.last_h = self.current_h
        if self.vehicle_pose is None:
            self.get_logger().info('Vehicle pose not yet received')
            return
        pc_data = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        new_points_float = np.array(list(pc_data))
        new_points = np.round(new_points_float).astype(int) if new_points_float.size > 0 else np.empty((0, 3))

        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
        fp = self.filtered_points
        if len(fp) > 0 and len(new_points) > 0:
            all_points = np.vstack((fp, new_points))
        elif len(new_points) > 0:
            all_points = new_points
        elif len(fp) > 0:
            all_points = fp
        else:
            all_points = np.empty((0, 3))

        if len(all_points) > 0:
            all_points = np.unique(all_points, axis=0)
            self.get_logger().info(f'len of all_points: {len(self.filtered_points)}')
            d = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
            self.filtered_points = all_points[d <= self.radius]
        else:
            self.filtered_points = np.empty((0, 3))

    def control_input_callback(self, data: WrenchStamped):
        self.prev_Th = self.current_Th
        self.T_alg = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.torque.z])
        self.T_z     = data.wrench.force.z
        self.T_roll  = data.wrench.torque.x
        self.T_pitch = data.wrench.torque.y

    def compute_state_derivative_of_h(self):
        self.current_time = self.get_clock().now().nanoseconds / 1e9
        dh = self.current_h - self.last_h
        du = self.current_u - self.last_u
        dv = self.current_v - self.last_v
        dw = self.current_w - self.last_w
        dr = self.current_r - self.last_r
        self.dh = dh
        if du == 0 or dv == 0 or dw == 0 or dr == 0:
            return np.array([float('inf'), float('inf'), float('inf')])
        return np.array([dh / du, dh / dv, dh / dr])

    def compute_h_dot(self, T, h_state_dot):
        Tx, Ty, Tpsi = T
        u_dot = (Tx + (self.X_u + self.X_uu * np.abs(self.current_u)) * self.current_u
                 + (self.mass * self.current_r * self.current_v)
                 - (self.Y_dot_v * self.current_r * self.current_v)
                 / (self.mass - self.X_dot_u))
        v_dot = (Ty + (self.Y_v + self.Y_vv * np.abs(self.current_v)) * self.current_v
                 + (self.mass * self.current_u * self.current_r)
                 + (self.X_dot_u * self.current_r * self.current_u)
                 / (self.mass - self.Y_dot_v))
        r_dot = (Tpsi + (self.N_r + self.N_rr * np.abs(self.current_r)) * self.current_r
                 + ((self.Y_dot_v - self.X_dot_u) * self.current_u * self.current_v)
                 ) / (self.I_z - self.N_dot_r)
        x_dot = np.array([u_dot, v_dot, r_dot])
        self.h_dot_t = np.dot(h_state_dot, x_dot)
        return self.h_dot_t

    def cbf_optimization(self, T_alg):
        def objective(T):
            return np.linalg.norm(T - T_alg) ** 2

        def constraint(T):
            h_state_dot = self.compute_state_derivative_of_h()
            h_dot = self.compute_h_dot(T, h_state_dot)
            return h_dot + self.kappa * self.current_h

        constraints = {'type': 'ineq', 'fun': constraint}
        options = {'disp': False, 'maxiter': 100}
        result = minimize(objective, T_alg, constraints=constraints, method='SLSQP', options=options)

        self.get_logger().info(f'h_dot: {self.h_dot_t}')
        self.get_logger().info(f'current_h: {self.current_h}')

        if result.success:
            return result.x
        else:
            return T_alg

    def process_data(self):
        if self.prev_Th is not None:
            if self.current_h != float('inf'):
                T_safe = self.cbf_optimization(self.T_alg)
                self.current_Th = T_safe
                self.get_logger().info('publishing optimised')
                self.publish_safe_control_input(T_safe)
            else:
                self.publish_safe_control_input(self.T_alg)
                self.current_Th = self.T_alg
        else:
            self.get_logger().info('1st command')
            self.publish_safe_control_input(self.T_alg)
            self.current_Th = self.T_alg

    def publish_safe_control_input(self, T_safe):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = '/rexrov2/base_link'
        wrench_msg.wrench.force.x  = float(T_safe[0])
        wrench_msg.wrench.force.y  = float(T_safe[1])
        wrench_msg.wrench.force.z  = self.T_z
        wrench_msg.wrench.torque.x = self.T_roll
        wrench_msg.wrench.torque.y = self.T_pitch
        wrench_msg.wrench.torque.z = float(self.T_alg[2])
        self.optimized_control_pub.publish(wrench_msg)
        self.get_logger().info(
            f'change in optimized: Tx={T_safe[0] - self.T_alg[0]:.3f}, '
            f'Ty={T_safe[1] - self.T_alg[1]:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = CBFControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
