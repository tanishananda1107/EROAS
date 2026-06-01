#!/usr/bin/env python3
"""
just_cbf_prev.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
"""

import numpy as np
from scipy.optimize import minimize
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


class CBFControlNode(Node):
    def __init__(self):
        super().__init__('just_cbf')

        # Publishers
        self.optimized_control_pub = self.create_publisher(
            WrenchStamped, '/rexrov2/thruster_manager/input_stamped', 10)
        self.cbf_on_pub = self.create_publisher(
            WrenchStamped, '/rexrov2/cbf_on', 10)

        # Subscribers
        self.create_subscription(WrenchStamped, '/rexrov2/thruster_manager/input_stamped_1',
                                 self.control_input_callback, 10)
        self.create_subscription(PointCloud2, '/rexrov2/point_cloud', self.point_cloud_callback, 10)
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)

        self.vehicle_pose = None
        self.radius = 10.0
        self.filtered_points = []

        self.closest_obstacle_distance = float('inf')
        self.kappa = 1.0
        self.R_o   = 1.0
        self.mass  = 1863
        self.I_z   = 691.23

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.last_h    = float('inf')
        self.last_u = self.last_v = self.last_w = self.last_r = 0.001
        self.current_h = float('inf')
        self.current_u = self.current_v = self.current_w = self.current_r = None

        # Dynamic model coefficients
        self.X_dot_u = 779.79;  self.Y_dot_v = 1222.0; self.Z_dot_w = 3959.9; self.N_dot_r = 224.32
        self.X_u = -74.82;      self.Y_v = -69.48;      self.Z_w = -782.4;     self.N_r = -105.0
        self.X_uu = -748.22;    self.Y_vv = -992.53;    self.Z_ww = -1821.01;  self.N_rr = -523.27

        self.T_alg   = np.zeros(4)
        self.T_roll  = self.T_pitch = 0.0
        self.obstacle_history = deque(maxlen=10000)

    # ------------------------------------------------------------------
    def pose_callback(self, msg: Odometry):
        self.last_u = self.current_u or 0.001
        self.last_v = self.current_v or 0.001
        self.last_w = self.current_w or 0.001
        self.last_r = self.current_r or 0.001
        self.vehicle_pose = msg.pose.pose.position
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z

    def point_cloud_callback(self, msg: PointCloud2):
        self.last_h = self.current_h
        if self.vehicle_pose is None:
            self.get_logger().info('Vehicle pose not yet received')
            return
        pc_data = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        new_points = np.array(list(pc_data))
        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z

        if len(self.filtered_points) > 0:
            all_points = np.vstack((np.array(self.filtered_points), new_points))
        else:
            all_points = new_points

        distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
        smallest = float(np.min(distances)) if distances.size > 0 else float('inf')
        if smallest > self.radius:
            smallest = float('inf')
        self.filtered_points = all_points[distances <= self.radius].tolist()

        self.get_logger().info(f'Total stored points: {len(self.filtered_points)}')
        self.get_logger().info(f'Smallest distance: {smallest}')
        self.closest_obstacle_distance = smallest
        self.get_logger().info(f'closest_obstacle_distance: {self.closest_obstacle_distance}')
        self.current_h = self.closest_obstacle_distance - self.R_o

    def control_input_callback(self, data: WrenchStamped):
        self.T_alg = np.array([
            data.wrench.force.x, data.wrench.force.y,
            data.wrench.force.z, data.wrench.torque.z,
        ])
        self.T_roll  = data.wrench.torque.x
        self.T_pitch = data.wrench.torque.y
        self.process_data()

    def compute_h(self):
        return self.closest_obstacle_distance - self.R_o

    def compute_state_derivative_of_h(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dh = self.current_h - self.last_h
        du = (self.current_u or 0) - self.last_u
        dv = (self.current_v or 0) - self.last_v
        dw = (self.current_w or 0) - self.last_w
        dr = (self.current_r or 0) - self.last_r
        self.get_logger().info(f'h:{self.current_h}, dh:{dh}, du:{du}, dv:{dv}, dw:{dw}')
        if du == 0 or dv == 0 or dw == 0 or dr == 0:
            return np.array([float('inf')] * 4)
        return np.array([dh / du, dh / dv, dh / dw, dh / dr])

    def compute_h_dot(self, T, h_state_dot):
        Tx, Ty, Tz, Tpsi = T
        u_dot = (Tx + (self.X_u + self.X_uu * np.abs(Tx)) * Tx
                 + self.mass * Tpsi * Ty - self.Y_dot_v * Tpsi * Ty) / (self.mass - self.X_dot_u)
        v_dot = (Ty + (self.Y_v + self.Y_vv * np.abs(Ty)) * Ty
                 + self.mass * Tpsi * Tx + self.X_dot_u * Tpsi * Tx) / (self.mass - self.Y_dot_v)
        w_dot = (Tz + (self.Z_w + self.Z_ww * np.abs(Tz)) * Tz
                 - (self.mass - self.mass * 9.81)) / (self.mass - self.Z_dot_w)
        r_dot = (Tpsi + (self.N_r + self.N_rr * np.abs(Tpsi)) * Tpsi
                 + (self.Y_dot_v - self.X_dot_u) * Tx * Ty) / (self.I_z - self.N_dot_r)
        return np.dot(h_state_dot, np.array([u_dot, v_dot, w_dot, r_dot]))

    def cbf_optimization(self, T_alg):
        def objective(T): return np.linalg.norm(T - T_alg) ** 2
        def constraint(T):
            return self.compute_h_dot(T, self.compute_state_derivative_of_h()) + self.kappa * self.current_h
        result = minimize(objective, T_alg,
                          constraints={'type': 'ineq', 'fun': constraint},
                          method='SLSQP', options={'disp': False, 'maxiter': 1000})
        if result.success:
            return result.x
        self.get_logger().warn('Optimization failed, using original input.')
        return T_alg

    def process_data(self):
        T_safe = self.cbf_optimization(self.T_alg)
        self.publish_safe_control_input(T_safe)

    def publish_safe_control_input(self, T_safe):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = '/rexrov2/base_link'
        wrench_msg.wrench.force.x  = T_safe[0]
        wrench_msg.wrench.force.y  = T_safe[1]
        wrench_msg.wrench.force.z  = T_safe[2]
        wrench_msg.wrench.torque.x = self.T_roll
        wrench_msg.wrench.torque.y = self.T_pitch
        wrench_msg.wrench.torque.z = T_safe[3]
        self.optimized_control_pub.publish(wrench_msg)
        self.get_logger().info(
            f'Published optimized: Tx={T_safe[0]:.3f}, Ty={T_safe[1]:.3f}, '
            f'Tz={T_safe[2]:.3f}, Tpsi={T_safe[3]:.3f}')


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
