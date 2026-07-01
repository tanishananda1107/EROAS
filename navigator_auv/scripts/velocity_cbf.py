#!/usr/bin/env python3
import math

import cvxpy as cp
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Float64


POINT_CLOUD_TOPIC = '/rexrov2/blueview_p900_point_cloud'


def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Twist, '/rexrov2/cmd_vel_1', self.vel_callback, 10)
        self.create_subscription(PointCloud2, POINT_CLOUD_TOPIC, self.point_cloud_callback, sensor_qos)
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)
        self.create_subscription(Float64, '/rexrov2/sonar/moving', self.sonar_callback, 10)
        self.h_pub = self.create_publisher(Float64, '/rexrov2/current_h', 10)
        self.closest_publisher = self.create_publisher(PointCloud2, '/rexrov2/closest_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        self.safe_distance = 0
        self.R_o = 2
        self.radius = 15
        self.kappa = 0.09
        self.kappa1 = 0.09

        self.filtered_points = np.empty((0, 3))
        self.current_pose = None
        self.current_vel = None
        self.point_cloud = None
        self.vehicle_pose = None
        self.current_h = float('inf')
        self.sonar_moving = False
        self.xy_cbf = False
        self.xz_cbf = False
        self.closest_points = []
        self.quaternion = None
        self.yaw = 0.0
        self.closest_obstacle_distance = float('inf')
        self.closest_point = None
        self.v_alg = Twist()

    def sonar_callback(self, msg):
        sonar_state = msg.data
        if sonar_state == 1:
            self.sonar_moving = False
            self.xy_cbf = False
            self.xz_cbf = True
            self.R_o = 2
        if sonar_state == 2:
            self.sonar_moving = True
            self.xy_cbf = False
            self.xz_cbf = False
            self.R_o = 4
        if sonar_state == 0:
            self.sonar_moving = False
            self.xy_cbf = True
            self.xz_cbf = False
            self.R_o = 4

    def vel_callback(self, msg):
        self.v_alg = msg
        self.process_data(self.v_alg)

    def point_cloud_callback(self, msg):
        self.last_h = self.current_h
        if self.vehicle_pose is None:
            self.get_logger().info('Vehicle pose not yet received')
            return

        pc_data = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        new_points_float = np.array(
            [[float(pt[0]), float(pt[1]), float(pt[2])] for pt in pc_data],
            dtype=np.float32,
        )
        if new_points_float.size == 0:
            new_points = np.empty((0, 3))
        else:
            new_points = np.round(new_points_float).astype(int)

        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
        if len(self.filtered_points) > 0 and len(new_points) > 0:
            all_points = np.vstack((np.array(self.filtered_points), new_points))
        elif len(new_points) > 0:
            all_points = np.array(new_points)
        elif len(self.filtered_points) > 0:
            all_points = np.array(self.filtered_points)
        else:
            all_points = np.empty((0, 3))

        if len(all_points) > 0:
            all_points = np.unique(all_points, axis=0)
            distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
            smallest_distance = np.min(distances) if distances.size > 0 else float('inf')
            self.get_logger().info(f'Smallest distance: {smallest_distance} ')
            if smallest_distance > self.radius:
                smallest_distance = float('inf')
            within_radius_mask = distances <= self.radius
            self.filtered_points = all_points[within_radius_mask]
        else:
            self.filtered_points = np.empty((0, 3))

    def pose_callback(self, msg):
        if self.xy_cbf or self.xz_cbf or self.sonar_moving:
            self.vehicle_pose = msg.pose.pose.position
            orientation_q = msg.pose.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.quaternion = quaternion
            _, _, self.yaw = euler_from_quaternion(quaternion)

            vxx, vyy, vzz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
            filtered = np.array(self.filtered_points)
            if filtered.size == 0:
                self.closest_obstacle_distance = float('inf')
                self.closest_point = None
                self.current_h = float('inf')
                return

            distance = np.linalg.norm(filtered - np.array([vxx, vyy, vzz]), axis=1)
            if distance.size > 0:
                smallest_distance = np.min(distance)
                min_indices = np.where(distance == smallest_distance)[0]
                self.closest_point = filtered[min_indices[0]]
            else:
                smallest_distance = float('inf')
                self.closest_point = None

            if smallest_distance > self.radius:
                smallest_distance = float('inf')
                self.closest_point = None

            self.closest_obstacle_distance = smallest_distance
            self.current_h = (self.closest_obstacle_distance ** 2) - (self.R_o ** 2)

    def compute_h_dot_xy(self, vel_1, vel_2):
        h_state_dot = np.array([
            2 * (self.vehicle_pose.x - self.closest_point[0]),
            2 * (self.vehicle_pose.y - self.closest_point[1]),
        ])
        self.h_dot_xy_t = np.dot(h_state_dot, np.transpose(np.array([vel_1, vel_2])))
        return self.h_dot_xy_t

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion
        return np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)],
        ])

    def global_to_local_3d(self, global_coord):
        vehicle_position = self.vehicle_pose
        orientation_q = self.quaternion
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation_q)
        global_x, global_y, global_z = global_coord
        vehicle_x, vehicle_y, vehicle_z = vehicle_position.x, vehicle_position.y, vehicle_position.z
        shifted_global = np.array([global_x - vehicle_x, global_y - vehicle_y, global_z - vehicle_z])
        return rotation_matrix.T @ shifted_global

    def compute_h_dot_xz(self, vel_1, vel_2):
        local_cordi = self.global_to_local_3d(self.closest_point)
        h_state_dot = np.array([2 * (-local_cordi[0]), 2 * (-local_cordi[2])])
        self.h_dot_xz_t = np.dot(h_state_dot, np.transpose(np.array([vel_1, vel_2])))
        return self.h_dot_xz_t

    def transform_velocity_to_global_xy(self, local_velocity_x, local_velocity_y):
        global_velocity_x = local_velocity_x * np.cos(self.yaw) - local_velocity_y * np.sin(self.yaw)
        global_velocity_y = local_velocity_x * np.sin(self.yaw) + local_velocity_y * np.cos(self.yaw)
        return global_velocity_x, global_velocity_y

    def transform_velocity_to_global_xz(self, local_velocity_x, local_velocity_z):
        global_velocity_x = local_velocity_x * np.cos(self.yaw)
        return global_velocity_x, local_velocity_z

    def transform_velocity_to_local_xy(self, global_velocity_x, global_velocity_y):
        local_velocity_x = global_velocity_x * np.cos(self.yaw) + global_velocity_y * np.sin(self.yaw)
        local_velocity_y = -global_velocity_x * np.sin(self.yaw) + global_velocity_y * np.cos(self.yaw)
        return local_velocity_x, local_velocity_y

    def transform_velocity_to_local_xz(self, global_velocity_x, global_velocity_z):
        local_velocity_x = global_velocity_x * np.cos(self.yaw)
        return local_velocity_x, global_velocity_z

    def cbf_optimization_xy(self, velll):
        linear_velocity_x = cp.Variable()
        linear_velocity_y = cp.Variable()
        desired_velocity_x, desired_velocity_y = self.transform_velocity_to_global_xy(velll.linear.x, velll.linear.y)
        objective = cp.Minimize(cp.square(linear_velocity_x - desired_velocity_x) + cp.square(linear_velocity_y - desired_velocity_y))
        constraint = self.compute_h_dot_xy(linear_velocity_x, linear_velocity_y) + self.kappa * (self.current_h - 0.5)
        problem = cp.Problem(objective, [constraint >= 0])
        try:
            problem.solve()
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                optimized_velocity_x, optimized_velocity_y = self.transform_velocity_to_local_xy(linear_velocity_x.value, linear_velocity_y.value)
                safe_velocity = np.array([optimized_velocity_x, optimized_velocity_y])
                h_dddot = self.compute_h_dot_xy(optimized_velocity_x, optimized_velocity_y)
                optimal_constraint = h_dddot + self.kappa * (self.current_h - 0.5)
                _ = optimal_constraint
            else:
                print('Optimization failed: No optimal solution found')
                safe_velocity = np.array([0.0, 0.0])
        except Exception as e:
            print(f'An error occurred during optimization: {e}')
            safe_velocity = np.array([0.0, 0.0])
        return safe_velocity

    def cbf_optimization_xz(self, velll):
        self.get_logger().info('entered xz optimization')
        linear_velocity_x = cp.Variable()
        linear_velocity_z = cp.Variable()
        desired_velocity_x, desired_velocity_z = velll.linear.x, velll.linear.z
        objective = cp.Minimize(cp.square(linear_velocity_x - desired_velocity_x) + cp.square(linear_velocity_z - desired_velocity_z))
        constraint = self.compute_h_dot_xz(linear_velocity_x, linear_velocity_z) + self.kappa1 * (self.current_h - 0.5)
        problem = cp.Problem(objective, [constraint >= 0])
        try:
            problem.solve()
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                optimized_velocity_x, optimized_velocity_z = linear_velocity_x.value, linear_velocity_z.value
                safe_velocity = np.array([optimized_velocity_x, optimized_velocity_z])
                h_dddot = self.compute_h_dot_xz(optimized_velocity_x, optimized_velocity_z)
                optimal_constraint = h_dddot + self.kappa * (self.current_h - 0.5)
                _ = optimal_constraint
                self.get_logger().info(f'closest distance: {self.closest_obstacle_distance}')
                self.get_logger().info(f'z alg :{self.v_alg.linear.z}')
                self.get_logger().info(f'z opti :{optimized_velocity_z}')
                self.get_logger().info(f'x alg :{self.v_alg.linear.x}')
                self.get_logger().info(f'x opti :{optimized_velocity_x}')
                print('Optimization successful')
            else:
                print('Optimization failed: No optimal solution found')
                safe_velocity = np.array([0.0, 0.0])
        except Exception as e:
            print(f'An error occurred during optimization: {e}')
            safe_velocity = np.array([0.0, 0.0])
        return safe_velocity

    def process_data(self, valg):
        if self.xy_cbf:
            if self.current_h != float('inf') and self.closest_point is not None and self.vehicle_pose is not None:
                v_safe = self.cbf_optimization_xy(valg)
                self.publish_safe_control_input_xy(v_safe)
            else:
                v_safe = np.array([valg.linear.x, valg.linear.y])
                self.publish_safe_control_input_xy(v_safe)

        if self.xz_cbf:
            if self.current_h != float('inf') and self.closest_point is not None and self.vehicle_pose is not None:
                v_safe = self.cbf_optimization_xz(valg)
                self.publish_safe_control_input_xz(v_safe)
            else:
                v_safe = np.array([valg.linear.x, valg.linear.z])
                self.publish_safe_control_input_xz(v_safe)

    def publish_safe_control_input_xy(self, velocity):
        commands = np.nan_to_num(np.array(velocity), nan=0.0, posinf=0.0, neginf=0.0).tolist()
        twist = Twist()
        twist.angular.z = self.v_alg.angular.z
        twist.linear.x = float(commands[0])
        twist.linear.y = float(commands[1])
        self.cmd_vel_pub.publish(twist)

    def publish_safe_control_input_xz(self, velocity):
        commands = np.nan_to_num(np.array(velocity), nan=0.0, posinf=0.0, neginf=0.0).tolist()
        twist = Twist()
        self.h_pub.publish(Float64(data=float(np.nan_to_num(self.current_h, nan=0.0, posinf=0.0, neginf=0.0))))
        twist.linear.x = float(commands[0])
        twist.linear.z = float(commands[1])
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
