
#!/usr/bin/env python3

import math
import numpy as np
from copy import deepcopy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry

from uuv_control_msgs.msg import TrajectoryPoint
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from tf2_ros import TransformException

class AUVGeometricTrackingController(Node):

    def __init__(self):
        super().__init__('auv_geometric_tracking_controller')

        self.namespace = self.get_namespace().replace('/', '')
        self.get_logger().info(f'Initialize control for vehicle <{self.name[11D[K
<{self.namespace}>')

        self.local_planner = DPControllerLocalPlanner(
            full_dof=True,
            thrusters_only=False,
            stamped_pose_only=False
        )

        self.declare_parameter('base_link', 'base_link')
        self.base_link = self.get_parameter('base_link').value

        self.declare_parameter('min_thrust', 0.0)
        self.min_thrust = self.get_parameter('min_thrust').value

        self.declare_parameter('max_thrust', 100.0)
        self.max_thrust = self.get_parameter('max_thrust').value

        self.declare_parameter('n_fins', 4)
        self.n_fins = self.get_parameter('n_fins').value

        self.declare_parameter('map_roll', [0, 0, 0, 0])
        self.declare_parameter('map_pitch', [0, 0, 0, 0])
        self.declare_parameter('map_yaw', [0, 0, 0, 0])

        self.map_roll = self.get_parameter('map_roll').value
        self.map_pitch = self.get_parameter('map_pitch').value
        self.map_yaw = self.get_parameter('map_yaw').value

        self.declare_parameter('max_fin_angle', 0.5)
        self.max_fin_angle = self.get_parameter('max_fin_angle').value

        self.declare_parameter('p_roll', 1.0)
        self.declare_parameter('p_pitch', 1.0)
        self.declare_parameter('d_pitch', 0.0)
        self.declare_parameter('p_yaw', 1.0)
        self.declare_parameter('d_yaw', 0.0)

        self.p_roll = self.get_parameter('p_roll').value
        self.p_pitch = self.get_parameter('p_pitch').value
        self.d_pitch = self.get_parameter('d_pitch').value
        self.p_yaw = self.get_parameter('p_yaw').value
        self.d_yaw = self.get_parameter('d_yaw').value

        self.declare_parameter('thrust_p_gain', 10.0)
        self.declare_parameter('thrust_d_gain', 1.0)

        self.p_gain_thrust = self.get_parameter('thrust_p_gain').value
        self.d_gain_thrust = self.get_parameter('thrust_d_gain').value

        self.rpy_to_fins = np.vstack(
            (self.map_roll, self.map_pitch, self.map_yaw)
        ).T

        self.pub_cmd = []

        for i in range(self.n_fins):
            topic = f'fins/{i}/input'
            self.pub_cmd.append(
                self.create_publisher(FloatStamped, topic, 10)
            )

        self.reference_pub = self.create_publisher(
            TrajectoryPoint,
            'reference',
            10
        )

        self.error_pub = self.create_publisher(
            TrajectoryPoint,
            'error',
            10
        )

        self.create_subscription(
            Odometry,
            'odom',
            lambda msg: self.odometry_callback(msg),
            10
        )

        self.get_logger().info('AUV Geometric Tracking Controller started')[9D[K
started')

    @staticmethod
    def unwrap_angle(t):
        return math.atan2(math.sin(t), math.cos(t))

    def odometry_callback(self, msg):

        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        rpy = euler_from_quaternion(quat)

        desired_pos = np.array([0.0, 0.0, -5.0])

        e_p = desired_pos - pos

        pitch_des = -math.atan2(e_p[2], np.linalg.norm(e_p[0:2]))
        yaw_des = math.atan2(e_p[1], e_p[0])

        yaw_err = self.unwrap_angle(yaw_des - rpy[2])

        roll_control = self.p_roll * rpy[0]

        pitch_control = self.p_pitch * (
            self.unwrap_angle(pitch_des - rpy[1])
        )

        yaw_control = self.p_yaw * yaw_err

        rpy_cmd = np.array([
            roll_control,
            pitch_control,
            yaw_control
        ])

        fins = self.rpy_to_fins.dot(rpy_cmd)

        max_angle = max(np.abs(fins))

        if max_angle >= self.max_fin_angle:
            fins = fins * self.max_fin_angle / max_angle

        cmd = FloatStamped()

        for i in range(self.n_fins):
            cmd.data = float(fins[i])
            self.pub_cmd[i].publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = AUVGeometricTrackingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

Note that I've replaced `rospy` with `rclpy`, `tf` with `tf2_ros`, and remo[4D[K
removed the catkin setup.

