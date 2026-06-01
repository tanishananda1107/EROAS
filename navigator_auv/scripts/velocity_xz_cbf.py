#!/usr/bin/env python3
# ROS 2 port (XZ-only variant)
import rclpy
from rclpy.node import Node
import numpy as np
import cvxpy as cp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations as tft


class ObstacleAvoidanceXZ(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_xz')

        self.create_subscription(Twist,       '/rexrov2/cmd_vel_1',    self.vel_cb,   10)
        self.create_subscription(PointCloud2, '/rexrov2/point_cloud',  self.pc_cb,    10)
        self.create_subscription(Odometry,    '/rexrov2/pose_gt',      self.pose_cb,  10)
        self.create_subscription(Float64,     '/rexrov2/sonar/moving', self.sonar_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        self.R_o = 1.5; self.radius = 15.0; self.kappa = 0.01
        self.filtered_points = np.empty((0,3))
        self.vehicle_pose    = None
        self.yaw             = 0.0
        self.current_h       = float('inf')
        self.closest_point   = None
        self.closest_obstacle_distance = float('inf')
        self.v_alg           = Twist()
        self.xz_cbf = False

    def sonar_cb(self, msg):
        s = msg.data
        self.xz_cbf = (s == 1)

    def vel_cb(self, msg):
        self.v_alg = msg
        self.process(msg)

    def pc_cb(self, msg):
        if self.vehicle_pose is None: return
        pts = np.round(np.array(list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)))).astype(int)
        vp  = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        all_p = np.vstack([self.filtered_points, pts]) if len(self.filtered_points) and len(pts) else (pts if len(pts) else self.filtered_points)
        if len(all_p):
            all_p = np.unique(all_p, axis=0)
            d = np.linalg.norm(all_p - vp, axis=1)
            self.filtered_points = all_p[d <= self.radius].tolist()
        else:
            self.filtered_points = np.empty((0,3))

    def pose_cb(self, msg):
        self.vehicle_pose = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, self.yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        if len(self.filtered_points):
            fp = np.array(self.filtered_points)
            d  = np.linalg.norm(fp - vp, axis=1)
            idx = np.argmin(d)
            sd  = d[idx]
            self.closest_point = fp[idx] if sd <= self.radius else None
            self.closest_obstacle_distance = sd if sd <= self.radius else float('inf')
        else:
            self.closest_point = None
            self.closest_obstacle_distance = float('inf')
        self.current_h = self.closest_obstacle_distance**2 - self.R_o**2

    def _h_dot(self, vx, vz):
        g = np.array([2*(self.vehicle_pose.x - self.closest_point[0]),
                      2*(self.vehicle_pose.z - self.closest_point[2])])
        return -g @ np.array([vx, vz])   # note: sign is (vehicle - obstacle)

    def _opt(self, v):
        vx, vz = cp.Variable(), cp.Variable()
        obj  = cp.Minimize(cp.square(vx-v.linear.x) + cp.square(vz-v.linear.z))
        con  = [self._h_dot(vx,vz) + self.kappa*(self.current_h-0.5) >= 0]
        prob = cp.Problem(obj, con)
        try:
            prob.solve()
            if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                return np.array([vx.value, vz.value])
        except Exception: pass
        return np.array([0.0, 0.0])

    def process(self, v):
        safe = self._opt(v) if self.current_h != float('inf') else np.array([v.linear.x, v.linear.z])
        tw = Twist(); tw.linear.x = safe[0]; tw.linear.z = safe[1]
        self.cmd_pub.publish(tw)


def main():
    rclpy.init()
    node = ObstacleAvoidanceXZ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
