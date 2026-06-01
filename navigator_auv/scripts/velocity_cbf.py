#!/usr/bin/env python3
# ROS 2 port
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


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.create_subscription(Twist,       '/rexrov2/cmd_vel_1',   self.vel_callback,  10)
        self.create_subscription(PointCloud2, '/rexrov2/point_cloud', self.pc_callback,   10)
        self.create_subscription(Odometry,    '/rexrov2/pose_gt',     self.pose_callback, 10)
        self.create_subscription(Float64,     '/rexrov2/sonar/moving',self.sonar_cb,      10)

        self.cmd_pub = self.create_publisher(Twist,   '/rexrov2/cmd_vel',       10)
        self.h_pub   = self.create_publisher(Float64, '/rexrov2/current_h',     10)

        self.R_o     = 2.0
        self.radius  = 15.0
        self.kappa   = 0.09
        self.kappa1  = 0.09

        self.filtered_points = np.empty((0,3))
        self.vehicle_pose    = None
        self.quaternion      = None
        self.yaw             = 0.0
        self.current_h       = float('inf')
        self.closest_obstacle_distance = float('inf')
        self.closest_point   = None
        self.v_alg           = Twist()
        self.xy_cbf = self.xz_cbf = self.sonar_moving = False

    # ---- state callbacks ----
    def sonar_cb(self, msg):
        s = msg.data
        self.xy_cbf = self.xz_cbf = self.sonar_moving = False
        self.R_o = 4.0
        if   s == 1: self.xz_cbf       = True; self.R_o = 2.0
        elif s == 2: self.sonar_moving  = True
        elif s == 0: self.xy_cbf        = True

    def vel_callback(self, msg):
        self.v_alg = msg
        self.process_data(msg)

    def pc_callback(self, msg):
        if self.vehicle_pose is None: return
        pts   = np.round(np.array(list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)))).astype(int)
        vp    = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        all_p = np.vstack([self.filtered_points, pts]) if len(self.filtered_points) and len(pts) else (pts if len(pts) else self.filtered_points)
        if len(all_p):
            all_p = np.unique(all_p, axis=0)
            d     = np.linalg.norm(all_p - vp, axis=1)
            self.filtered_points = all_p[d <= self.radius].tolist()
        else:
            self.filtered_points = np.empty((0,3))

    def pose_callback(self, msg):
        if not (self.xy_cbf or self.xz_cbf or self.sonar_moving): return
        self.vehicle_pose = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        q   = [ori.x, ori.y, ori.z, ori.w]
        self.quaternion = q
        _, _, self.yaw = tft.euler_from_quaternion(q)
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

    def _opt_xy(self, v):
        vx, vy = cp.Variable(), cp.Variable()
        dgx, dgy = self._tf_l2g_xy(v.linear.x, v.linear.y)
        obj  = cp.Minimize(cp.square(vx-dgx) + cp.square(vy-dgy))
        con  = [self._h_dot_xy(vx,vy) + self.kappa*(self.current_h-0.5) >= 0]
        prob = cp.Problem(obj, con)
        try:
            prob.solve()
            if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                return np.array(list(self._tf_g2l_xy(vx.value, vy.value)))
        except Exception: pass
        return np.array([0.0, 0.0])

    def _opt_xz(self, v):
        vx, vz = cp.Variable(), cp.Variable()
        obj  = cp.Minimize(cp.square(vx-v.linear.x) + cp.square(vz-v.linear.z))
        con  = [self._h_dot_xz(vx,vz) + self.kappa1*(self.current_h-0.5) >= 0]
        prob = cp.Problem(obj, con)
        try:
            prob.solve()
            if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                return np.array([vx.value, vz.value])
        except Exception: pass
        return np.array([0.0, 0.0])

    def process_data(self, v):
        if self.xy_cbf:
            safe = self._opt_xy(v) if self.current_h != float('inf') else np.array([v.linear.x, v.linear.y])
            tw = Twist(); tw.angular.z = v.angular.z; tw.linear.x = safe[0]; tw.linear.y = safe[1]
            self.cmd_pub.publish(tw)
        if self.xz_cbf:
            safe = self._opt_xz(v) if self.current_h != float('inf') else np.array([v.linear.x, v.linear.z])
            tw = Twist(); tw.linear.x = safe[0]; tw.linear.z = safe[1]
            self.h_pub.publish(Float64(data=float(self.current_h)))
            self.cmd_pub.publish(tw)


def main():
    rclpy.init()
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
