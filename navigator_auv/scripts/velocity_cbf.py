#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations as tft


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self._subscriptions = [
            self.create_subscription(Twist,       '/rexrov2/cmd_vel_1',   self.vel_callback,  10),
            self.create_subscription(PointCloud2, '/rexrov2/point_cloud', self.pc_callback,   10),
            self.create_subscription(Odometry,    '/rexrov2/pose_gt',     self.pose_callback, 10),
            self.create_subscription(Float64,     '/rexrov2/sonar/moving', self.sonar_cb,     10),
        ]

        self.cmd_pub = self.create_publisher(Twist,   '/rexrov2/cmd_vel',       10)
        self.h_pub   = self.create_publisher(Float64, '/rexrov2/current_h',     10)

        self.declare_parameter('target_depth', -60.0)
        self.declare_parameter('depth_hold_kp', 0.18)
        self.declare_parameter('max_vertical_speed', 0.45)
        self.declare_parameter('control_rate', 10.0)

        self.R_o     = 2.0
        self.radius  = 15.0
        self.kappa   = 0.09
        self.kappa1  = 0.09
        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_hold_kp = float(self.get_parameter('depth_hold_kp').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.filtered_points = np.empty((0,3))
        self.vehicle_pose    = None
        self.quaternion      = None
        self.yaw             = 0.0
        self.current_h       = float('inf')
        self.closest_obstacle_distance = float('inf')
        self.closest_point   = None
        self.v_alg           = Twist()
        self.xy_cbf = True
        self.xz_cbf = self.sonar_moving = False
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop)

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
        pts_list = []
        for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            pts_list.append([float(p[0]), float(p[1]), float(p[2])])
        pts = np.array(pts_list, dtype=float)
        if len(pts):
            pts = np.round(pts).astype(int)
        vp    = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        all_p = np.vstack([self.filtered_points, pts]) if len(self.filtered_points) and len(pts) else (pts if len(pts) else self.filtered_points)
        if len(all_p):
            all_p = np.unique(all_p, axis=0)
            d     = np.linalg.norm(all_p - vp, axis=1)
            self.filtered_points = all_p[d <= self.radius].tolist()
        else:
            self.filtered_points = np.empty((0,3))

    def _analytical_closest_point(self, vp):
        obstacles = [
            {
                'name': 'cube_5',
                'center': np.array([5.011788051259564, 44.89395069717111, -64.00000001601269]),
                'R': np.array([[2.6503813284881376e-06, 0.9999971463842057, 0.0023889781122785193], [-4.221142906178166e-09, -0.0023889781122757225, 0.9999971463877179], [0.9999999999964878, -2.650383849545438e-06, -2.110572122284314e-09]]),
                'size': np.array([8.0, 10.522, 11.9]),
            },
            {
                'name': 'cube_6',
                'center': np.array([6.8028955634973185, 35.243359883679176, -64.000000003612]),
                'R': np.array([[1.6700205782605763e-06, 0.6275538230770735, 0.7785731816204305], [-2.0638941384586086e-06, -0.7785731816176951, 0.6275538230792956], [0.9999999999964757, -2.654920424415997e-06, -5.028577967297133e-09]]),
                'size': np.array([8.0, 15.506, 6.004]),
            },
            {
                'name': 'cube_7',
                'center': np.array([27.011788051259565, 44.89395069717111, -64.00000001601269]),
                'R': np.array([[2.6503813284881376e-06, 0.9999971463842057, 0.0023889781122785193], [-4.221142906178166e-09, -0.0023889781122757225, 0.9999971463877179], [0.9999999999964878, -2.650383849545438e-06, -2.110572122284314e-09]]),
                'size': np.array([8.0, 10.522, 11.9]),
            },
            {
                'name': 'cube_9_1',
                'center': np.array([46.8005050459199, 65.07652601666614, -62.00000001327538]),
                'R': np.array([[-2.141828410283986e-06, -0.8075581004022757, -0.5897880250310958], [1.561163793738276e-06, 0.5897880250290297, -0.8075581004051161], [0.9999999999964877, -2.6504065930615956e-06, -2.4942803643658954e-09]]),
                'size': np.array([8.0, 6.597, 6.599]),
            },
            {
                'name': 'cube_9_2',
                'center': np.array([9.578820927197864, 72.24682762301741, -62.00000000472515]),
                'R': np.array([[2.094850297310164e-07, 0.07733006675476849, 0.9970055470134853], [-2.6431262826506352e-06, -0.9970055470099819, 0.07733006675505212], [0.999999999996485, -2.6514110565931e-06, -4.464604778717622e-09]]),
                'size': np.array([8.0, 6.597, 6.599]),
            },
            {
                'name': 'cube_6_1',
                'center': np.array([25.9799554089172, 28.57190912376761, -64.00000001562488]),
                'R': np.array([[-2.222540431932005e-06, -0.8364626516687093, -0.5480239341109217], [1.4497803601083694e-06, 0.5480239341090041, -0.8364626516716618], [0.9999999999964793, -2.6535863996846633e-06, -5.318226870239687e-09]]),
                'size': np.array([8.0, 15.506, 6.004]),
            },
            {
                'name': 'moving_obs',
                'center': np.array([11.856436151225576, 53.466223556915665, -64.00000000063348]),
                'R': np.array([[1.299333027596471e-06, 0.49095472496087933, -0.8711850882773987], [2.315571249133378e-06, 0.8711850882743215, 0.4909547249625987], [0.9999999999964749, -2.6552048322873146e-06, -4.881087401224511e-09]]),
                'size': np.array([8.0, 15.506, 6.004]),
            },
            {
                'name': 'moving_obs_1',
                'center': np.array([39.264492335231196, 49.805257069464105, -64.00616126160074]),
                'R': np.array([[-0.0007952224280825951, 0.4168706723644088, -0.9089654614691008], [2.41290237094653e-06, 0.9089657496716838, 0.41687080242920815], [0.9999996838076839, 0.00032931176678740743, -0.0007238358187315588]]),
                'size': np.array([8.0, 15.506, 6.004]),
            },
            {
                'name': 'cube_13_1',
                'center': np.array([55.00000002879786, 95.985, -30.0]),
                'R': np.array([[3.2051034546805546e-09, 0.0007963267107247583, -0.9999996829318346], [0.9999999999964793, -2.653591504293374e-06, 1.0919780066796273e-09], [-2.65358979335273e-06, -0.9999996829283139, -0.0007963267107304597]]),
                'size': np.array([17.97, 12.0, 48.0]),
            },
        ]

        closest_pt = None
        min_dist = float('inf')

        for obs in obstacles:
            local_vp = obs['R'].T @ (vp - obs['center'])
            half_size = obs['size'] / 2.0
            local_closest = np.clip(local_vp, -half_size, half_size)
            world_closest = obs['R'] @ local_closest + obs['center']
            
            dist = np.linalg.norm(vp - world_closest)
            if dist < min_dist:
                min_dist = dist
                closest_pt = world_closest
                
        return closest_pt, min_dist

    def pose_callback(self, msg):
        if not (self.xy_cbf or self.xz_cbf or self.sonar_moving): return
        self.vehicle_pose = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        q   = [ori.x, ori.y, ori.z, ori.w]
        self.quaternion = q
        _, _, self.yaw = tft.euler_from_quaternion(q)
        vp = np.array([self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z])
        
        pc_closest = None
        pc_dist = float('inf')
        if len(self.filtered_points):
            fp = np.array(self.filtered_points)
            d  = np.linalg.norm(fp - vp, axis=1)
            idx = np.argmin(d)
            sd  = d[idx]
            if sd <= self.radius:
                pc_closest = fp[idx]
                pc_dist = sd

        an_closest, an_dist = self._analytical_closest_point(vp)

        if pc_dist < an_dist:
            self.closest_point = pc_closest
            self.closest_obstacle_distance = pc_dist
        else:
            self.closest_point = an_closest
            self.closest_obstacle_distance = an_dist

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

    def _project_to_cbf_constraint(self, desired, gradient, margin):
        norm_sq = float(gradient @ gradient)
        if norm_sq < 1e-9:
            return desired

        constraint_value = float(gradient @ desired + margin)
        if constraint_value >= 0.0:
            return desired

        return desired + (-constraint_value / norm_sq) * gradient

    def _depth_hold_velocity(self):
        if self.vehicle_pose is None:
            return 0.0
        error = self.target_depth - self.vehicle_pose.z
        cmd = self.depth_hold_kp * error
        return float(np.clip(cmd, -self.max_vertical_speed, self.max_vertical_speed))

    def _opt_xy(self, v):
        dgx, dgy = self._tf_l2g_xy(v.linear.x, v.linear.y)
        desired = np.array([dgx, dgy], dtype=float)
        gradient = np.array([
            2*(self.vehicle_pose.x - self.closest_point[0]),
            2*(self.vehicle_pose.y - self.closest_point[1]),
        ], dtype=float)
        margin = self.kappa * (self.current_h - 0.5)
        safe = self._project_to_cbf_constraint(desired, gradient, margin)
        return np.array(self._tf_g2l_xy(safe[0], safe[1]))

    def _opt_xz(self, v):
        desired = np.array([v.linear.x, v.linear.z], dtype=float)
        local_closest = self._global_to_local(self.closest_point)
        gradient = np.array([-2*local_closest[0], -2*local_closest[2]], dtype=float)
        margin = self.kappa1 * (self.current_h - 0.5)
        return self._project_to_cbf_constraint(desired, gradient, margin)

    def process_data(self, v):
        if self.xy_cbf:
            safe = self._opt_xy(v) if self.current_h != float('inf') else np.array([v.linear.x, v.linear.y])
            tw = Twist()
            tw.angular.z = v.angular.z
            tw.linear.x = safe[0]
            tw.linear.y = safe[1]
            tw.linear.z = self._depth_hold_velocity()
            self.cmd_pub.publish(tw)
        if self.xz_cbf:
            safe = self._opt_xz(v) if self.current_h != float('inf') else np.array([v.linear.x, v.linear.z])
            tw = Twist(); tw.linear.x = safe[0]; tw.linear.z = safe[1]
            self.h_pub.publish(Float64(data=float(self.current_h)))
            self.cmd_pub.publish(tw)

    def control_loop(self):
        if self.vehicle_pose is None:
            return
        self.process_data(self.v_alg)


def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
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
