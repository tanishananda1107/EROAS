#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import tf_transformations as tft
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, PointCloud2, JointState
from std_msgs.msg import Float64
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')

        self.declare_parameter('cmd_vel_topic', '/rexrov2/cmd_vel_1')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('sonar_topic', '/rexrov2/blueview_p900/sonar_image_raw')
        self.declare_parameter('waypoints', '29,97,-50;31,110,-55;30,90,-90;30,120,-40')
        self.declare_parameter('sonar_timeout', 1.0)
        self.declare_parameter('fallback_speed', 0.35)
        self.declare_parameter('fallback_yaw_kp', 0.8)
        self.declare_parameter('fallback_max_yaw_rate', 0.5)
        self.declare_parameter('loop_waypoints', False)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        sonar_topic = self.get_parameter('sonar_topic').value

        self.cmd_vel_pub   = self.create_publisher(Twist,   cmd_vel_topic,  10)
        self.img_pub       = self.create_publisher(Image,   '/rexrov2/detected_objects', 10)
        self.joint_pub     = self.create_publisher(Float64, '/rexrov2/sonar_joint_position_controller/command', 10)
        self.sonar_move_pub= self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)

        self.create_subscription(ProjectedSonarImage, sonar_topic,
                                 self.sonar_image_raw_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, pose_topic, self.pose_callback, 10)

        self.bridge = CvBridge()
        self.timer  = self.create_timer(1.0/8.0, self.run_once)  # 8 Hz

        # --- state ---
        self.beam_directions = []
        self.ranges = []
        self.data_raw = None
        self.ping_info = None
        self.data_available = False
        self.last_sonar_time = None
        self.global_angle = 0.0
        self.turn_around = False
        self.right_goal = self.left_goal = False
        self.avg_right_slope = self.avg_left_slope = None
        self.a_poly = None
        self.pose = None
        self.in_z_bound = True
        self.target_x = self.target_y = self.target_z = None
        self.collect_3d_data_flag = False
        self.latest_scan_available = False
        self.start_3d_time = None
        self.go_vertical_angle = []
        self.min_ranges = []
        self.min_range = 0
        self.z_motion_ongoing = False
        self.pose_before_z_motion = None
        self.closest_to_target_vertical = None
        self.rospy_first_time = True
        self.ttd = True
        self.fms = True
        self.new_midb_available = False
        self.check_verti_go = True
        self.target_beam = None
        self.orient_3d = False
        self.sonar_centered = True
        self.xy_called = False
        self.frequency = 0.01
        self.amplitude = 0.7
        self.offset = 0
        self.angle_z_goal = 0.0
        self.sonar_angle = 0

        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.current_goal_index = 0
        self.current_goal = self.waypoints[self.current_goal_index]
        self.sonar_timeout_ns = int(
            float(self.get_parameter('sonar_timeout').value) * 1e9)
        self.fallback_speed = float(self.get_parameter('fallback_speed').value)
        self.fallback_yaw_kp = float(self.get_parameter('fallback_yaw_kp').value)
        self.fallback_max_yaw_rate = float(
            self.get_parameter('fallback_max_yaw_rate').value)
        self.loop_waypoints = bool(self.get_parameter('loop_waypoints').value)
        self.fallback_announced = False

    def _parse_waypoints(self, value):
        waypoints = []
        for item in str(value).split(';'):
            item = item.strip()
            if not item:
                continue
            fields = [float(v.strip()) for v in item.split(',')]
            if len(fields) != 3:
                raise ValueError(f'Invalid waypoint "{item}", expected x,y,z')
            waypoints.append(tuple(fields))
        return waypoints or [(29, 97, -50), (31, 110, -55), (30, 90, -90), (30, 120, -40)]

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def pose_callback(self, pose_msg):
        self.target_x, self.target_y, self.target_z = self.current_goal
        pose = pose_msg.pose.pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        self.pose = pose
        ori = pose.orientation
        _, _, yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        angle_to_target = math.atan2(self.target_y - y, self.target_x - x)
        angular_error   = angle_to_target - yaw
        self.global_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        if   -math.pi/2 < self.global_angle < math.pi/2:
            self.global_angle = math.pi/2 + self.global_angle;  self.turn_around = False
        elif  math.pi/2 < self.global_angle < math.pi:
            self.global_angle = 3.14;  self.turn_around = False
        elif -math.pi  < self.global_angle < -math.pi/2:
            self.global_angle = 0.0;   self.turn_around = False

        self.in_z_bound = abs(z - self.target_z) <= 5

        d_xy = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
        d    = math.sqrt(d_xy**2 + (self.target_z - z)**2)

        if self.collect_3d_data_flag and d_xy != 0:
            vert_dist = self.target_z - z
            self.angle_z_goal = math.degrees(math.asin(vert_dist / d))

        if d < 5.0:
            self.update_goal()

    def sonar_image_raw_callback(self, data):
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.ping_info = data.ping_info
        if self.beam_directions and self.ranges and data.image.data and self.ping_info:
            self.data_raw = np.frombuffer(data.image.data, dtype=np.uint8)
            self.data_available = True
            self.last_sonar_time = self.get_clock().now()
            self.fallback_announced = False

    # ------------------------------------------------------------------ #
    #  Goal management                                                     #
    # ------------------------------------------------------------------ #
    def update_goal(self):
        if self.current_goal_index < len(self.waypoints) - 1:
            self.current_goal_index += 1
            self.current_goal = self.waypoints[self.current_goal_index]
            self.get_logger().info('Next waypoint')
        elif self.loop_waypoints:
            self.current_goal_index = 0
            self.current_goal = self.waypoints[self.current_goal_index]
            self.get_logger().info('Restarting waypoint loop')
        else:
            self.get_logger().info('All waypoints reached.')

    # ------------------------------------------------------------------ #
    #  Geometry helpers                                                    #
    # ------------------------------------------------------------------ #
    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        angle_rad = i * 0.0030739647336304188 + math.pi / 4
        distance  = j * 15 / max_bins
        return distance * math.cos(angle_rad), distance * math.sin(angle_rad)

    def find_and_plot_curve(self, x_coords, y_coords):
        sorted_pts = sorted(zip(x_coords, y_coords))
        x_coords, y_coords = zip(*sorted_pts)
        unique = []
        for p in zip(x_coords, y_coords):
            if not unique or unique[-1][0] != p[0]:
                unique.append(p)
        x_coords, y_coords = zip(*unique)
        coeffs = np.polyfit(x_coords, y_coords, 2)
        deriv  = [2*coeffs[0], coeffs[1]]
        self.a_poly = deriv[0] / 2
        pos_slopes = [np.polyval(deriv, x) for x in x_coords if x > 0]
        neg_slopes = [np.polyval(deriv, x) for x in x_coords if x < 0]
        self.avg_right_slope = np.mean(pos_slopes) if pos_slopes else 0
        self.avg_left_slope  = np.mean(neg_slopes) if neg_slopes else 0

    # ------------------------------------------------------------------ #
    #  Core navigation                                                     #
    # ------------------------------------------------------------------ #
    def _global_angle_to_beam(self, ga):
        start, end = 0.7853, 2.35619
        rpb = 1.57 / 512
        if start <= ga <= end:
            return int((ga - start) / rpb)
        return 512 if ga > end else 0

    def check_for_10_degree_coverage(self, obstacle_free_beams):
        required_beams = int(15.0 / (90.0/512 * 5))
        obstacle_free_beams.sort()
        mid_beam = []
        for i in range(len(obstacle_free_beams) - required_beams):
            if obstacle_free_beams[i+required_beams] - obstacle_free_beams[i] == 5*required_beams:
                mid_beam.append(obstacle_free_beams[i + required_beams//2])
        if not mid_beam:
            return False, None
        target = self._global_angle_to_beam(self.global_angle)
        self.target_beam = target
        return True, min(mid_beam, key=lambda b: abs(b - target))

    def check_for_boundedness(self, obs_free):
        self.right_goal = 1.5 <= self.global_angle <= 3.14
        self.left_goal  = 0   <= self.global_angle <  1.5
        s1 = {0,5,10,15,20,25,30,35}
        s2 = {510,505,500,495,490,485,480}
        has0   = s1.issubset(set(obs_free))
        has512 = s2.issubset(set(obs_free))
        if has0   and self.right_goal: return 1
        if has512 and self.left_goal:  return 1
        if has0   and self.left_goal:  return 2
        if has512 and self.right_goal: return 2
        return 4

    def process_data(self):
        self.orient_3d = False
        if self.turn_around:
            self.publish_heading(256, False); return
        beam_count  = 512
        range_count = len(self.ranges)
        threshold   = 2
        data = np.array(self.data_raw).reshape((range_count, beam_count))
        obs_free, contour = [], []
        for i in range(0, beam_count, 5):
            hit = False
            for j in range(10, range_count - 40, 5):
                if np.mean(data[j:j+3, i]) > threshold:
                    contour.append((i, j)); hit = True; break
            if not hit:
                obs_free.append(i)

        if obs_free:
            ok, beam = self.check_for_10_degree_coverage(obs_free)
            if ok:
                self.publish_heading(beam, 1); return
            bn = self.check_for_boundedness(obs_free)
            if   bn == 1 and self.right_goal: self.publish_heading(10,  1)
            elif bn == 1 and self.left_goal:  self.publish_heading(500, 1)
            elif bn == 2 and self.left_goal:  self.publish_heading(10,  1)
            elif bn == 2 and self.right_goal: self.publish_heading(500, 1)
            else:
                pts = [self.polar_to_cartesian(i,j,beam_count,range_count) for i,j in contour]
                if pts:
                    xs, ys = zip(*pts)
                    self.a_poly = None
                    self.find_and_plot_curve(xs, ys)
                    if self.a_poly is not None and self.a_poly < 0.02:
                        if not self.xy_called:
                            self.check_verti_go = True
                            self.navigate_3d(True)
                    else:
                        if self.left_goal:  self.publish_heading(self.avg_left_slope,  2)
                        elif self.right_goal: self.publish_heading(self.avg_right_slope, 2)
        else:
            self.get_logger().info('no gap')

    def publish_heading(self, beam_number, move):
        if self.turn_around:
            tw = Twist(); tw.angular.z = 1.0; self.cmd_vel_pub.publish(tw); return
        tw = Twist()
        if move in (1, 3):
            deg = (beam_number - 256) * (90.0/512)
            rad = math.radians(deg)
            tw.angular.z = 0.12 * rad
            if move == 1:
                tw.linear.x = 0.35 * (1.6 - abs(rad))
        elif move == 2:
            tw.angular.z = 0.1 * beam_number
        self.cmd_vel_pub.publish(tw)

    # ------------------------------------------------------------------ #
    #  3-D navigation                                                      #
    # ------------------------------------------------------------------ #
    def navigate_3d(self, xy_called):
        if self.latest_scan_available:
            self.latest_scan_available = False
            obs_free = sorted(set(self.go_vertical_angle))
            self.go_vertical_angle = []
            mid = []
            req = 5
            for i in range(len(obs_free) - req):
                if obs_free[i+req] - obs_free[i] == 3*req:
                    mid.append(obs_free[i+req//2])
            if mid:
                self.closest_to_target_vertical = min(mid, key=lambda x: abs(x - self.angle_z_goal))
                self.new_midb_available = True
                self.z_motion_ongoing = True
                self.start_data_collection()
            else:
                self.z_motion_ongoing = False
                self.stop_data_collection()
            self.xy_called = xy_called
        else:
            self.collect_3d_data_flag = True
            if self.orient_3d:
                self.start_data_collection()
            if self.sonar_centered:
                self.joint_pub.publish(Float64(data=0.8))

    def move_sonar(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_3d_time
        half = 0.5 / self.frequency
        if t < half:
            phase = math.pi/2 if self.ttd else math.pi/2
            z = self.amplitude * math.sin(2*math.pi*self.frequency*t + phase) + self.offset
            deg = int(z * 180/math.pi)
            self.sonar_angle = round(deg/3)*3
            self.joint_pub.publish(Float64(data=z))
            self.process_3d_data(self.sonar_angle)
        else:
            self.latest_scan_available = True
            self.ttd = not self.ttd
            self.min_range = min(self.min_ranges) - 2 if self.min_ranges else 7
            self.min_ranges = []
            self.pose_before_z_motion = self.pose
            self.navigate_3d(False)

    def process_3d_data(self, sonar_angle):
        beam_count  = 512
        range_count = len(self.ranges)
        data = np.array(self.data_raw).reshape((range_count, beam_count))
        hit = False
        for j in range(5, 350, 5):
            if hit: break
            for i in range(100, 400, 7):
                if data[j, i] > 15:
                    self.min_ranges.append(j * 12/len(self.ranges))
                    hit = True; break
        if not hit:
            self.go_vertical_angle.append(int(sonar_angle))

    def start_data_collection(self):
        self.collect_3d_data_flag = True
        self.start_3d_time = self.get_clock().now().nanoseconds / 1e9

    def stop_data_collection(self):
        self.collect_3d_data_flag = False
        self.joint_pub.publish(Float64(data=0.0))
        self.sonar_centered = True

    def orient_for_3d(self):
        if 1.4 < self.global_angle < 1.7:
            self.orient_3d = True
            self.navigate_3d(False)
        else:
            self.orient_3d = False
            self.publish_heading(self._global_angle_to_beam(self.global_angle), 3)

    def avoidance(self):
        if not self.in_z_bound:
            self.check_verti_go = True
            self.navigate_3d(False)
        else:
            self.process_data()

    def move_in_z(self):
        if not self.closest_to_target_vertical:
            return
        ang = max(self.closest_to_target_vertical, 1)
        lx  = 0.5
        lz  = lx * math.tan(ang * math.pi / 180)
        tw  = Twist()
        tw.linear.x = lx
        tw.linear.z = lz
        self.cmd_vel_pub.publish(tw)

    def publish_waypoint_fallback(self):
        if self.pose is None:
            return

        position = self.pose.position
        orientation = self.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
        target_x, target_y, _ = self.current_goal
        target_yaw = math.atan2(target_y - position.y, target_x - position.x)
        yaw_error = math.atan2(
            math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

        tw = Twist()
        tw.angular.z = float(np.clip(
            self.fallback_yaw_kp * yaw_error,
            -self.fallback_max_yaw_rate,
            self.fallback_max_yaw_rate))
        tw.linear.x = self.fallback_speed * max(0.0, math.cos(yaw_error))
        self.cmd_vel_pub.publish(tw)

        if not self.fallback_announced:
            self.get_logger().warning(
                'No fresh sonar frames; following waypoints using pose fallback')
            self.fallback_announced = True

    # ------------------------------------------------------------------ #
    #  Main timer                                                          #
    # ------------------------------------------------------------------ #
    def run_once(self):
        if self.rospy_first_time:
            self.stop_data_collection()
            self.rospy_first_time = False
        now = self.get_clock().now()
        sonar_is_fresh = (
            self.data_available and
            self.last_sonar_time is not None and
            (now - self.last_sonar_time).nanoseconds <= self.sonar_timeout_ns
        )
        if not sonar_is_fresh:
            self.publish_waypoint_fallback()
            return
        if self.z_motion_ongoing:
            self.sonar_move_pub.publish(Float64(data=1.0))
            self.move_in_z()
            if self.collect_3d_data_flag:
                self.move_sonar()
        else:
            if self.collect_3d_data_flag:
                if self.orient_3d:
                    self.sonar_move_pub.publish(Float64(data=2.0))
                    self.move_sonar()
                else:
                    self.sonar_move_pub.publish(Float64(data=0.0))
                    self.orient_for_3d()
            else:
                self.sonar_move_pub.publish(Float64(data=0.0))
                self.avoidance()


def main():
    rclpy.init()
    node = SonarHeadingNode()
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
