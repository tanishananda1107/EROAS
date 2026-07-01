#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64


POINT_CLOUD_TOPIC = '/rexrov2/blueview_p900_point_cloud'
BEAM_COUNT = 512
RANGE_COUNT = 512
FOV_RAD = math.pi / 2.0
HALF_FOV_RAD = FOV_RAD / 2.0
MAX_RANGE = 15.0


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


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel_1', 10)
        self.joint_pub = self.create_publisher(Float64, '/rexrov2/sonar_joint_position_controller/command', 10)
        self.sonar_move_pub = self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)
        self.create_subscription(PointCloud2, POINT_CLOUD_TOPIC, self.point_cloud_callback, sensor_qos)
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)
        self.spawn_client = self.create_client(SpawnEntity, '/world/eroas_world_a/create')

        self.beam_hit = np.zeros(BEAM_COUNT, dtype=bool)
        self.beam_range = np.full(BEAM_COUNT, MAX_RANGE, dtype=float)
        self.ranges = np.linspace(0.0, MAX_RANGE, RANGE_COUNT)
        self.data_available = False
        self.latest_point_count = 0
        self.last_cloud_time = self.get_clock().now()

        self.criti_cal = False
        self.Kv = 0.35
        self.dominant_slope = None
        self.right_avg_slope = None
        self.left_avg_slope = None
        self.global_angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.turn_around = False
        self.right_goal = False
        self.left_goal = False
        self.avg_right_slope = None
        self.avg_left_slope = None
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.sonar_centered = True
        self.a_poly = None
        self.contour_steps = 0
        self.contour_escape_distance = 8.0
        self.contour_start_pose = None

        self.waypoints = [
            (29, 97, -50),
            (31, 110, -55),
            (30, 90, -90),
            (30, 120, -40),
        ]
        self.current_goal_index = 0
        self.current_goal = self.waypoints[self.current_goal_index]

        self.collect_3d_data_flag = False
        self.latest_scan_available = False
        self.start_3d_time = None
        self.go_vertical_angle = []
        self.in_z_bound = True
        self.frequency = 0.01
        self.amplitude = 0.7
        self.offset = 0.0
        self.min_ranges = []
        self.xy_called = False
        self.min_range = 0.0
        self.z_motion_ongoing = False
        self.pose_before_z_motion = None
        self.closest_to_target_vertical = None
        self.first_cycle = True
        self.ttd = True
        self.fms = True
        self.new_midb_available = False
        self.check_verti_go = True
        self.target_beam = None
        self.orient_3d = False
        self.angle_z_goal = 0.0
        self.pose = None
        self.sonar_angle = 0

        self.timer = self.create_timer(1.0 / 8.0, self.run_once)

    def spawn_marker(self):
        if not self.spawn_client.service_is_ready():
            self.get_logger().info('World create service not ready; skipping marker spawn')
            return
        request = SpawnEntity.Request()
        request.entity_factory.name = 'red_balll'
        request.entity_factory.sdf = """
        <sdf version='1.6'>
            <model name='red_balll'>
                <static>true</static>
                <link name='link'>
                    <visual name='visual'>
                        <geometry><sphere><radius>0.8</radius></sphere></geometry>
                        <material><ambient>1 1 0 1</ambient></material>
                    </visual>
                </link>
            </model>
        </sdf>
        """
        request.entity_factory.pose = Pose()
        request.entity_factory.pose.position.x = 29.0
        request.entity_factory.pose.position.y = 117.0
        request.entity_factory.pose.position.z = -48.0
        self.spawn_client.call_async(request)
        self.get_logger().info('Goal marker spawn requested')

    def pose_callback(self, pose_msg):
        self.target_x, self.target_y, self.target_z = self.current_goal
        pose = pose_msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        self.x = x
        self.y = y
        self.z = z
        orientation = pose.orientation
        self.pose = pose

        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        angle_to_target = math.atan2(self.target_y - y, self.target_x - x)
        angular_error = angle_to_target - yaw
        self.global_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        if (-math.pi / 2.0) < self.global_angle < (math.pi / 2.0):
            self.global_angle = math.pi / 2.0 + self.global_angle
            self.turn_around = False
        elif (math.pi / 2.0) < self.global_angle < math.pi:
            self.global_angle = 3.14
            self.turn_around = False
        elif -math.pi < self.global_angle < -(math.pi / 2.0):
            self.global_angle = 0.0
            self.turn_around = False

        self.in_z_bound = abs(z - self.target_z) <= 5.0

        distance_to_goal_xy = math.sqrt((self.target_x - x) ** 2 + (self.target_y - y) ** 2)
        distance_to_goal = math.sqrt(distance_to_goal_xy ** 2 + (self.target_z - z) ** 2)

        if self.collect_3d_data_flag:
            vertical_distance = self.target_z - z
            if distance_to_goal_xy != 0.0:
                ratio = max(-1.0, min(1.0, vertical_distance / max(distance_to_goal, 1e-6)))
                angle_to_z_goal = math.asin(ratio)
            else:
                angle_to_z_goal = math.pi / 2.0 if vertical_distance > 0.0 else -math.pi / 2.0
            self.angle_z_goal = math.degrees(angle_to_z_goal)

        if distance_to_goal < 5.0:
            self.update_goal()

        print(f'current_position x={x:.3f} y={y:.3f} z={z:.3f}')

    def update_goal(self):
        if self.current_goal_index < len(self.waypoints) - 1:
            self.current_goal_index += 1
            self.current_goal = self.waypoints[self.current_goal_index]
            print(self.current_goal)
            print('Current waypoint reached, going to next')
        else:
            print('All waypoints have been reached.')
            print('mission over')

    def point_cloud_callback(self, msg):
        beam_range = np.full(BEAM_COUNT, MAX_RANGE, dtype=float)
        point_count = 0
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            horizontal_range = math.hypot(x, y)
            total_range = math.sqrt(x * x + y * y + z * z)
            if horizontal_range <= 1e-6 or total_range <= 0.05 or total_range > MAX_RANGE:
                continue
            angle = math.atan2(y, x)
            if angle < -HALF_FOV_RAD or angle > HALF_FOV_RAD:
                continue
            beam = int((angle + HALF_FOV_RAD) / FOV_RAD * (BEAM_COUNT - 1))
            beam = max(0, min(BEAM_COUNT - 1, beam))
            if total_range < beam_range[beam]:
                beam_range[beam] = total_range
            point_count += 1

        self.beam_range = beam_range
        self.beam_hit = beam_range < MAX_RANGE
        self.latest_point_count = point_count
        self.last_cloud_time = self.get_clock().now()
        self.data_available = True
        free_beams = int(np.count_nonzero(~self.beam_hit))
        nearest = float(np.min(self.beam_range)) if np.any(self.beam_hit) else float('inf')
        print(f'[FLS_SENSOR] source=point_cloud points={point_count} free_beams={free_beams} nearest={nearest:.2f}')

    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        angle_per_beam = FOV_RAD / max_beams
        angle_rad = (i * angle_per_beam) - HALF_FOV_RAD
        distance = j * MAX_RANGE / max_bins
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    def print_polynomial_coefficients(self, coeffs):
        print('Polynomial coefficients:', coeffs)

    def compute_polynomial_derivative(self, coeffs):
        return np.polyder(coeffs)

    def find_and_plot_curve(self, x_coords, y_coords):
        if len(x_coords) < 3:
            self.a_poly = 0.0
            self.avg_right_slope = 0.0
            self.avg_left_slope = 0.0
            return
        coeffs = np.polyfit(x_coords, y_coords, 2)
        derivative_coeffs = self.compute_polynomial_derivative(coeffs)
        slopes = np.polyval(derivative_coeffs, x_coords)
        right_slopes = [s for y, s in zip(y_coords, slopes) if y >= 0]
        left_slopes = [s for y, s in zip(y_coords, slopes) if y < 0]
        self.avg_right_slope = float(np.mean(right_slopes)) if right_slopes else 0.0
        self.avg_left_slope = float(np.mean(left_slopes)) if left_slopes else 0.0
        self.dominant_slope = self.avg_right_slope if abs(self.avg_right_slope) > abs(self.avg_left_slope) else self.avg_left_slope
        self.a_poly = abs(float(coeffs[0]))

    def process_data(self):
        self.orient_3d = False
        if self.turn_around:
            self.publish_heading(256, False)
            return

        beam_count = BEAM_COUNT
        range_count = RANGE_COUNT
        obstacle_free_beam_numbers = []
        contour_points = []
        self.criti_cal = False

        for i in range(0, beam_count, 5):
            if self.beam_hit[i]:
                range_bin = int(max(0, min(range_count - 1, self.beam_range[i] / MAX_RANGE * range_count)))
                contour_points.append((i, range_bin))
            else:
                obstacle_free_beam_numbers.append(i)

        is_covered = False
        go_beam_no = None
        if obstacle_free_beam_numbers:
            is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)

        if is_covered:
            print(f'[GAP_FINDING] gaps=1 selected_gap={go_beam_no}')
            print('[SPD2C] decision=gap_follow')
            self.contour_start_pose = None
            self.contour_steps = 0
            self.publish_heading(go_beam_no, 1)
            return

        print('no gap')
        boundedness_number = self.check_for_boundedness(obstacle_free_beam_numbers)
        if boundedness_number == 1 and self.right_goal:
            print('right bounded')
            self.publish_heading(10, 1)
        elif boundedness_number == 1 and self.left_goal:
            print('left bounded')
            self.publish_heading(500, 1)
        elif boundedness_number == 2 and self.left_goal:
            print('turn right')
            self.publish_heading(10, 1)
        elif boundedness_number == 2 and self.right_goal:
            print('turn left')
            self.publish_heading(500, 1)
        else:
            print('contour_1')
            points_cartesian = [self.polar_to_cartesian(i, j, beam_count, range_count) for (i, j) in contour_points]
            if points_cartesian:
                x_coords, y_coords = zip(*points_cartesian)
                self.a_poly = None
                self.find_and_plot_curve(x_coords, y_coords)
                print(self.a_poly)
            if self.a_poly is not None and self.a_poly < 0.02:
                if not self.xy_called:
                    print('3d')
                    print(self.a_poly)
                    self.check_verti_go = True
                    self.navigate_3d(True)
                else:
                    self.publish_heading(self.avg_left_slope if self.avg_left_slope is not None else 0.0, 2)
            else:
                print('3d not required')
                if self.left_goal:
                    self.publish_heading(self.avg_left_slope if self.avg_left_slope is not None else 0.0, 2)
                elif self.right_goal:
                    self.publish_heading(self.avg_right_slope if self.avg_right_slope is not None else 0.0, 2)

    def orient_for_3d(self):
        def global_angle_to_beam_number(global_angle):
            angle_range_start = 0.7853
            angle_range_end = 2.35619
            rad_per_beam = 1.57 / BEAM_COUNT
            if angle_range_start <= global_angle <= angle_range_end:
                return int((global_angle - angle_range_start) / rad_per_beam)
            if global_angle > angle_range_end:
                return BEAM_COUNT
            return 0

        if 1.4 < self.global_angle < 1.7:
            self.orient_3d = True
            print('3d - global_angle')
            print(self.global_angle)
            self.navigate_3d(False)
        else:
            print('orienting 3d global angle')
            self.orient_3d = False
            go_to_beam = global_angle_to_beam_number(self.global_angle)
            print(go_to_beam)
            self.publish_heading(go_to_beam, 3)

    def check_for_boundedness(self, obstacle_free_beam_numbers):
        self.right_goal = False
        self.left_goal = False
        print(obstacle_free_beam_numbers)
        if not self.turn_around:
            if 1.5 <= self.global_angle <= 3.14:
                self.right_goal = True
                self.left_goal = False
            elif 0.0 <= self.global_angle < 1.5:
                self.left_goal = True
                self.right_goal = False

            beams_to_check_1 = {0, 5, 10, 15, 20, 25, 30, 35}
            beams_to_check_2 = {510, 505, 500, 495, 490, 485, 480}
            has_beam_0 = beams_to_check_1.issubset(obstacle_free_beam_numbers)
            has_beam_512 = beams_to_check_2.issubset(obstacle_free_beam_numbers)
            if has_beam_0 and self.right_goal:
                return 1
            if has_beam_512 and self.left_goal:
                return 1
            if has_beam_0 and self.left_goal:
                return 2
            if has_beam_512 and self.right_goal:
                return 2
            print('detected object not bounded')
        return 4

    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        total_beams = BEAM_COUNT
        angle_per_beam = 90.0 / total_beams
        angle_range_start = 0.7853
        angle_range_end = 2.35619
        rad_per_beam = 1.57 / total_beams
        required_angle = 15.0
        required_beams = int(required_angle / (angle_per_beam * 5))

        def global_angle_to_beam_number(global_angle):
            if angle_range_start <= global_angle <= angle_range_end:
                return int((global_angle - angle_range_start) / rad_per_beam)
            if global_angle > angle_range_end:
                return BEAM_COUNT
            return 0

        obstacle_free_beam_numbers.sort()
        mid_beam = []
        for i in range(len(obstacle_free_beam_numbers) - required_beams):
            if obstacle_free_beam_numbers[i + required_beams] - obstacle_free_beam_numbers[i] == 5 * required_beams:
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])

        target_beam_number = global_angle_to_beam_number(self.global_angle)
        self.target_beam = target_beam_number
        if mid_beam:
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))
            return True, closest_to_target
        return False, None

    def publish_heading(self, beam_number, move):
        if not self.turn_around:
            if move == 1 or move == 3:
                total_angle_degrees = 90.0
                num_beams = BEAM_COUNT
                angle_per_beam_degrees = total_angle_degrees / num_beams
                desired_heading_degrees = (beam_number - (num_beams // 2)) * angle_per_beam_degrees
                desired_heading_radians = math.radians(desired_heading_degrees)
                kp = 0.12
                kv = 0.35
                angular_velocity = kp * desired_heading_radians
                linear_velocity_y = 0.0
                linear_velocity_x = 0.0
                if move == 1:
                    linear_velocity_x = kv * (1.6 - abs(desired_heading_radians))
                    linear_velocity_x = max(0.0, min(1.0, linear_velocity_x))
            elif move == 2:
                if self.contour_start_pose is None:
                    self.contour_start_pose = (self.x, self.y, self.z)
                self.contour_steps += 1
                dist = math.sqrt(
                    (self.x - self.contour_start_pose[0]) ** 2 +
                    (self.y - self.contour_start_pose[1]) ** 2)
                if dist > self.contour_escape_distance:
                    self.contour_start_pose = None
                    self.contour_steps = 0
                    self.data_available = False
                desired_heading_radians = float(beam_number)
                angular_velocity = 0.1 * desired_heading_radians
                linear_velocity_x = self.Kv * (
                    math.pi / 2 - abs(angular_velocity))
                linear_velocity_x = max(0.0, min(linear_velocity_x, 0.5))
                linear_velocity_y = 0.0
                print(f'entered contour based navigation with '
                      f'desired slope: {self.dominant_slope}, '
                      f'Vx: {linear_velocity_x:.3f}, '
                      f'angular: {angular_velocity:.3f}')
            else:
                angular_velocity = 0.0
                linear_velocity_x = 0.0
                linear_velocity_y = 0.0
        else:
            angular_velocity = 1.0
            linear_velocity_x = 0.0
            linear_velocity_y = 0.0

        twist = Twist()
        twist.angular.z = float(angular_velocity)
        twist.linear.x = float(linear_velocity_x)
        twist.linear.y = float(linear_velocity_y)
        self.cmd_vel_pub.publish(twist)

    def avoidance(self):
        if not self.in_z_bound:
            self.check_verti_go = True
            self.navigate_3d(False)
        if self.in_z_bound:
            self.process_data()

    def move_sonar(self):
        if self.start_3d_time is None:
            self.start_3d_time = time.monotonic()
        current_time = time.monotonic() - self.start_3d_time
        cycle_duration = 0.5 / self.frequency
        if current_time < cycle_duration:
            phase_time = current_time if self.ttd else current_time + cycle_duration
            z_position = self.amplitude * math.sin(2.0 * math.pi * self.frequency * phase_time + math.pi / 2.0) + self.offset
            degree_angle = int(z_position * (180.0 / math.pi))
            self.sonar_angle = round(degree_angle / 3.0) * 3
            self.joint_pub.publish(Float64(data=float(z_position)))
            print(f'[SONAR_PIVOT] sonar_angle={z_position:.3f}rad sweeping=true')
            self.process_3d_data(self.sonar_angle)
        else:
            self.latest_scan_available = True
            self.ttd = not self.ttd
            self.min_range = min(self.min_ranges) - 2.0 if self.min_ranges else 7.0
            self.min_ranges = []
            print('completed a full cycle.')
            self.pose_before_z_motion = self.pose
            self.navigate_3d(False)

    def process_3d_data(self, sonar_angle):
        central_hits = []
        for i in range(100, 400, 7):
            if self.beam_hit[i]:
                central_hits.append(self.beam_range[i])
        if central_hits:
            print('exced threshold')
            self.min_ranges.append(min(central_hits))
        else:
            self.go_vertical_angle.append(int(sonar_angle))

    def move_sonar_extreme(self):
        self.joint_pub.publish(Float64(data=0.8))

    def start_data_collection(self):
        self.collect_3d_data_flag = True
        self.start_3d_time = time.monotonic()

    def stop_data_collection(self):
        print('stop_data_collection')
        self.collect_3d_data_flag = False
        self.joint_pub.publish(Float64(data=0.0))
        self.joint_pub.publish(Float64(data=0.0))
        self.sonar_centered = True
        print('[SONAR_PIVOT] sonar_angle=0.000rad sweeping=false')

    def navigate_3d(self, xy_called):
        if self.latest_scan_available:
            self.latest_scan_available = False
            mid_beam = []
            required_beams = 5
            obstacle_free_beam_numbers = self.go_vertical_angle
            print(f'go_vertical_angle {self.go_vertical_angle}')
            self.go_vertical_angle = []
            if obstacle_free_beam_numbers:
                obstacle_free_beam_numbers = np.unique(np.array(obstacle_free_beam_numbers)).tolist()
                for i in range(len(obstacle_free_beam_numbers) - required_beams):
                    if obstacle_free_beam_numbers[i + required_beams] - obstacle_free_beam_numbers[i] == 3 * required_beams:
                        mid_index = i + required_beams // 2
                        mid_beam.append(obstacle_free_beam_numbers[mid_index])
            if mid_beam:
                self.closest_to_target_vertical = min(mid_beam, key=lambda x: abs(x - self.angle_z_goal))
                self.new_midb_available = True
                self.z_motion_ongoing = True
                self.start_data_collection()
            else:
                self.z_motion_ongoing = False
                self.stop_data_collection()
                if xy_called:
                    self.xy_called = True
                    print(' going back to process data')
                else:
                    self.xy_called = False
                    print('stop_data_collection xy not called')
                    self.process_data()
        else:
            self.collect_3d_data_flag = True
            if self.orient_3d:
                self.start_data_collection()
                if self.sonar_centered:
                    self.move_sonar_extreme()

    def move_in_z(self):
        if self.closest_to_target_vertical is None:
            return
        angle = self.closest_to_target_vertical
        if angle == 0:
            angle = 1
        linear_x = 0.5
        linear_z = linear_x * math.tan(angle * (math.pi / 180.0))
        print(f'losest_to_target_vertical{self.closest_to_target_vertical}')
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.z = float(linear_z)
        self.cmd_vel_pub.publish(twist)

    def compute_distance(self, pose1, pose2):
        pos1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        pos2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        return float(np.linalg.norm(pos2 - pos1))

    def run_once(self):
        if self.first_cycle:
            time.sleep(0.1)
            self.spawn_marker()
            print('1st time')
            self.stop_data_collection()
            self.first_cycle = False

        if not self.data_available:
            return

        if self.z_motion_ongoing:
            print('z motion ongoing')
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


def main(args=None):
    rclpy.init(args=args)
    node = SonarHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
