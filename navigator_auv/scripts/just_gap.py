#!/usr/bin/env python3
"""
just_gap.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Twist
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('only_gap')

        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel_1', 10)

        self._subscriptions = [
            self.create_subscription(
                ProjectedSonarImage,
                '/rexrov2/blueview_p900/sonar_image_raw',
                self.sonar_image_raw_callback,
                10),
            self.create_subscription(Float64, '/rexrov2/global_angle', self.global_angle_callback, 10),
        ]

        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.criti_cal = False
        self.global_angle = 0.0
        self.turn_around = False

        # 5 Hz processing loop
        self.timer = self.create_timer(0.2, self.process_data)

    # ------------------------------------------------------------------
    def global_angle_callback(self, msg: Float64):
        self.global_angle = msg.data
        if (-math.pi / 2) < self.global_angle < (math.pi / 2):
            self.global_angle = math.pi / 2 - self.global_angle
            self.turn_around = False
        elif (math.pi / 2) < self.global_angle < math.pi:
            self.global_angle = 0.0
            self.turn_around = False
        elif -math.pi < self.global_angle < -(math.pi / 2):
            self.global_angle = 3.14
            self.turn_around = False

    def sonar_image_raw_callback(self, data: ProjectedSonarImage):
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.data = data.image.data
        self.ping_info = data.ping_info

    # ------------------------------------------------------------------
    def process_data(self):
        if self.turn_around:
            self.publish_heading(256, False)
            return

        if not (self.beam_directions and self.ranges and self.data and self.ping_info):
            return

        beam_count = 512
        range_count = len(self.ranges)
        obstacle_free_beam_numbers = []

        for i in range(beam_count - 20):
            critical = False
            for j in range(10, range_count - 90, 2):
                if self.data[beam_count * j + i] > 20:
                    critical = True
                    break
            if not critical:
                obstacle_free_beam_numbers.append(i)

        if obstacle_free_beam_numbers:
            is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
            if is_covered:
                self.publish_heading(go_beam_no, True)
            else:
                self.publish_heading(256, False)

    # ------------------------------------------------------------------
    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        total_beams = 512
        angle_per_beam = 90.0 / total_beams
        angle_range_start = 0.7853
        angle_range_end   = 2.35619
        rad_per_beam      = 1.57 / total_beams
        required_angle    = 5.0
        required_beams    = int(required_angle / angle_per_beam)

        def global_angle_to_beam_number(ga):
            if angle_range_start <= ga <= angle_range_end:
                return int((ga - angle_range_start) / rad_per_beam)
            elif ga > 2.35619:
                return 512
            return 0

        obstacle_free_beam_numbers.sort()
        mid_beam = []
        for i in range(len(obstacle_free_beam_numbers) - required_beams + 1):
            if (obstacle_free_beam_numbers[i + required_beams - 1]
                    - obstacle_free_beam_numbers[i] + 1 == required_beams):
                mid_beam.append(obstacle_free_beam_numbers[i + required_beams // 2])

        target = global_angle_to_beam_number(self.global_angle)
        target = 512 - target
        self.get_logger().info(f'global_angle:{self.global_angle:.3f}, target beam:{target}')

        if target > 384:
            self.get_logger().info('Turn extreme left')
        elif 256 < target <= 384:
            self.get_logger().info('Turn slight left')
        elif 128 < target <= 256:
            self.get_logger().info('Turn slight right')
        else:
            self.get_logger().info('Turn extreme right')

        if mid_beam:
            closest = min(mid_beam, key=lambda x: abs(x - target))
            return True, closest
        return False, None

    # ------------------------------------------------------------------
    def publish_heading(self, beam_number, move):
        if self.turn_around:
            return
        if move:
            num_beams = 512
            angle_per_beam_degrees = 90.0 / num_beams
            desired_heading_degrees = (beam_number - num_beams // 2) * angle_per_beam_degrees
            desired_heading_radians = math.radians(desired_heading_degrees)
            Kp = 0.3
            Kv = 1.0
            angular_velocity  = Kp * desired_heading_radians
            linear_velocity_x = Kv * (1.6 - abs(desired_heading_radians))
            linear_velocity_y = 0.0
        else:
            angular_velocity  = 0.0
            linear_velocity_x = -0.25
            linear_velocity_y = 0.0

        if self.turn_around:
            angular_velocity  = 1.0
            linear_velocity_x = 0.0
            linear_velocity_y = 0.0

        twist = Twist()
        twist.angular.z  = angular_velocity
        twist.linear.x   = linear_velocity_x
        twist.linear.y   = linear_velocity_y
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Traveling to beam number: {beam_number}')


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
