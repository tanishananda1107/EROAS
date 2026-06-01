#!/usr/bin/env python3
# ROS 2 port of sonar_obs_avoidance_merged_old.py
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        self.create_subscription(
            ProjectedSonarImage,
            '/rexrov2/blueview_p900/sonar_image_raw',
            self.sonar_image_raw_callback,
            10)

        self.create_subscription(
            Float32,
            '/rexrov2/dominant_slope',
            self.dominant_slope_callback,
            10)

        self.beam_directions = []
        self.ranges          = []
        self.data            = []
        self.ping_info       = None
        self.criti_cal       = False
        self.dominant_slope  = None

        # 20 Hz timer replaces rospy.Rate(20)
        self.timer = self.create_timer(1.0 / 20.0, self.process_data)

    # ------------------------------------------------------------------ #
    def sonar_image_raw_callback(self, data):
        self.beam_directions = data.beam_directions
        self.ranges          = data.ranges
        self.data            = data.image.data
        self.ping_info       = data.ping_info

    def dominant_slope_callback(self, msg):
        self.dominant_slope = msg.data

    # ------------------------------------------------------------------ #
    def process_data(self):
        if not (self.beam_directions and self.ranges
                and self.data and self.ping_info):
            return

        beam_count  = 512
        range_count = len(self.ranges)

        obstacle_free_beam_numbers = []
        self.criti_cal = False

        # First pass — skip early rows
        for i in range(beam_count - 20):
            hit = False
            for j in range(50, range_count - 90):
                if self.data[beam_count * j + i] > 20:
                    hit = True
                    break
            if not hit:
                obstacle_free_beam_numbers.append(i)

        # Second pass if first found nothing
        if not obstacle_free_beam_numbers:
            for i in range(beam_count):
                hit = False
                for j in range(range_count - 20):
                    if self.data[beam_count * j + i] > 20:
                        hit = True
                        break
                if not hit:
                    obstacle_free_beam_numbers.append(i)

        if obstacle_free_beam_numbers:
            ok, beam = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
            if ok:
                self.publish_heading(beam, True)
            else:
                self.publish_heading(256, False)

    # ------------------------------------------------------------------ #
    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        total_beams    = 512
        angle_per_beam = 90.0 / total_beams
        required_beams = int(25.0 / angle_per_beam)

        obstacle_free_beam_numbers.sort()
        mid_beam = []
        for i in range(len(obstacle_free_beam_numbers) - required_beams + 1):
            if (obstacle_free_beam_numbers[i + required_beams - 1]
                    - obstacle_free_beam_numbers[i] + 1 == required_beams):
                mid_beam.append(
                    obstacle_free_beam_numbers[i + required_beams // 2])

        if mid_beam:
            closest = min(mid_beam, key=lambda b: abs(b - 256))
            return True, closest
        return False, None

    # ------------------------------------------------------------------ #
    def publish_heading(self, beam_number, move):
        twist = Twist()
        if move:
            num_beams            = 512
            angle_per_beam_deg   = 90.0 / num_beams
            desired_heading_deg  = (beam_number - num_beams // 2) * angle_per_beam_deg
            desired_heading_rad  = math.radians(desired_heading_deg)

            Kp = 0.3
            Kv = 1.0

            twist.angular.z  = Kp * desired_heading_rad
            twist.linear.x   = Kv * (1.6 - abs(desired_heading_rad))
            twist.linear.y   = 0.0
        else:
            # Contour-following mode
            if self.dominant_slope and self.dominant_slope != 0:
                twist.linear.x  = 0.2
                twist.linear.y  = -0.2 / self.dominant_slope
            else:
                twist.linear.x  = 0.2
                twist.linear.y  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(
                f'Contour nav — slope: {self.dominant_slope}, '
                f'Vx: {twist.linear.x:.3f}, Vy: {twist.linear.y:.3f}')

        self.cmd_vel_pub.publish(twist)


# ------------------------------------------------------------------ #
def main():
    rclpy.init()
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
