#!/usr/bin/env python3
# ROS 2 port — sonar_heading_node.py
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Twist
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        self.create_subscription(ProjectedSonarImage, '/rexrov2/blueview_p900/sonar_image_raw',
                                 self.sonar_callback, 10)
        self.create_subscription(Float32, '/rexrov2/dominant_slope',  self.dominant_slope_cb, 10)
        self.create_subscription(Float64, '/rexrov2/global_angle',    self.global_angle_cb,   10)
        self.create_subscription(Float32, '/rexrov2/avg_right_slope', self.avg_right_cb,      10)
        self.create_subscription(Float32, '/rexrov2/avg_left_slope',  self.avg_left_cb,       10)

        self.beam_directions = []
        self.ranges = []
        self.data   = []
        self.ping_info = None
        self.dominant_slope = None
        self.global_angle   = 0.0
        self.turn_around    = False
        self.right_goal = self.left_goal = False
        self.avg_right_slope = self.avg_left_slope = None

        self.timer = self.create_timer(0.2, self.process_data)  # 5 Hz

    def sonar_callback(self, data):
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.data   = data.image.data
        self.ping_info = data.ping_info

    def dominant_slope_cb(self, msg):  self.dominant_slope  = msg.data
    def avg_right_cb(self, msg):       self.avg_right_slope = msg.data
    def avg_left_cb(self, msg):        self.avg_left_slope  = msg.data

    def global_angle_cb(self, msg):
        ga = msg.data
        if   -math.pi/2 < ga < math.pi/2:  ga = math.pi/2 - ga;  self.turn_around = False
        elif  math.pi/2 < ga < math.pi:    ga = 0.0;               self.turn_around = False
        elif -math.pi   < ga < -math.pi/2: ga = 3.14;              self.turn_around = False
        self.global_angle = ga

    def _ga_to_beam(self, ga):
        start, rpb = 0.7853, 1.57/512
        if 0.7853 <= ga <= 2.35619:
            return int((ga - start) / rpb)
        return 512 if ga > 2.35619 else 0

    def check_for_boundedness(self, obs_free):
        self.right_goal = 1.5 <= self.global_angle <= 3.14
        self.left_goal  = 0   <= self.global_angle <  1.5
        has0   = 0   in obs_free
        has512 = 512 in obs_free
        if has0   and self.right_goal: return 1
        if has512 and self.left_goal:  return 1
        if has0   and self.left_goal:  return 2
        if has512 and self.right_goal: return 2
        return 4

    def check_for_10_degree_coverage(self, obs_free):
        req = int(25.0 / (90.0/512))
        obs_free.sort()
        mid = []
        for i in range(len(obs_free) - req + 1):
            if obs_free[i+req-1] - obs_free[i] + 1 == req:
                mid.append(obs_free[i + req//2])
        if not mid: return False, None
        tgt = 512 - self._ga_to_beam(self.global_angle)
        return True, min(mid, key=lambda b: abs(b - tgt))

    def publish_heading(self, beam, move):
        if self.turn_around:
            tw = Twist(); tw.angular.z = 1.0; self.cmd_vel_pub.publish(tw); return
        tw = Twist()
        if move in (1, 2):
            rad = math.radians((beam - 256) * 90.0/512)
            tw.angular.z = 0.3 * rad
            if move == 1:
                tw.linear.x = 1.0 * (1.6 - abs(rad))
            elif move == 2:
                tw.linear.x = 0.2
        self.cmd_vel_pub.publish(tw)

    def process_data(self):
        if not (self.beam_directions and self.ranges and self.data and self.ping_info):
            return
        if self.turn_around:
            self.publish_heading(256, False); return

        beam_count  = 512
        range_count = len(self.ranges)
        obs_free = []
        for i in range(beam_count - 20):
            hit = False
            for j in range(10, range_count - 90, 2):
                if self.data[beam_count*j + i] > 20:
                    hit = True; break
            if not hit:
                obs_free.append(i)

        if obs_free:
            ok, beam = self.check_for_10_degree_coverage(obs_free)
            if ok:
                self.publish_heading(beam, 1); return
            bn = self.check_for_boundedness(obs_free)
            table = {
                (1, True,  False): (10,  1),
                (1, False, True):  (10,  1),
                (2, False, True):  (10,  1),
                (2, True,  False): (500, 1),
                (4, False, True):  (400, 2),
                (4, True,  False): (100, 2),
            }
            key = (bn, self.right_goal, self.left_goal)
            if key in table:
                self.publish_heading(*table[key])


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
