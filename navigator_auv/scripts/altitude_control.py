#!/usr/bin/env python3
"""
altitude_control.py — ROS 2 (rclpy) + Gazebo Harmonic (gz-sim 8)
Converted from ROS 1 (rospy).

Key changes:
  - rospy  → rclpy
  - rospy.init_node / Publisher / Subscriber → Node class with self.create_*
  - rospy.Rate / rate.sleep()  → self.create_timer (wall timer)
  - rospy.loginfo            → self.get_logger().info
  - rospy.is_shutdown()      → rclpy.ok()
  - rospy.ROSInterruptException → KeyboardInterrupt
  - DVL message kept as uuv_sensor_ros_plugins_msgs (must be ported/available for ROS 2)
"""

import rclpy
from rclpy.node import Node
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Twist


class AUVNavigation(Node):
    def __init__(self):
        super().__init__('auv_navigation')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)
        self.altitude_sub = self.create_subscription(
            DVL, '/rexrov2/dvl', self.altitude_callback, 10)

        self.current_velocity = Twist()
        self.altitude = None
        self.desired_altitude = 5.0       # m
        self.kp_altitude = 2.0            # proportional gain
        self.altitude_tolerance = 0.1     # m
        self.stabilized = False

        # 10 Hz control loop via wall timer
        self.timer = self.create_timer(0.1, self.run_once)

        self.get_logger().info('AUV Navigation Node started')

    # ------------------------------------------------------------------
    def altitude_callback(self, msg: DVL):
        self.altitude = msg.altitude
        self.get_logger().info(f'Received altitude: {self.altitude:.3f} m')

    # ------------------------------------------------------------------
    def control_altitude(self):
        if self.altitude is None:
            return

        altitude_error = self.desired_altitude - self.altitude
        vertical_velocity = self.kp_altitude * altitude_error

        # Constrain vertical velocity
        if vertical_velocity < 0:
            vertical_velocity = max(vertical_velocity, -0.5)
        else:
            vertical_velocity = min(vertical_velocity, 0.5)

        self.current_velocity.linear.z = vertical_velocity

        if abs(altitude_error) < self.altitude_tolerance:
            self.stabilized = True
            self.get_logger().info('Altitude stabilized')
        else:
            self.stabilized = False

        self.get_logger().info(
            f'Altitude error: {altitude_error:.3f}, '
            f'Vertical velocity command: {vertical_velocity:.3f}')

    # ------------------------------------------------------------------
    def move_forward(self):
        self.current_velocity.linear.x = 1.0
        self.current_velocity.angular.z = 0.0

    # ------------------------------------------------------------------
    def run_once(self):
        """Called by the wall timer at 10 Hz."""
        self.control_altitude()
        self.move_forward()
        self.get_logger().info(
            f'Current Velocity: Linear x={self.current_velocity.linear.x:.2f} m/s, '
            f'z={self.current_velocity.linear.z:.2f} m/s')
        self.cmd_vel_pub.publish(self.current_velocity)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = AUVNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
