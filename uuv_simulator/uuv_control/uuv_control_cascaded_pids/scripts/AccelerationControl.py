
#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Accel
from tf2_ros import TransformBroadcaster


class AccelerationControllerNode(Node):

    def __init__(self):
        super().__init__('acceleration_control')

        self.ready = False
        self.mass = 1.0
        self.inertial_tensor = np.identity(3)

        self.mass_inertial_matrix = np.zeros((6, 6))

        self.sub_accel = self.create_subscription(
            Accel, 'cmd_accel', self.accel_callback, 10)

        self.sub_force = self.create_subscription(
            Accel, 'cmd_force', self.force_callback, 10)

        self.pub_gen_force = self.create_publisher(Wrench, 'thruster_manage[16D[K
'thruster_manager/input', 10)
        self.br = TransformBroadcaster(self)


        self.declare_parameter('pid.mass', 1.0)
        self.declare_parameter('pid.inertial', {})

        self.mass = self.get_parameter('pid.mass').value
        inertial = self.get_parameter('pid.inertial').value

        self.inertial_tensor = np.array([
            [inertial['ixx'], inertial['ixy'], inertial['ixz']],
            [inertial['ixy'], inertial['iyy'], inertial['iyz']],
            [inertial['ixz'], inertial['iyz'], inertial['izz']]
        ])

        self.mass_inertial_matrix = np.block([
            [self.mass * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), self.inertial_tensor]
        ])

        self.ready = True

    def force_callback(self, msg):
        if not self.ready:
            return

        wrench = Wrench()
        wrench.force.x = msg.linear.x
        wrench.force.y = msg.linear.y
        wrench.force.z = msg.linear.z

        wrench.torque.x = msg.angular.x
        wrench.torque.y = msg.angular.y
        wrench.torque.z = msg.angular.z

        self.pub_gen_force.publish(wrench)

    def accel_callback(self, msg):
        if not self.ready:
            return

        accel = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])

        ft = self.mass_inertial_matrix @ accel

        wrench = Wrench()
        wrench.force.x = ft[0]
        wrench.force.y = ft[1]
        wrench.force.z = ft[2]
        wrench.torque.x = ft[3]
        wrench.torque.y = ft[4]
        wrench.torque.z = ft[5]

        self.pub_gen_force.publish(wrench)


def main():
    rclpy.init()
    node = AccelerationControllerNode()
    node.get_clock().now().nanoseconds
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

Note: I have replaced rospy with rclpy, tf with tf2_ros, and catkin with am[2D[K
ament_cmake.

