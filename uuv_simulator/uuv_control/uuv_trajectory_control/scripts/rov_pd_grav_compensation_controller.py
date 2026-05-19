
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import numpy as np
from tf2_ros import Buffer, TransformException
from std_msgs.msg import Float64

class ROVPDGravityComp(Node):

    def __init__(self):
        super().__init__('rov_pd_gravity_comp')

        self.pub = self.create_publisher(Wrench, 'cmd_wrench', 10)
        self.timer = self.create_timer(0.1, self.update)

        self.Kp = np.diag(np.ones(6))
        self.Kd = np.diag(np.ones(6))

    def get_error(self):
        return np.zeros(6)

    def gravity(self):
        return np.zeros(6)

    def update(self):
        e = self.get_error()
        de = np.zeros(6)

        tau = self.Kp @ e + self.Kd @ de + self.gravity()

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = tau[:3]
        msg.torque.x, msg.torque.y, msg.torque.z = tau[3:]

        self.pub.publish(msg)


def main():
    rclpy.init(args=None)
    node = ROVPDGravityComp()
    try:
        node.get_clock().now()
    except Exception as e:
        print(f'Failed to get clock: {e}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

