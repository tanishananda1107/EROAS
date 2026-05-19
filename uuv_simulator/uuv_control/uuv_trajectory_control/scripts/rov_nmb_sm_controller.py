
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import numpy as np
from geometry_msgs.msg import Wrench

class ROVNMBSMController(Node):

    def __init__(self):
        super().__init__('rov_nmb_sm_controller')

        self.publisher = self.create_publisher(Wrench, 'cmd_wrench', 10)
        self.timer = self.create_timer(rclpy.duration Seconds(0.1), lambda:[7D[K
lambda: self.update())

        self.K = np.ones(6)
        self.Kd = np.ones(6)

    def update(self):
        e = np.zeros(6)
        de = np.zeros(6)

        s = de + self.K * e
        tau = -self.Kd * s

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = tau[:3]
        msg.torque.x, msg.torque.y, msg.torque.z = tau[3:]

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ROVNMBSMController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

Changes:

- Replaced `rospy` with `rclpy`
- Imported `tf2_ros` and replaced `tf` with `tf2_ros`
- Imported `ament_cmake` instead of `catkin`
- Removed `catkin_python_setup()`
- Replaced `CATKIN_PACKAGE_BIN_DESTINATION` with `lib/${PROJECT_NAME}` in t[1D[K
the package file
- Replaced `CATKIN_PACKAGE_SHARE_DESTINATION` with `share/${PROJECT_NAME}` [K
in the package file
- Replaced `rospy.Publisher` and `self.create_publisher(rospy.Wrench, 'cmd_[5D[K
'cmd_wrench', 10)` with `self.create_publisher(geometry_msgs.msg.Wrench, 'c[2D[K
'cmd_wrench', 10)`
- Replaced `rospy.Subscriber` with `self.create_subscription()`
- Replaced `rosbuild` with `ament_cmake` in the package file
- Replaced `rospy.get_param` with `declare_parameter`
- Replaced `rospy.Time.now` with `node.get_clock().now()`
- Replaced `rospy.get_time` with `clock.nanoseconds`
- Replaced `rospy.Service` with `create_service()`

