
import rclpy
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat


class SetScalarParameter(Node):

    def __init__(self):
        super().__init__('set_scalar_parameter')

        self.declare_parameter('service_name', '')
        self.declare_parameter('data', 0.0)

        service_name = self.get_parameter(
            'service_name').value

        data = self.get_parameter(
            'data').value

        self.client = self.create_service_client(
            SetFloat,
            service_name
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = SetFloat.Request()
        req.data = float(data)

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Scalar parameter updated')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SetScalarParameter()
    try:
        node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


The changes made include:

1. Replacing `rospy` with `rclpy`.
2. Replacing `tf` with `tf2_ros`.
3. Removing the `catkin_python_setup()` function.
build dependencies.
5. Converting Python code:
   - Replaced `rospy.Publisher` with `self.create_publisher()`.
   - Replaced `rospy.Subscriber` with `self.create_subscription()`.
   - Replaced `rospy.get_param` with `declare_parameter`.
   - Replaced `rospy.Time.now` with `node.get_clock().now()`.
   - Replaced `rospy.get_time` with `clock.nanoseconds`.

