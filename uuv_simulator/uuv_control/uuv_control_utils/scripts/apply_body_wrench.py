
#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from ros_gz_interfaces.srv import ApplyLinkWrench


class ApplyBodyWrenchNode(Node):

    def __init__(self):

        super().__init__('apply_body_wrench')

        self.declare_parameter(
            'starting_time', 0.0)

        self.declare_parameter(
            'duration', 1.0)

        self.declare_parameter(
            'force',
            [0.0, 0.0, 0.0])

        self.declare_parameter(
            'torque',
            [0.0, 0.0, 0.0])

        starting_time = self.get_parameter(
            'starting_time').value

        duration = self.get_parameter(
            'duration').value

        force = self.get_parameter(
            'force').value

        torque = self.get_parameter(
            'torque').value

        self.client = self.create_client(
            ApplyLinkWrench,
            '/world/default/apply_link_wrench'
        )

        while not self.client.wait_for_service(
                timeout_sec=2.0):

            self.get_logger().info(
                'Waiting for service...'
            )

        ns = self.get_namespace().replace(
            '/', '')

        if ns == '':
            body_name = 'base_link'
        else:
            body_name = f'{ns}/base_link'

        request = ApplyLinkWrench.Request()

        request.link_name = body_name

        request.force = Vector3(
            x=float(force[0]),
            y=float(force[1]),
            z=float(force[2])
        )

        request.torque = Vector3(
            x=float(torque[0]),
            y=float(torque[1]),
            z=float(torque[2])
        )

        request.duration.sec = int(
            duration)

        request.duration.nanosec = int(
            (duration % 1.0) * 1e9
        )

        future = self.client.call_async(
            request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result():

            self.get_logger().info(
                'Body wrench applied'
            )

        else:

            self.get_logger().error(
                'Failed'
            )

        self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ApplyBodyWrenchNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
