
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_publisher(Thruster, 'thrusters')
        self.create_subscription(ThrusterProportional, 'proportional_thrust[20D[K
'proportional_thrusters', lambda msg: print(msg))
        self.create_subscription(ThrusterCustom, 'custom_thrusters', lambda[6D[K
lambda msg: print(msg))
        self.declare_parameter('param_name', '')
        self.get_clock().now()

Note that I've removed the `catkin_python_setup()` and replaced it with the[3D[K
subscribers, and service to their respective ROS2 counterparts.

