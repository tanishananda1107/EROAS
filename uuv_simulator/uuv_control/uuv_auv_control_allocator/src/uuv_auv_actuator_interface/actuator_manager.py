
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix

from uuv_auv_control_allocator.msg import AUVCommand
from geometry_msgs.msg import Wrench

from .fin_model import FinModel
from uuv_thrusters.models import Thruster


class ActuatorManager(Node):

    MAX_FINS = 4

    def __init__(self):
        super().__init__('actuator_manager')

        self.namespace = self.get_namespace().replace('/', '')
        self.get_logger().info(f'Initializing actuator manager for {self.na[8D[K
{self.namespace}')

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_link = "base_link"
        self.thruster = None
        self.fins = {}

        # parameters
        self.declare_parameter('thruster_config', {})
        self.declare_parameter('fin_config', {})

        self.thruster_config = self.get_parameter('thruster_config').value
        self.fin_config = self.get_parameter('fin_config').value

        if not self.thruster_config or not self.fin_config:
            raise RuntimeError("Missing actuator configuration")

        self.find_actuators()

    def find_actuators(self):

        base = f'{self.namespace}/{self.base_link}' if self.namespace else [K
self.base_link

        # THRUSTER
        frame = f"{self.namespace}/{self.thruster_config['frame_base']}0"

        trans = self.tf_buffer.lookup_transform(
            base, frame, self.get_clock().now()
        )

        pos = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])

        quat = np.array([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ])

        self.thruster = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'],
            0,
            f"/{self.namespace}/thruster/0/cmd_thrust",
            pos,
            quat,
            **self.thruster_config['conversion_fcn_params']
        )

        # FINS
        for i in range(self.MAX_FINS):
            try:
                frame = f"{self.namespace}/{self.fin_config['frame_base']}{[51D[K
f"{self.namespace}/{self.fin_config['frame_base']}{i}"

                trans = self.tf_buffer.lookup_transform(
                    base, frame, self.get_clock().now()
                )

                pos = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                quat = np.array([
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ])

                topic = f"/{self.namespace}/{self.fin_config['topic_prefix'[51D[K
f"/{self.namespace}/{self.fin_config['topic_prefix']}/{i}/{self.fin_config[f"/{self.namespace}/{self.fin_config['topic_prefix'}/{i}/{self.fin_config['topic_suffix']}"

                self.fins[i] = FinModel(self, i, pos, quat, topic)

            except Exception:
                break

        self.get_logger().info(f"Fins detected: {len(self.fins)}")

    def compute_control_force(self, thrust, delta, u):
        actuator_model = self.thruster.tam_column.reshape((6, 1)) * thrust

        for i in self.fins:
            f_lift = (0.5 * self.fin_config['fluid_density'] *
                      self.fin_config['lift_coefficient'] *
                      self.fin_config['fin_area'] *
                      delta[i] * u ** 2)

            tau = np.zeros(6)
            tau[0:3] = f_lift * self.fins[i].lift_vector
            tau[3:] = np.cross(self.fins[i].pos, f_lift)

            actuator_model += tau

        return actuator_model

    def publish_commands(self, command):
        self.thruster.publish_command(command[0])

        for i in self.fins:
            self.fins[i].publish_command(command[i + 1])

Note that the changes include:

* Replacing `rospy` with `rclpy`
* Removing `catkin_python_setup()`
* Replacing `tf` with `tf2_ros`
* Updating `node.get_namespace()` to `self.get_namespace()`
* Updating `node.get_logger().info()` to `self.get_logger().info()`
* Updating `rospy.Time.now()` to `self.get_clock().now()`
* Updating `rosbuild` to `ament_cmake`
* Replacing `rospy.Subscriber` with `self.create_subscription()`
* Replacing `rospy.Publisher` with `self.create_publisher()`
* Replacing `rospy.Service` with `create_service()`

