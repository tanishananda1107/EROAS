
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty

from .models import Thruster


class ThrusterManager(Node):

    MAX_THRUSTERS = 16

    def __init__(self):
        super().__init__('thruster_manager')

        self._ready = False

        self.namespace = self.get_namespace()

        # ---------------- Parameters ----------------
        self.declare_parameter('thruster_manager.base_link', 'base_link')
        self.declare_parameter('thruster_manager.thruster_frame_base',
                               'thruster_')
        self.declare_parameter('thruster_manager.thruster_topic_prefix',
                               '/thrusters/')
        self.declare_parameter('thruster_manager.thruster_topic_suffix',
                               '/input')
        self.declare_parameter('thruster_manager.timeout', -1.0)
        self.declare_parameter('thruster_manager.update_rate', 50.0)
        self.declare_parameter('thruster_manager.max_thrust', 1000.0)
        self.declare_parameter('thruster_manager.conversion_fcn',
                               'proportional')
        self.declare_parameter(
            'thruster_manager.conversion_fcn_params',
            {'gain': 1.0}
        )

        self.config = {
            'base_link':
                self.get_parameter(
                    'thruster_manager.base_link').value,

            'thruster_frame_base':
                self.get_parameter(
                    'thruster_manager.thruster_frame_base').value,

            'thruster_topic_prefix':
                self.get_parameter(
                    'thruster_manager.thruster_topic_prefix').value,

            'thruster_topic_suffix':
                self.get_parameter(
                    'thruster_manager.thruster_topic_suffix').value,

            'timeout':
                self.get_parameter(
                    'thruster_manager.timeout').value,

            'update_rate':
                self.get_parameter(
                    'thruster_manager.update_rate').value,

            'max_thrust':
                self.get_parameter(
                    'thruster_manager.max_thrust').value,

            'conversion_fcn':
                self.get_parameter(
                    'thruster_manager.conversion_fcn').value,

            'conversion_fcn_params':
                self.get_parameter(
                    'thruster_manager.conversion_fcn_params').value
        }

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_link_ned_to_enu = None

        # ---------------- URDF ----------------
        self.use_robot_descr = False
        self.axes = {}

        self.declare_parameter('robot_description', '')

        robot_description = self.get_parameter(
            'robot_description').value

        if robot_description != '':
            self.use_robot_descr = True
            self.parse_urdf(robot_description)

        # ---------------- Output directory ----------------
        self.declare_parameter('output_dir', '')

        self.output_dir = self.get_parameter('output_dir').value

        if self.output_dir != '':
            if not isdir(self.output_dir):
                raise RuntimeError(
                    f'Invalid output directory {self.output_dir}'
                )

        # ---------------- Variables ----------------
        self.n_thrusters = 0
        self.thrusters = []
        self.thrust = None

        self.configuration_matrix = None
        self.inverse_configuration_matrix = None

        # ---------------- Build TAM ----------------
        if not self.update_tam():
            raise RuntimeError('No thrusters found')

        self.inverse_configuration_matrix = numpy.linalg.pinv(
            self.configuration_matrix
        )

        if self.output_dir != '':
            with open(join(self.output_dir, 'TAM.yaml'), 'w') as yaml_file:[10D[K
yaml_file:
                yaml.safe_dump(
                    dict(tam=self.configuration_matrix.tolist()),
                    yaml_file
                )

        self.ready = True

        self.get_logger().info('ThrusterManager ready')

    # ... (rest of the class definition remains the same)

    def parse_urdf(self, urdf_str):

        root = etree.fromstring(urdf_str)

        for joint in root.findall('joint'):

            if joint.get('type') == 'fixed':
                continue

            axis_str_list = joint.find('axis').get('xyz').split()

            child = joint.find('child').get('link')

            if child[0] != '/':
                child = '/' + child

            self.axes[child] = numpy.array([
                float(axis_str_list[0]),
                float(axis_str_list[1]),
                float(axis_str_list[2]),
                0.0
            ])

    # ... (rest of the class definition remains the same)

    def update_tam(self, recalculate=False):

        if self.configuration_matrix is not None and not recalculate:
            self.ready = True
            return True

        self.ready = False

        self.get_logger().info(
            'ThrusterManager updating thruster poses'
        )

        base = self.namespace + self.config['base_link']

        self.thrusters = []

        equal_thrusters = True
        idx_thruster_model = 0

        if isinstance(self.config['conversion_fcn_params'], list) and \
           isinstance(self.config['conversion_fcn'], list):

            if len(self.config['conversion_fcn_params']) != \
               len(self.config['conversion_fcn']):

                raise RuntimeError(
                    'conversion_fcn lists mismatch'
                )

            equal_thrusters = False

        sleep(1.0)

        for i in range(self.MAX_THRUSTERS):

            frame = (
                self.namespace +
                self.config['thruster_frame_base'] +
                str(i)
            )

            try:

                tf_trans = self.tf_buffer.lookup_transform(
                    base,
                    frame,
                    node.get_clock().now(),
                    timeout=Duration(seconds=1.0)
                )

                pos = [
                    tf_trans.transform.translation.x,
                    tf_trans.transform.translation.y,
                    tf_trans.transform.translation.z
                ]

                quat = [
                    tf_trans.transform.rotation.x,
                    tf_trans.transform.rotation.y,
                    tf_trans.transform.rotation.z,
                    tf_trans.transform.rotation.w
                ]

                topic = (
                    self.config['thruster_topic_prefix'] +
                    str(i) +
                    self.config['thruster_topic_suffix']
                )

                thrust_axis = (
                    self.axes[frame]
                    if self.use_robot_descr else None
                )

                if equal_thrusters:

                    params = self.config['conversion_fcn_params']

                    thruster = Thruster.create_thruster(
                        self.config['conversion_fcn'],
                        self,
                        i,
                        topic,
                        pos,
                        quat,
                        thrust_axis,
                        **params
                    )

                else:

                    params = self.config[
                        'conversion_fcn_params'
                    ][idx_thruster_model]

                    conv_fcn = self.config[
                        'conversion_fcn'
                    ][idx_thruster_model]

                    thruster = Thruster.create_thruster(
                        conv_fcn,
                        self,
                        i,
                        topic,
                        pos,
                        quat,
                        thrust_axis,
                        **params
                    )

                    idx_thruster_model += 1

                self.thrusters.append(thruster)

            except Exception as e:

                self.get_logger().info(
                    f'Could not get transform {base} -> {frame}: {e}'
                )

                break

        if len(self.thrusters) == 0:
            return False

        self.n_thrusters = len(self.thrusters)

        self.thrust = numpy.zeros(self.n_thrusters)

        self.configuration_matrix = numpy.zeros(
            (6, self.n_thrusters)
        )

        for i in range(self.n_thrusters):
            self.configuration_matrix[:, i] = \
                self.thrusters[i].tam_column

        self.configuration_matrix[
            numpy.abs(self.configuration_matrix) < 1e-3
        ] = 0.0

        self.inverse_configuration_matrix = numpy.linalg.pinv(
            self.configuration_matrix
        )

        self.ready = True

        self.get_logger().info('ThrusterManager ready')

        return True

    # ... (rest of the class definition remains the same)

    def command_thrusters(self):

        if self.thrust is None:
            return

        for i in range(self.n_thrusters):
            self.thrusters[i].publish_command(
                self.thrust[i]
            )

    # ... (rest of the class definition remains the same)

    def publish_thrust_forces(
        self,
        control_forces,
        control_torques,
        frame_id=None
    ):

        if not self.ready:
            return

        gen_forces = numpy.hstack(
            (control_forces, control_torques)
        ).transpose()

        self.thrust = self.compute_thruster_forces(
            gen_forces
        )

        self.command_thrusters()

    # ... (rest of the class definition remains the same)

    def compute_thruster_forces(self, gen_forces):

        thrust = self.inverse_configuration_matrix.dot(
            gen_forces
        )

        if isinstance(self.config['max_thrust'], list):

            max_thrust = self.config['max_thrust']

        else:

            max_thrust = [
                self.config['max_thrust']
                for _ in range(self.n_thrusters)
            ]

        for i in range(self.n_thrusters):

            if abs(thrust[i]) > max_thrust[i]:

                thrust[i] = (
                    numpy.sign(thrust[i]) *
                    max_thrust[i]
                )

        return thrust

Note that I've replaced `ros1` imports with their `ros2` equivalents, and m[1D[K
made the necessary changes to use `rclpy` and `Node` instead of `rospy`. Ad[2D[K
Additionally, I've replaced `rosbuild` with `ament_cmake`, and updated the [K
package.xml file accordingly.

