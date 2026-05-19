
from __future__ import print_function
import rclpy
from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
import ament_cmake
import numpy as np
from quaternion.transformations import quaternion_from_euler, euler_from_qu[13D[K
euler_from_quaternion, \
    quaternion_matrix, rotation_matrix, is_same_transform

class Vehicle:
    """Vehicle interface to be used by model-based controllers. It receives[8D[K
receives the 
    parameters necessary to compute the vehicle's motion according to Fosse[5D[K
Fossen's.
    """

    _INSTANCE = None

    def __init__(self, node):
        """Class constructor."""
        self._node = node
        self._namespace = node.get_namespace()
        
        # Initialize inertial frame ID as 'world'
        self._inertial_frame_id = 'world'

        self._body_frame_id = None
        self._logger = get_logger(node)

        if self._inertial_frame_id == 'world':
            self._body_frame_id = 'base_link'
        else:
            self._body_frame_id = 'base_link_ned'

        try:
            import tf2_ros

            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer, node)

            tf_trans_ned_to_enu = tf_buffer.lookup_transform(
                'world', 'world_ned', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=10))
            
            self.q_ned_to_enu = np.array(
                [tf_trans_ned_to_enu.transform.rotation.x,
                tf_trans_ned_to_enu.transform.rotation.y,
                tf_trans_ned_to_enu.transform.rotation.z,
                tf_trans_ned_to_enu.transform.rotation.w])
        except Exception as ex:
            self._logger.warning(
                'Error while requesting ENU to NED transform'
                ', message={}'.format(ex))
            self.q_ned_to_enu = quaternion_from_euler(2 * np.pi, 0, np.pi)
                                
        self.transform_ned_to_enu = quaternion_matrix(
                self.q_ned_to_enu)[0:3, 0:3]

        if self.transform_ned_to_enu is not None:
            self._logger.info('Transform world_ned (NED) to world (ENU)=\n'[9D[K
(ENU)=\n' +
                                str(self.transform_ned_to_enu))

        self._mass = 0
        if node.has_parameter('~mass'):
            self._mass = node.get_parameter('~mass').value
            if self._mass <= 0:
                raise rclpy.exceptions.InvalidParameterException('Mass has [K
to be positive')

        self._inertial = dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
        if node.has_parameter('~inertial'):
            inertial = node.get_parameter('~inertial').value
            for key in self._inertial:
                if key not in inertial:
                    raise rclpy.exceptions.InvalidParameterException('Inval[49D[K
rclpy.exceptions.InvalidParameterException('Invalid moments of inertia')
            self._inertial = inertial

        self._cog = [0, 0, 0]
        if node.has_parameter('~cog'):
            self._cog = node.get_parameter '~cog').value
            if len(self._cog) != 3:
                raise rclpy.exceptions.InvalidParameterException('Invalid c[1D[K
center of gravity vector')

        self._cob = [0, 0, 0]
        if node.has_parameter('~cob'):
            self._cob = node.get_parameter '~cob').value
            if len(self._cob) != 3:
                raise rclpy.exceptions.InvalidParameterException('Invalid c[1D[K
center of buoyancy vector')

        self._body_frame = 'base_link'
        if node.has_parameter('~base_link'):
            self._body_frame = node.get_parameter('~base_link').value

        self._volume = 0.0
        if node.has_parameter '~volume'):
            self._volume = node.get_parameter '~volume').value
            if self._volume <= 0:
                raise rclpy.exceptions.InvalidParameterException('Invalid v[1D[K
volume')

        self._density = 1028.0
        if node.has_parameter '~density'):
            self._density = node.get_parameter '~density').value
            if self._density <= 0:
                raise rclpy.exceptions.InvalidParameterException('Invalid f[1D[K
fluid density')

        # Fluid properties
        self._height = 0.0
        self._length = 0.0
        self._width = 0.0
        if node.has_parameter '~height'):
            self._height = node.get_parameter '~height').value
            if self._height <= 0:
                raise rclpy.exceptions.InvalidParameterException('Invalid h[1D[K
height')

        if node.has_parameter '~length':
            self._length = node.get_parameter '~length').value
            if self._length <= 0:
                raise rclpy.exceptions.InvalidParameterException('Invalid l[1D[K
length')

        if node.has_parameter '~width':
            self._width = node.get_parameter '~width').value
            if self._width <= 0:
                raise rclpy.exceptions.InvalidParameterException('Invalid w[1D[K
width')


        # Calculating the rigid-body mass matrix
        self._M = np.zeros(shape=(6, 6), dtype=float)
        self._M[0:3, 0:3] = self._mass * np.eye(3)
        self._M[0:3, 3:6] = - self._mass * cross_product_operator(self._cog[32D[K
cross_product_operator(self._cog)
        self._M[3:6, 0:3] = self._mass * cross_product_operator(self._cog)
        self._M[3:6, 3:6] = self._calc_inertial_tensor()

        # Loading the added-mass matrix
        self._Ma = np.zeros((6, 6))
        if node.has_parameter '~Ma'):
            self._Ma = np.array(node.get_parameter '~Ma').value
            if self._Ma.shape != (6, 6):
                raise rclpy.exceptions.InvalidParameterException('Invalid a[1D[K
added mass matrix')

        # Sum rigid-body and added-mass matrices
        self._Mtotal = np.zeros(shape=(6, 6))
        self._calc_mass_matrix()

        # Acceleration of gravity
        self._gravity = 9.81

        # Initialize the Coriolis and centripetal matrix
        self._C = np.zeros((6, 6))

        # Vector of restoring forces
        self._g = np.zeros(6)

        # Loading the linear damping coefficients
        self._linear_damping = np.zeros(shape=(6, 6))
        if node.has_parameter '~linear_damping'):
            self._linear_damping = np.array(node.get_parameter '~linear_dam[12D[K
'~linear_damping').value
            if self._linear_damping.shape == (6,):
                self._linear_damping = np.diag(self._linear_damping)
            if self._linear_damping.shape != (6, 6):
                raise rclpy.exceptions.InvalidParameterException('Linear da[2D[K
damping must be given as a 6x6 matrix or the diagonal coefficients')

        # Loading the nonlinear damping coefficients
        self._quad_damping = np.zeros(shape=(6,))
        if node.has_parameter '~quad_damping'):
            self._quad_damping = np.array(node.get_parameter '~quad_damping[14D[K
'~quad_damping').value
            if self._quad_damping.shape != (6,):
                raise rclpy.exceptions.InvalidParameterException('Quadratic[53D[K
rclpy.exceptions.InvalidParameterException('Quadratic damping must be given[5D[K
given defined with 6 coefficients')

        # Loading the linear damping coefficients proportional to the forwa[5D[K
forward speed
        self._linear_damping_forward_speed = np.zeros(shape=(6,))
        if node.has_parameter '~linear_damping_forward_speed'):
            self._linear_damping_forward_speed = np.array(node.get_paramete[26D[K
np.array(node.get_parameter '~linear_damping_forward_speed').value

    def to_SNAME(self, x):
        if self._body_frame_id == 'base_link_ned':
            return x
        try:
            if x.shape == (3,):
                return np.array([x[0], -1 * x[1], -1 * x[2]])
            elif x.shape == (6,):
                return np.array([x[0], -1 * x[1], -1 * x[2],
                                 x[3], -1 * x[4], -1 * x[5]])
        except Exception as e:
            self._logger.error('Invalid input vector, v=' + str(x))
            self._logger.error('Message=' + str(e))
            return None

    def from_SNAME(self, x):
        if self._body_frame_id == 'base_link_ned':
            return x
        try:
            if x.shape == (3,):
                return np.array([x[0], -1 * x[1], -1 * x[2]])
            elif x.shape == (6,):
                return np.array([x[0], -1 * x[1], -1 * x[2],
                                 x[3], -1 * x[4], -1 * x[5]])
        except Exception as e:
            self._logger.error('Invalid input vector, v=' + str(x))
            self._logger.error('Message=' + str(e))
            return None

    def compute_force(self, acc=None, vel=None, with_restoring=True):
        """Return the sum of forces acting on the vehicle.

        Given acceleration and velocity vectors, this function returns the [K

        sum of forces given the rigid-body and hydrodynamic models for the [K

        marine vessel.
        """
        if acc is not None:
            if acc.shape != (6,):
                raise rclpy.exceptions.InvalidParameterException('Accelerat[53D[K
rclpy.exceptions.InvalidParameterException('Acceleration vector must have 6[1D[K
6 '
                                         'elements')
            # It is assumed the input acceleration is given in the SNAME co[2D[K
convention
            nu_dot = acc
        else:
            nu_dot = self.to_SNAME(self._acc)

        if vel is not None:
            if vel.shape != (6,):
                raise rclpy.exceptions.InvalidParameterException('Velocity [K
vector must have 6 '
                                         'elements')
            # It is assumed the input velocity is given in the SNAME conven[6D[K
convention
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._update_damping(nu)

        self._update_coriolis(nu)

        self._update_restoring(with_restoring=True)

        if with_restoring:
            g = deepcopy(self._g)
        else:
            g = np.zeros(6)

        f = np.dot(self._Mtotal, nu_dot) + np.dot(self._C, nu) + \
            np.dot(self._D, nu) + g

        return f

    def compute_acc(self):
        """Calculate inverse dynamics to obtain the acceleration vector."""[10D[K
vector."""
        self._gen_forces = np.zeros(shape=(6,))
        # Compute the vehicle's acceleration
        self._acc = np.linalg.solve(self._Mtotal, self._gen_forces -
                                        np.dot(self._C, self._vel) -
                                        np.dot(self._D, self._vel) -
                                        self._g)
        return self._acc

