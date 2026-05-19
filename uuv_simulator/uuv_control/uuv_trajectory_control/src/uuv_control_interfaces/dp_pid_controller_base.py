
import numpy as np
from rclpy import RealTimestamp
from tf2_ros import Buffer, TransformException
from .dp_controller_base import DPControllerBase


class DPPIDControllerBase(DPControllerBase):
    """Abstract class for PID-based controllers. The base 
    class method `update_controller` must be overridden 
    in other for a controller to work.
    """

    def __init__(self, *args):
        # Start the super class
        DPControllerBase.__init__(self, *args)
        self._logger.info('Initializing: PID controller')
        # Proportional gains
        self._Kp = np.zeros(shape=(6, 6))
        # Derivative gains
        self._Kd = np.zeros(shape=(6, 6))
        # Integral gains
        self._Ki = np.zeros(shape=(6, 6))
        # Integrator component
        self._int = np.zeros(6)
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)

        if self.node.has_parameter('~Kp'):
            Kp_diag = self.node.get_parameter '~Kp').value
            if len(Kp_diag) == 6:
                self._Kp = np.diag(Kp_diag)
            else:
                raise rclpy.exceptions.ROSException('Kp matrix error: 6 coe[3D[K
coefficients needed')

        self._logger.info('Kp=' + str([self._Kp[i, i] for i in range(6)]))

        if self.node.has_parameter('~Kd'):
            Kd_diag = self.node.get_parameter('~Kd').value
            if len(Kd_diag) == 6:
                self._Kd = np.diag(Kd_diag)
            else:
                raise rclpy.exceptions.ROSException('Kd matrix error: 6 coe[3D[K
coefficients needed')

        self._logger.info('Kd=' + str([self._Kd[i, i] for i in range(6)]))

        if self.node.has_parameter('~Ki'):
            Ki_diag = self.node.get_parameter '~Ki').value
            if len(Ki_diag) == 6:
                self._Ki = np.diag(Ki_diag)
            else:
                raise rclpy.exceptions.ROSException('Ki matrix error: 6 coe[3D[K
coefficients needed')

        self._logger.info('Ki=' + str([self._Ki[i, i] for i in range(6)]))

        self.node.create_service(SetPIDParams, 'set_pid_params', self.set_p[10D[K
self.set_pid_params_callback)
        self.node.create_service(GetPIDParams, 'get_pid_params', self.get_p[10D[K
self.get_pid_params_callback)

        self._logger.info('PID controller ready!')

    def _reset_controller(self):
        """Reset reference and and error vectors."""
        super(DPPIDControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(6)
        self._int = np.zeros(6)

    def set_pid_params_callback(self, request, response):
        """Service callback function to set the 
        PID's parameters
        """
        kp = request.Kp
        kd = request.Kd
        ki = request.Ki
        if len(kp) != 6 or len(kd) != 6 or len(ki) != 6:
            response.success = False
            return response
        self._Kp = np.diag(kp)
        self._Ki = np.diag(ki)
        self._Kd = np.diag(kd)
        response.success = True
        return response

    def get_pid_params_callback(self, request, response):
        """Service callback function to return 
        the PID's parameters
        """
        response.Kp = [self._Kp[i, i] for i in range(6)]
        response.Kd = [self._Kd[i, i] for i in range(6)]
        response.Ki = [self._Ki[i, i] for i in range(6)]
        return response

    def update_pid(self):
        """Return the control signal computed from the PID 
        algorithm. To implement a PID-based controller that
        inherits this class, call this function in the
        derived class' `update` method to obtain the control
        vector.

        > *Returns*

        `numpy.array`: Control signal
        """
        if not self.odom_is_init:
            return
        # Update integrator
        self._int += 0.5 * (self.error_pose_euler + self._error_pose) * sel[3D[K
self.node.get_clock().now()
        # Store current pose error
        self._error_pose = self.error_pose_euler
        return np.dot(self._Kp, self.error_pose_euler) \
            + np.dot(self._Kd, self._errors['vel']) \
            + np.dot(self._Ki, self._int)

Note that I've replaced `rospy` with `rclpy`, `tf` with `tf2_ros`, and remo[4D[K
removed the `catkin_python_setup()` function. I've also replaced `rosbuild`[10D[K
`rosbuild` with `ament_cmake`. Additionally, I've replaced `rospy.Publisher[16D[K
`rospy.Publisher` and `rospy.Subscriber` with `self.create_publisher()` and[3D[K
and `self.create_subscription()`, respectively.

