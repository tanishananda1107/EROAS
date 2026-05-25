# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =============================================================================
# ROS1 → ROS2 Migration Notes:
#
#   - import rospy                        → import rclpy + from rclpy.node import Node
#   - import tf                           → import tf2_ros (tf module removed in ROS2)
#   - from rospy.numpy_msg import numpy_msg → removed; ROS2 handles numpy natively
#   - rospy.get_namespace()               → self._node.get_namespace()
#   - rospy.has_param / rospy.get_param   → declare_parameter + get_parameter
#   - rospy.Publisher()                   → node.create_publisher()
#   - rospy.Subscriber()                  → node.create_subscription()
#   - rospy.Service()                     → node.create_service()
#   - rospy.get_time()                    → self._get_time()
#   - rospy.Time.now()                    → node.get_clock().now().to_msg()
#   - rospy.ROSException                  → Exception
#   - publisher.get_num_connections()     → publisher.get_subscription_count()
#   - Service callbacks (request)         → (request, response) → return response
#   - LocalPlanner.__init__               → now requires node as first argument
#   - numpy_msg(Odometry)                 → plain Odometry (numpy bridge not needed)
# =============================================================================

from copy import deepcopy
import numpy as np
import rclpy
import rclpy.parameter
from rclpy.node import Node
import logging
import sys
import tf2_ros

from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, \
    Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from uuv_control_interfaces.vehicle import Vehicle
from tf_quaternion.transformations import euler_from_quaternion, \
    quaternion_multiply, quaternion_matrix, quaternion_conjugate, \
    quaternion_inverse
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from uuv_control_msgs.srv import ResetController
from uuv_auv_control_allocator.msg import AUVCommand

from .dp_controller_local_planner import DPControllerLocalPlanner as LocalPlanner
from ._log import get_logger


class DPControllerBase(object):
    """General abstract class for DP controllers for underwater vehicles.
    This is an abstract class, must be inherited by a controller module that
    overrides the update_controller method. If the controller is set to be
    model based (is_model_based=True), then the vehicle parameters are going
    to be read from the ROS parameter server.

    > *Input arguments*

    * `node` (*type:* `rclpy.node.Node`): The ROS2 node instance.
    * `is_model_based` (*type:* `bool`, *default:* `False`): If `True`, the
    controller uses a model of the vehicle.
    * `list_odometry_callbacks` (*type:* `list`, *default:* `None`): List of
    function handles that will be called after each odometry update.
    * `planner_full_dof` (*type:* `bool`, *default:* `False`): Set the local
    planner to generate 6 DoF trajectories.
    """

    _LABEL = ''

    def __init__(self, node: Node, is_model_based=False,
                 list_odometry_callbacks=None, planner_full_dof=True):
        # Flag will be set to true when all parameters are initialized correctly
        self._is_init = False
        self._logger = get_logger()

        # ------------------------------------------------------------------
        # Store node reference — all ROS2 API calls go through self._node
        # ------------------------------------------------------------------
        self._node = node

        # rospy.get_namespace() → node.get_namespace()
        self._namespace = self._node.get_namespace()

        self._is_model_based = is_model_based

        if self._is_model_based:
            self._logger.info('Setting controller as model-based')
        else:
            self._logger.info('Setting controller as non-model-based')

        # ------------------------------------------------------------------
        # Parameters  (declare → get pattern replaces rospy.has_param/get_param)
        # ------------------------------------------------------------------
        self._use_stamped_poses_only = False
        self._node.declare_parameter('use_stamped_poses_only', False)
        self._use_stamped_poses_only = (
            self._node.get_parameter('use_stamped_poses_only')
            .get_parameter_value().bool_value
        )

        self._node.declare_parameter('thrusters_only', True)
        self.thrusters_only = (
            self._node.get_parameter('thrusters_only')
            .get_parameter_value().bool_value
        )

        # ------------------------------------------------------------------
        # Local planner — pass node so it can make its own ROS2 calls
        # ------------------------------------------------------------------
        self._local_planner = LocalPlanner(
            node=self._node,
            full_dof=True,
            stamped_pose_only=self._use_stamped_poses_only,
            thrusters_only=self.thrusters_only,
        )

        self._control_saturation = 5000
        self._node.declare_parameter('saturation', -1.0)
        _sat = (
            self._node.get_parameter('saturation')
            .get_parameter_value().double_value
        )
        if _sat > 0:
            self._control_saturation = _sat
            if self._control_saturation <= 0:
                raise Exception('Invalid control saturation forces')

        # AUV control allocator flag
        self.use_auv_control_allocator = False
        if not self.thrusters_only:
            self._node.declare_parameter('use_auv_control_allocator', False)
            self.use_auv_control_allocator = (
                self._node.get_parameter('use_auv_control_allocator')
                .get_parameter_value().bool_value
            )

        # ------------------------------------------------------------------
        # Publishers  (rospy.Publisher → node.create_publisher)
        # ------------------------------------------------------------------
        if self.thrusters_only:
            self._thrust_pub = self._node.create_publisher(
                WrenchStamped, 'thruster_output', 1
            )
        else:
            self._thrust_pub = None

        if not self.thrusters_only:
            self._auv_command_pub = self._node.create_publisher(
                AUVCommand, 'auv_command_output', 1
            )
        else:
            self._auv_command_pub = None

        self._node.declare_parameter('min_thrust', 40.0)
        self._min_thrust = (
            self._node.get_parameter('min_thrust')
            .get_parameter_value().double_value
        )

        self._reference_pub = self._node.create_publisher(
            TrajectoryPoint, 'reference', 1
        )
        self._error_pub = self._node.create_publisher(
            TrajectoryPoint, 'error', 1
        )

        self._init_reference = False

        # Reference with relation to the INERTIAL frame
        self._reference = dict(
            pos=np.zeros(3),
            rot=np.zeros(4),
            vel=np.zeros(6),
            acc=np.zeros(6),
        )

        # Errors with relation to the BODY frame
        self._errors = dict(
            pos=np.zeros(3),
            rot=np.zeros(4),
            vel=np.zeros(6),
        )

        # Time step
        self._dt = 0
        self._prev_time = self._get_time()

        # ------------------------------------------------------------------
        # Services  (rospy.Service → node.create_service)
        # ------------------------------------------------------------------
        self._services = dict()
        self._services['reset'] = self._node.create_service(
            ResetController, 'reset_controller', self.reset_controller_callback
        )

        self._stamp_trajectory_received = self._get_time()

        self._vehicle_model = None

        if list_odometry_callbacks is not None and \
                isinstance(list_odometry_callbacks, list):
            self._odometry_callbacks = list_odometry_callbacks
        else:
            self._odometry_callbacks = [
                self.update_errors,
                self.update_controller,
            ]

        self._create_vehicle_model()
        self._init_odom = False

        # ------------------------------------------------------------------
        # Subscriber  (rospy.Subscriber → node.create_subscription)
        #   numpy_msg(Odometry) → plain Odometry (ROS2 handles numpy natively)
        # ------------------------------------------------------------------
        self._odom_topic_sub = self._node.create_subscription(
            Odometry, 'odom', self._odometry_callback, 10
        )

        self._prev_t = -1.0
        self._logger.info('DP controller successfully initialized')

    # -----------------------------------------------------------------------
    # Helper: unified time source
    # rospy.get_time() → self._get_time()
    # -----------------------------------------------------------------------
    def _get_time(self) -> float:
        """Return current ROS time as a float (seconds)."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def __del__(self):
        while self._logger.handlers:
            self._logger.handlers.pop()

    @staticmethod
    def get_controller(name, *args):
        """Create instance of a specific DP controller."""
        for controller in DPControllerBase.__subclasses__():
            if name == controller.__name__:
                return controller(*args)

    @staticmethod
    def get_list_of_controllers():
        """Return list of DP controllers using this interface."""
        return [controller.__name__ for controller in
                DPControllerBase.__subclasses__()]

    @property
    def label(self):
        """`str`: Identifier name of the controller"""
        return self._LABEL

    @property
    def odom_is_init(self):
        """`bool`: `True` if the first odometry message was received"""
        return self._init_odom

    @property
    def error_pos_world(self):
        """`numpy.array`: Position error wrt world frame"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['pos'])

    @property
    def error_orientation_quat(self):
        """`numpy.array`: Orientation error"""
        return deepcopy(self._errors['rot'][0:3])

    @property
    def error_orientation_rpy(self):
        """`numpy.array`: Orientation error in Euler angles."""
        e1 = self._errors['rot'][0]
        e2 = self._errors['rot'][1]
        e3 = self._errors['rot'][2]
        eta = self._errors['rot'][3]
        rot = np.array([
            [1 - 2 * (e2**2 + e3**2),
             2 * (e1 * e2 - e3 * eta),
             2 * (e1 * e3 + e2 * eta)],
            [2 * (e1 * e2 + e3 * eta),
             1 - 2 * (e1**2 + e3**2),
             2 * (e2 * e3 - e1 * eta)],
            [2 * (e1 * e3 - e2 * eta),
             2 * (e2 * e3 + e1 * eta),
             1 - 2 * (e1**2 + e2**2)],
        ])
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        den = np.sqrt(1 - rot[2, 1]**2)
        pitch = -np.arctan(rot[2, 1] / max(0.001, den))
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return np.array([roll, pitch, yaw])

    @property
    def error_pose_euler(self):
        """`numpy.array`: Pose error with orientation in Euler angles."""
        return np.hstack((self._errors['pos'], self.error_orientation_rpy))

    @property
    def error_vel_world(self):
        """`numpy.array`: Linear velocity error"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['vel'])

    def __str__(self):
        msg = 'Dynamic positioning controller\n'
        msg += 'Controller= ' + self._LABEL + '\n'
        msg += 'Is model based? ' + str(self._is_model_based) + '\n'
        msg += 'Vehicle namespace= ' + self._namespace
        return msg

    def _create_vehicle_model(self):
        """Create a new instance of a vehicle model."""
        if self._vehicle_model is not None:
            del self._vehicle_model
        self._vehicle_model = Vehicle(
            inertial_frame_id=self._local_planner.inertial_frame_id
        )

    def _update_reference(self):
        """Call the local planner interpolator to retrieve a trajectory
        point and publish the reference message.
        """
        self._local_planner.update_vehicle_pose(
            self._vehicle_model.pos, self._vehicle_model.quat
        )

        t = self._get_time()                     # rospy.get_time() → _get_time()
        reference = self._local_planner.interpolate(t)

        if reference is not None:
            self._reference['pos'] = reference.p
            self._reference['rot'] = reference.q
            self._reference['vel'] = np.hstack((reference.v, reference.w))
            self._reference['acc'] = np.hstack((reference.a, reference.alpha))

        # get_num_connections() → get_subscription_count()
        if reference is not None and \
                self._reference_pub.get_subscription_count() > 0:
            msg = TrajectoryPoint()
            # rospy.Time.now() → node.get_clock().now().to_msg()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._local_planner.inertial_frame_id
            msg.pose.position = Vector3(*self._reference['pos'])
            msg.pose.orientation = Quaternion(*self._reference['rot'])
            msg.velocity.linear = Vector3(*self._reference['vel'][0:3])
            msg.velocity.angular = Vector3(*self._reference['vel'][3:6])
            msg.acceleration.linear = Vector3(*self._reference['acc'][0:3])
            msg.acceleration.angular = Vector3(*self._reference['acc'][3:6])
            self._reference_pub.publish(msg)
        return True

    def _update_time_step(self):
        """Update time step."""
        t = self._get_time()                     # rospy.get_time() → _get_time()
        self._dt = t - self._prev_time
        self._prev_time = t

    def _reset_controller(self):
        """Reset reference and error vectors."""
        self._init_reference = False
        self._reference = dict(
            pos=np.zeros(3),
            rot=np.zeros(4),
            vel=np.zeros(6),
            acc=np.zeros(6),
        )
        self._errors = dict(
            pos=np.zeros(3),
            rot=np.zeros(4),
            vel=np.zeros(6),
        )

    # -----------------------------------------------------------------------
    # Service callback
    # ROS2: (request, response) → return response
    # -----------------------------------------------------------------------
    def reset_controller_callback(self, request, response):
        """Service handler function."""
        self._reset_controller()
        response.success = True
        return response

    def update_controller(self):
        """Must be overloaded by derived controller classes."""
        raise NotImplementedError()

    def update_errors(self):
        """Update error vectors."""
        if not self.odom_is_init:
            self._logger.warning('Odometry topic has not been updated yet')
            return
        self._update_reference()
        self._update_time_step()

        rotItoB = self._vehicle_model.rotItoB
        rotBtoI = self._vehicle_model.rotBtoI

        if self._dt > 0:
            pos = self._vehicle_model.pos
            vel = self._vehicle_model.vel
            quat = self._vehicle_model.quat

            self._errors['pos'] = np.dot(
                rotItoB, self._reference['pos'] - pos
            )
            self._errors['rot'] = quaternion_multiply(
                quaternion_inverse(quat), self._reference['rot']
            )
            self._errors['vel'] = np.hstack((
                np.dot(rotItoB, self._reference['vel'][0:3]) - vel[0:3],
                np.dot(rotItoB, self._reference['vel'][3:6]) - vel[3:6],
            ))

        # get_num_connections() → get_subscription_count()
        if self._error_pub.get_subscription_count() > 0:
            # rospy.Time.now() → node.get_clock().now().to_msg()
            stamp = self._node.get_clock().now().to_msg()
            msg = TrajectoryPoint()
            msg.header.stamp = stamp
            msg.header.frame_id = self._local_planner.inertial_frame_id
            msg.pose.position = Vector3(
                *np.dot(rotBtoI, self._errors['pos'])
            )
            msg.pose.orientation = Quaternion(*self._errors['rot'])
            msg.velocity.linear = Vector3(
                *np.dot(rotBtoI, self._errors['vel'][0:3])
            )
            msg.velocity.angular = Vector3(
                *np.dot(rotBtoI, self._errors['vel'][3:6])
            )
            self._error_pub.publish(msg)

    def publish_control_wrench(self, force):
        """Publish the thruster manager control set-point.

        > *Input arguments*

        * `force` (*type:* `numpy.array`): 6 DoF control set-point wrench vector
        """
        if not self.odom_is_init:
            return

        # Apply saturation
        for i in range(6):
            if force[i] < -self._control_saturation:
                force[i] = -self._control_saturation
            elif force[i] > self._control_saturation:
                force[i] = self._control_saturation

        if not self.thrusters_only:
            surge_speed = self._vehicle_model.vel[0]
            self.publish_auv_command(surge_speed, force)
            return

        force_msg = WrenchStamped()
        # rospy.Time.now() → node.get_clock().now().to_msg()
        force_msg.header.stamp = self._node.get_clock().now().to_msg()
        force_msg.header.frame_id = '%s/%s' % (
            self._namespace, self._vehicle_model.body_frame_id
        )
        force_msg.wrench.force.x = force[0]
        force_msg.wrench.force.y = force[1]
        force_msg.wrench.force.z = force[2]
        force_msg.wrench.torque.x = force[3]
        force_msg.wrench.torque.y = force[4]
        force_msg.wrench.torque.z = force[5]

        self._thrust_pub.publish(force_msg)

    def publish_auv_command(self, surge_speed, wrench):
        """Publish the AUV control command message.

        > *Input arguments*

        * `surge_speed` (*type:* `float`): Reference surge speed
        * `wrench` (*type:* `numpy.array`): 6 DoF wrench vector
        """
        if not self.odom_is_init:
            return

        surge_speed = max(0, surge_speed)

        msg = AUVCommand()
        # rospy.Time.now() → node.get_clock().now().to_msg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = '%s/%s' % (
            self._namespace, self._vehicle_model.body_frame_id
        )
        msg.surge_speed = surge_speed
        msg.command.force.x = max(self._min_thrust, wrench[0])
        msg.command.force.y = wrench[1]
        msg.command.force.z = wrench[2]
        msg.command.torque.x = wrench[3]
        msg.command.torque.y = wrench[4]
        msg.command.torque.z = wrench[5]

        self._auv_command_pub.publish(msg)

    def _odometry_callback(self, msg):
        """Odometry topic subscriber callback function.

        > *Input arguments*

        * `msg` (*type:* `nav_msgs/Odometry`): Input odometry message
        """
        # numpy_msg wrapper no longer needed — ROS2 handles numpy natively
        self._vehicle_model.update_odometry(msg)

        if not self._init_odom:
            self._init_odom = True

        if len(self._odometry_callbacks):
            for func in self._odometry_callbacks:
                func()
