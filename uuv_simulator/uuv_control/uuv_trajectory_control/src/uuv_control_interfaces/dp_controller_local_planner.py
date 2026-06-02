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
#   - rospy                  → rclpy
#   - rospy.get_param()      → node.declare_parameter() / node.get_parameter()
#   - rospy.has_param()      → try/except ParameterNotDeclaredException
#   - rospy.set_param()      → node.set_parameters()
#   - rospy.Publisher()      → node.create_publisher()
#   - rospy.Subscriber()     → node.create_subscription()
#   - rospy.Service()        → node.create_service()
#   - rospy.Timer()          → node.create_timer() (no event arg in callback)
#   - rospy.get_time()       → self._get_time()  [helper using node clock]
#   - rospy.Time(s, ns)      → rclpy.time.Time(seconds=s, nanoseconds=ns)
#   - rospy.Duration(s)      → rclpy.duration.Duration(seconds=s)
#   - rospy.Time.now()       → node.get_clock().now()
#   - rospy.get_namespace()  → node.get_namespace()
#   - rospy.ROSException     → Exception
#   - Service callbacks      → (request, response) → return response
#   - Bool(val)              → Bool(data=val)
#   - Float64(val)           → Float64(data=val)
#   - header.stamp           → node.get_clock().now().to_msg()
#   - tf2_ros.TransformListener → requires node argument
# =============================================================================

import rclpy
import rclpy.duration
import rclpy.parameter
import rclpy.time
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

import logging
import sys
import time
import numpy as np
from copy import deepcopy
from os.path import isfile
from threading import Lock, Event

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from uuv_control_msgs.srv import (
    Hold,
    InitWaypointSet,
    InitCircularTrajectory,
    InitHelicalTrajectory,
    InitWaypointsFromFile,
    GoTo,
    GoToIncremental,
)
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import uuv_trajectory_generator
import uuv_waypoints
from tf_transformations import (
    quaternion_about_axis,
    quaternion_multiply,
    quaternion_inverse,
    quaternion_matrix,
    euler_from_quaternion,
    quaternion_from_euler,
)

from ._log import get_logger


class DPControllerLocalPlanner(object):
    """Local planner for the dynamic positioning controllers
    to interpolate trajectories and generate trajectories from
    interpolated waypoint paths.

    > *Input parameters*

    * `node` (*type:* `rclpy.node.Node`): The ROS2 node instance. The planner
    uses this for all ROS communication instead of the module-level rospy API.
    * `full_dof` (*type:* `bool`, *default:* `False`): If `True`,
    the reference trajectory reference will be computed for 6 DoF,
    otherwise, 4 DoF `(x, y, z, yaw)`.
    * `stamped_pose_only` (*type:* `bool`, *default:* `False`): If
    `True`, only stamped poses will be generated as a reference, with
    velocity and acceleration reference being set to zero.
    * `thrusters_only` (*type:* `bool`, *default:* `True`): If `False`,
    the idle mode will be used to keep the vehicle moving.

    > *ROS parameters*

    * `max_forward_speed` (*type:* `float`, *default:* `1.0`)
    * `idle_radius` (*type:* `float`, *default:* `10.0`)
    * `inertial_frame_id` (*type:* `str`): `world` or `world_ned`
    * `timeout_idle_mode` (*type:* `float`)
    * `look_ahead_delay` (*type:* `float`)
    """

    def __init__(self, node: Node, full_dof=True, stamped_pose_only=False, thrusters_only=True):
        # ------------------------------------------------------------------
        # Store node reference — all ROS2 API calls go through self._node
        # ------------------------------------------------------------------
        self._node = node
        self._logger = get_logger()
        self._lock = Lock()

        self._traj_interpolator = uuv_trajectory_generator.TrajectoryGenerator(
            full_dof=full_dof, stamped_pose_only=stamped_pose_only
        )

        # ------------------------------------------------------------------
        # Parameters  (declare → get pattern replaces rospy.get_param)
        # ------------------------------------------------------------------
        self._node.declare_parameter('max_forward_speed', 1.0)
        self._max_forward_speed = (
            self._node.get_parameter('max_forward_speed').get_parameter_value().double_value
        )

        self._idle_circle_center = None
        self._idle_z = None
        self._logger.info('Max. forward speed [m/s]=%.2f' % self._max_forward_speed)

        self._node.declare_parameter('idle_radius', 10.0)
        self._idle_radius = (
            self._node.get_parameter('idle_radius').get_parameter_value().double_value
        )
        assert self._idle_radius > 0
        self._logger.info('Idle circle radius [m] = %.2f' % self._idle_radius)

        self._node.declare_parameter('is_underactuated', False)
        self._is_underactuated = (
            self._node.get_parameter('is_underactuated').get_parameter_value().bool_value
        )

        # ------------------------------------------------------------------
        # Inertial frame ID
        # rospy.has_param / rospy.get_param → declare with sentinel default
        # ------------------------------------------------------------------
        self.inertial_frame_id = 'world'
        self.transform_ned_to_enu = None
        self.q_ned_to_enu = None

        self._node.declare_parameter('inertial_frame_id', '')
        _frame_param = (
            self._node.get_parameter('inertial_frame_id').get_parameter_value().string_value
        )
        if _frame_param:                          # non-empty → use it
            self.inertial_frame_id = _frame_param
            assert len(self.inertial_frame_id) > 0
            assert self.inertial_frame_id in ['world', 'world_ned']

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)

        # rospy.set_param → node.set_parameters
        self._node.set_parameters([
            rclpy.parameter.Parameter(
                'inertial_frame_id',
                rclpy.parameter.Parameter.Type.STRING,
                self.inertial_frame_id,
            )
        ])

        # ------------------------------------------------------------------
        # TF2 lookup  (TransformListener now requires a node argument)
        # ------------------------------------------------------------------
        try:
            import tf2_ros

            tf_buffer = tf2_ros.Buffer()
            # ROS2: TransformListener requires the node
            listener = tf2_ros.TransformListener(tf_buffer, self._node)

            tf_trans_ned_to_enu = tf_buffer.lookup_transform(
                'world',
                'world_ned',
                rclpy.time.Time(),                          # rospy.Time() → rclpy.time.Time()
                rclpy.duration.Duration(seconds=10),        # rospy.Duration(10) → Duration(seconds=10)
            )

            self.q_ned_to_enu = np.array([
                tf_trans_ned_to_enu.transform.rotation.x,
                tf_trans_ned_to_enu.transform.rotation.y,
                tf_trans_ned_to_enu.transform.rotation.z,
                tf_trans_ned_to_enu.transform.rotation.w,
            ])
        except Exception as ex:
            self._logger.warning(
                'Error while requesting ENU to NED transform, message={}'.format(ex)
            )
            self.q_ned_to_enu = quaternion_from_euler(2 * np.pi, 0, np.pi)

        self.transform_ned_to_enu = quaternion_matrix(self.q_ned_to_enu)[0:3, 0:3]

        if self.transform_ned_to_enu is not None:
            self._logger.info(
                'Transform world_ned (NED) to world (ENU)=\n' + str(self.transform_ned_to_enu)
            )

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)
        self._logger.info('Max. forward speed = ' + str(self._max_forward_speed))

        for method in self._traj_interpolator.get_interpolator_tags():
            # rospy.has_param / rospy.get_param → declare + get pattern
            try:
                self._node.declare_parameter(method, rclpy.parameter.Parameter.Type.NOT_SET)
            except Exception:
                pass  # already declared

            try:
                params = (
                    self._node.get_parameter(method).get_parameter_value()
                )
                # Only use if it was actually set (not NOT_SET)
                self._logger.info(
                    'Parameters for interpolation method <%s> found' % method
                )
                self._logger.info('\t' + str(params))
                self._traj_interpolator.set_interpolator_parameters(method, params)
            except ParameterNotDeclaredException:
                self._logger.info(
                    'No parameters for interpolation method <%s> found' % method
                )

        # Teleop state
        self._dt = 0.0
        self._last_teleop_update = None
        self._is_teleop_active = False
        self._teleop_vel_ref = None

        self.init_odom_event = Event()
        self.init_odom_event.clear()

        self._node.declare_parameter('timeout_idle_mode', 5.0)
        self._timeout_idle_mode = (
            self._node.get_parameter('timeout_idle_mode').get_parameter_value().double_value
        )
        self._start_count_idle = self._get_time()

        self._thrusters_only = thrusters_only

        if not self._thrusters_only:
            self._node.declare_parameter('look_ahead_delay', 3.0)
            self._look_ahead_delay = (
                self._node.get_parameter('look_ahead_delay').get_parameter_value().double_value
            )
        else:
            self._look_ahead_delay = 0.0

        self._station_keeping_center = None

        # ------------------------------------------------------------------
        # Publishers  (rospy.Publisher → node.create_publisher)
        #   Signature: create_publisher(MsgType, topic, qos_depth)
        # ------------------------------------------------------------------
        self._trajectory_pub = self._node.create_publisher(Trajectory, 'trajectory', 1)
        self._waypoints_pub = self._node.create_publisher(WaypointSet, 'waypoints', 1)
        self._station_keeping_pub = self._node.create_publisher(Bool, 'station_keeping_on', 1)
        self._automatic_control_pub = self._node.create_publisher(Bool, 'automatic_on', 1)
        self._traj_tracking_pub = self._node.create_publisher(Bool, 'trajectory_tracking_on', 1)
        self._interp_visual_markers = self._node.create_publisher(
            MarkerArray, 'interpolator_visual_markers', 1
        )
        self._max_time_pub = self._node.create_publisher(Float64, 'time_to_target', 1)

        # ------------------------------------------------------------------
        # Subscribers  (rospy.Subscriber → node.create_subscription)
        #   Signature: create_subscription(MsgType, topic, callback, qos_depth)
        # ------------------------------------------------------------------
        self._teleop_sub = self._node.create_subscription(
            Twist, 'cmd_vel', self._update_teleop, 10
        )
        self._input_trajectory_sub = self._node.create_subscription(
            Trajectory, 'input_trajectory', self._update_trajectory_from_msg, 10
        )

        self._waypoints_msg = None
        self._trajectory_msg = None

        # ------------------------------------------------------------------
        # Timer  (rospy.Timer → node.create_timer)
        #   Callback must NOT accept an event argument in ROS2
        # ------------------------------------------------------------------
        self._traj_info_update_timer = self._node.create_timer(
            0.2, self._publish_trajectory_info   # period in seconds
        )

        # State flags
        self._station_keeping_on = True
        self._is_automatic = True
        self._traj_running = False
        self._vehicle_pose = None
        self._this_ref_pnt = None
        self._smooth_approach_on = False
        self._stamp_trajectory_received = 0.0

        # ------------------------------------------------------------------
        # Services  (rospy.Service → node.create_service)
        #   Signature: create_service(SrvType, name, callback)
        #   Callbacks: (request, response) → return response
        # ------------------------------------------------------------------
        self._controller_services = dict()
        self._controller_services['hold_vehicle'] = self._node.create_service(
            Hold, 'hold_vehicle', self.hold_vehicle
        )
        self._controller_services['start_waypoint_list'] = self._node.create_service(
            InitWaypointSet, 'start_waypoint_list', self.start_waypoint_list
        )
        self._controller_services['start_circular_trajectory'] = self._node.create_service(
            InitCircularTrajectory, 'start_circular_trajectory', self.start_circle
        )
        self._controller_services['start_helical_trajectory'] = self._node.create_service(
            InitHelicalTrajectory, 'start_helical_trajectory', self.start_helix
        )
        self._controller_services['init_waypoints_from_file'] = self._node.create_service(
            InitWaypointsFromFile, 'init_waypoints_from_file', self.init_waypoints_from_file
        )
        self._controller_services['go_to'] = self._node.create_service(
            GoTo, 'go_to', self.go_to
        )
        self._controller_services['go_to_incremental'] = self._node.create_service(
            GoToIncremental, 'go_to_incremental', self.go_to_incremental
        )

    # -----------------------------------------------------------------------
    # Helper: unified time source
    # rospy.get_time() → self._get_time()
    # -----------------------------------------------------------------------
    def _get_time(self) -> float:
        """Return current ROS time as a float (seconds)."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def __del__(self):
        """Remove logging message handlers."""
        while self._logger.handlers:
            self._logger.handlers.pop()

    # -----------------------------------------------------------------------
    # Frame transforms (unchanged logic)
    # -----------------------------------------------------------------------
    def _transform_position(self, vec, target, source):
        if target == source:
            return vec
        if target == 'world':
            return np.dot(self.transform_ned_to_enu, vec)
        if target == 'world_ned':
            return np.dot(self.transform_ned_to_enu.T, vec)

    def _transform_waypoint(self, waypoint):
        output = deepcopy(waypoint)
        output.pos = self._transform_position(
            output.pos, self.inertial_frame_id, output.inertial_frame_id
        )
        output.inertial_frame_id = self.inertial_frame_id
        output.max_forward_speed = min(waypoint.max_forward_speed, self._max_forward_speed)
        return output

    def _transform_waypoint_set(self, waypoint_set):
        output = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = self._transform_waypoint(waypoint_set.get_waypoint(i))
            output.add_waypoint(wp)
        return output

    def _apply_workspace_constraints(self, waypoint_set):
        wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = waypoint_set.get_waypoint(i)
            if wp.z > 0 and self.inertial_frame_id == 'world':
                continue
            if wp.z < 0 and self.inertial_frame_id == 'world_ned':
                continue
            wp_set.add_waypoint(wp)
        return wp_set

    # -----------------------------------------------------------------------
    # Timer callback
    # ROS2: no `event` parameter — remove it from the signature
    # -----------------------------------------------------------------------
    def _publish_trajectory_info(self):          # was: def ...(self, event)
        if self._waypoints_msg is not None:
            self._waypoints_pub.publish(self._waypoints_msg)
        if self._trajectory_msg is not None:
            self._trajectory_pub.publish(self._trajectory_msg)
        markers = self._traj_interpolator.get_visual_markers()
        if markers is not None:
            self._interp_visual_markers.publish(markers)
        else:
            self._interp_visual_markers.publish(MarkerArray())
        # Bool(val) → Bool(data=val)
        self._station_keeping_pub.publish(Bool(data=self._station_keeping_on))
        self._automatic_control_pub.publish(Bool(data=self._is_automatic))
        self._traj_tracking_pub.publish(Bool(data=self._traj_running))
        return True

    def _update_trajectory_info(self):
        self._waypoints_msg = WaypointSet()
        if self._traj_interpolator.is_using_waypoints():
            wps = self._traj_interpolator.get_waypoints()
            if wps is not None:
                wps.inertial_frame_id = self.inertial_frame_id
                self._waypoints_msg = wps.to_message()
                self._waypoints_msg.header.frame_id = self.inertial_frame_id
        msg = self._traj_interpolator.get_trajectory_as_message()
        if msg is not None:
            msg.header.frame_id = self.inertial_frame_id
            self._trajectory_msg = msg
            self._logger.info('Updating the trajectory information')
        else:
            self._trajectory_msg = None
            self._logger.error('Error generating trajectory message')

    # -----------------------------------------------------------------------
    # Subscriber callbacks
    # -----------------------------------------------------------------------
    def _update_teleop(self, msg):
        if self._is_automatic:
            self._teleop_vel_ref = None
            return

        if self._last_teleop_update is None:
            self._teleop_vel_ref = None
            self._last_teleop_update = self._get_time()   # rospy.get_time() → _get_time()
            return

        self._teleop_vel_ref = msg
        vel = np.array([
            self._teleop_vel_ref.linear.x,
            self._teleop_vel_ref.linear.y,
            self._teleop_vel_ref.linear.z,
            self._teleop_vel_ref.angular.z,
        ])
        self._is_teleop_active = np.abs(vel).sum() > 0
        self._last_teleop_update = self._get_time()       # rospy.get_time() → _get_time()

    def _calc_teleop_reference(self):
        if self._last_teleop_update is None:
            self._is_teleop_active = False

        self._dt = self._get_time() - self._last_teleop_update  # rospy.get_time() → _get_time()

        if self._dt > 0 and self._teleop_vel_ref is not None and self._dt < 0.1:
            speed = np.sqrt(
                self._teleop_vel_ref.linear.x ** 2 + self._teleop_vel_ref.linear.y ** 2
            )
            vel = np.array([
                self._teleop_vel_ref.linear.x,
                self._teleop_vel_ref.linear.y,
                self._teleop_vel_ref.linear.z,
            ])
            if speed > self._max_forward_speed:
                vel[0] *= self._max_forward_speed / speed
                vel[1] *= self._max_forward_speed / speed

            vel = np.dot(self._vehicle_pose.rot_matrix, vel)

            step = uuv_trajectory_generator.TrajectoryPoint()
            step.pos = np.dot(self._vehicle_pose.rot_matrix, vel * self._dt)
            step.rotq = quaternion_about_axis(
                self._teleop_vel_ref.angular.z * self._dt, [0, 0, 1]
            )

            ref_pnt = uuv_trajectory_generator.TrajectoryPoint()
            ref_pnt.pos = self._vehicle_pose.pos + step.pos
            ref_pnt.rotq = quaternion_multiply(self.get_vehicle_rot(), step.rotq)

            if ref_pnt.z > 0:
                ref_pnt.z = 0.0
                ref_pnt.vel = [vel[0], vel[1], 0, 0, 0, self._teleop_vel_ref.angular.z]
            else:
                ref_pnt.vel = [vel[0], vel[1], vel[2], 0, 0, self._teleop_vel_ref.angular.z]

            ref_pnt.acc = np.zeros(6)
        else:
            self._is_teleop_active = False
            ref_pnt = deepcopy(self._vehicle_pose)
        return ref_pnt

    def _calc_smooth_approach(self):
        if self._vehicle_pose is None:
            self._logger.error(
                'Simulation not properly initialized yet, ignoring approach...'
            )
            return
        if not self._traj_interpolator.is_using_waypoints():
            self._logger.error('Not using the waypoint interpolation method')
            return

        heading = euler_from_quaternion(self.get_vehicle_rot())[2]

        if self._thrusters_only:
            init_wp = uuv_waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],
                y=self._vehicle_pose.pos[1],
                z=self._vehicle_pose.pos[2],
                max_forward_speed=self._traj_interpolator.get_waypoints()
                    .get_waypoint(0).max_forward_speed,
                heading_offset=self._traj_interpolator.get_waypoints()
                    .get_waypoint(0).heading_offset,
            )
        else:
            max_speed = (
                self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed
            )
            init_wp = uuv_waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],
                y=self._vehicle_pose.pos[1],
                z=self._vehicle_pose.pos[2],
                max_forward_speed=max_speed,
                heading_offset=self._traj_interpolator.get_waypoints()
                    .get_waypoint(0).heading_offset,
            )
        first_wp = self._traj_interpolator.get_waypoints().get_waypoint(0)

        dx = first_wp.x - init_wp.x
        dy = first_wp.y - init_wp.y
        dz = first_wp.z - init_wp.z

        self._logger.info(
            'Adding waypoints to approach the first position in the given waypoint set'
        )
        steps = int(np.floor(first_wp.dist(init_wp.pos)) / 10)
        if steps > 0 and self._traj_interpolator.get_interp_method() != 'dubins':
            for i in range(1, steps):
                wp = uuv_waypoints.Waypoint(
                    x=first_wp.x - i * dx / steps,
                    y=first_wp.y - i * dy / steps,
                    z=first_wp.z - i * dz / steps,
                    max_forward_speed=self._traj_interpolator.get_waypoints()
                        .get_waypoint(0).max_forward_speed,
                )
                self._traj_interpolator.add_waypoint(wp, add_to_beginning=True)
        self._traj_interpolator.add_waypoint(init_wp, add_to_beginning=True)
        self._update_trajectory_info()

    # -----------------------------------------------------------------------
    # Public state helpers (unchanged)
    # -----------------------------------------------------------------------
    def is_station_keeping_on(self):
        return self._station_keeping_on

    def is_automatic_on(self):
        return self._is_automatic

    def set_station_keeping(self, is_on=True):
        self._station_keeping_on = is_on
        self._logger.info('STATION KEEPING MODE = ' + ('ON' if is_on else 'OFF'))

    def set_automatic_mode(self, is_on=True):
        self._is_automatic = is_on
        self._logger.info('AUTOMATIC MODE = ' + ('ON' if is_on else 'OFF'))

    def set_trajectory_running(self, is_on=True):
        self._traj_running = is_on
        self._logger.info('TRAJECTORY TRACKING = ' + ('ON' if is_on else 'OFF'))

    def has_started(self):
        return self._traj_interpolator.has_started()

    def has_finished(self):
        return self._traj_interpolator.has_finished()

    def update_vehicle_pose(self, pos, quat):
        if self._vehicle_pose is None:
            self._vehicle_pose = uuv_trajectory_generator.TrajectoryPoint()
        self._vehicle_pose.pos = pos
        self._vehicle_pose.rotq = quat
        self._vehicle_pose.t = self._get_time()   # rospy.get_time() → _get_time()
        self.init_odom_event.set()

    def get_vehicle_rot(self):
        self.init_odom_event.wait()
        return self._vehicle_pose.rotq

    def _update_trajectory_from_msg(self, msg):
        self._stamp_trajectory_received = self._get_time()  # rospy.get_time() → _get_time()
        self._traj_interpolator.init_from_trajectory_message(msg)
        self._logger.info(
            'New trajectory received at ' + str(self._stamp_trajectory_received) + 's'
        )
        self._update_trajectory_info()

    def start_station_keeping(self):
        if self._vehicle_pose is not None:
            self._this_ref_pnt = deepcopy(self._vehicle_pose)
            self._this_ref_pnt.vel = np.zeros(6)
            self._this_ref_pnt.acc = np.zeros(6)
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self._smooth_approach_on = False

    # -----------------------------------------------------------------------
    # Service callbacks
    # ROS2: signature is (request, response) → return response
    #       instead of (request) → return ResponseType(...)
    # -----------------------------------------------------------------------
    def hold_vehicle(self, request, response):   # was: def hold_vehicle(self, request)
        if self._vehicle_pose is None:
            self._logger.error('Current pose of the vehicle is invalid')
            response.success = False
            return response
        self.start_station_keeping()
        response.success = True
        return response

    def start_waypoint_list(self, request, response):
        if len(request.waypoints) == 0:
            self._logger.error('Waypoint list is empty')
            response.success = False
            return response

        # rospy.Time(secs, nsecs) → rclpy.time.Time(seconds=..., nanoseconds=...)
        t = rclpy.time.Time(
            seconds=request.start_time.data.secs,
            nanoseconds=request.start_time.data.nsecs,
        )
        t_sec = t.nanoseconds * 1e-9

        if t_sec < self._get_time() and not request.start_now:
            self._logger.error(
                'The trajectory starts in the past, correct the starting time!'
            )
            response.success = False
            return response
        else:
            self._logger.info('Start waypoint trajectory now!')

        self._lock.acquire()
        wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        waypointset_msg = WaypointSet()
        # header.stamp → node clock
        waypointset_msg.header.stamp = self._node.get_clock().now().to_msg()
        waypointset_msg.header.frame_id = self.inertial_frame_id
        if request.start_now:
            waypointset_msg.start_time = self._get_time()
        else:
            waypointset_msg.start_time = t_sec
        waypointset_msg.waypoints = request.waypoints
        wp_set.from_message(waypointset_msg)
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time(
                t_sec if not request.start_now else self._get_time()
            )
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True
            self._logger.info('============================')
            self._logger.info('      WAYPOINT SET          ')
            self._logger.info('============================')
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info(
                '# waypoints = %d'
                % self._traj_interpolator.get_waypoints().num_waypoints
            )
            self._logger.info(
                'Starting time = %.2f'
                % (t_sec if not request.start_now else self._get_time())
            )
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            response.success = True
            return response
        else:
            self._logger.error('Error occurred while parsing waypoints')
            self._lock.release()
            response.success = False
            return response

    def start_circle(self, request, response):
        if (
            request.max_forward_speed <= 0
            or request.radius <= 0
            or request.n_points <= 0
        ):
            self._logger.error(
                'Invalid parameters to generate a circular trajectory'
            )
            response.success = False
            return response

        t = rclpy.time.Time(
            seconds=request.start_time.data.secs,
            nanoseconds=request.start_time.data.nsecs,
        )
        t_sec = t.nanoseconds * 1e-9

        if t_sec < self._get_time() and not request.start_now:
            self._logger.error(
                'The trajectory starts in the past, correct the starting time!'
            )
            response.success = False
            return response

        try:
            wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
            success = wp_set.generate_circle(
                radius=request.radius,
                center=request.center,
                num_points=request.n_points,
                max_forward_speed=request.max_forward_speed,
                theta_offset=request.angle_offset,
                heading_offset=request.heading_offset,
            )
            if not success:
                self._logger.error(
                    'Error generating circular trajectory from waypoint set'
                )
                response.success = False
                return response
            wp_set = self._apply_workspace_constraints(wp_set)
            if wp_set.is_empty:
                self._logger.error(
                    'Waypoints violate workspace constraints, '
                    'are you using world or world_ned as reference?'
                )
                response.success = False
                return response

            self._lock.acquire()
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic')
            self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot())
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time(
                t_sec if not request.start_now else self._get_time()
            )
            if request.duration > 0:
                if self._traj_interpolator.set_duration(request.duration):
                    self._logger.info(
                        'Setting a maximum duration, duration=%.2f s' % request.duration
                    )
                else:
                    self._logger.error('Setting maximum duration failed')
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('CIRCULAR TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION')
            self._logger.info('============================')
            self._logger.info('Radius [m] = %.2f' % request.radius)
            self._logger.info(
                'Center [m] = (%.2f, %.2f, %.2f)'
                % (request.center.x, request.center.y, request.center.z)
            )
            self._logger.info('# of points = %d' % request.n_points)
            self._logger.info('Max. forward speed = %.2f' % request.max_forward_speed)
            self._logger.info('Circle angle offset = %.2f' % request.angle_offset)
            self._logger.info('Heading offset = %.2f' % request.heading_offset)
            self._logger.info(
                '# waypoints = %d'
                % self._traj_interpolator.get_waypoints().num_waypoints
            )
            self._logger.info(
                'Starting from = '
                + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos)
            )
            self._logger.info(
                'Starting time [s] = %.2f'
                % (t_sec if not request.start_now else self._get_time())
            )
            self._logger.info('============================')
            self._lock.release()
            response.success = True
            return response
        except Exception as e:
            self._logger.error(
                'Error while setting circular trajectory, msg={}'.format(e)
            )
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self.set_trajectory_running(False)
            self._lock.release()
            response.success = False
            return response

    def start_helix(self, request, response):
        if request.radius <= 0 or request.n_points <= 0 or request.n_turns <= 0:
            self._logger.error(
                'Invalid parameters to generate a helical trajectory'
            )
            response.success = False
            return response

        t = rclpy.time.Time(
            seconds=request.start_time.data.secs,
            nanoseconds=request.start_time.data.nsecs,
        )
        t_sec = t.nanoseconds * 1e-9

        if t_sec < self._get_time() and not request.start_now:
            self._logger.error(
                'The trajectory starts in the past, correct the starting time!'
            )
            response.success = False
            return response
        else:
            self._logger.info('Start helical trajectory now!')

        try:
            wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
            success = wp_set.generate_helix(
                radius=request.radius,
                center=request.center,
                num_points=request.n_points,
                max_forward_speed=request.max_forward_speed,
                delta_z=request.delta_z,
                num_turns=request.n_turns,
                theta_offset=request.angle_offset,
                heading_offset=request.heading_offset,
            )
            if not success:
                self._logger.error(
                    'Error generating circular trajectory from waypoint set'
                )
                response.success = False
                return response
            wp_set = self._apply_workspace_constraints(wp_set)
            if wp_set.is_empty:
                self._logger.error(
                    'Waypoints violate workspace constraints, '
                    'are you using world or world_ned as reference?'
                )
                response.success = False
                return response

            self._lock.acquire()
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic')
            if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
                self._logger.error('Error setting the waypoints')
                response.success = False
                return response

            self._station_keeping_center = None
            self._traj_interpolator.set_start_time(
                t_sec if not request.start_now else self._get_time()
            )
            if request.duration > 0:
                if self._traj_interpolator.set_duration(request.duration):
                    self._logger.info(
                        'Setting a maximum duration, duration=%.2f s' % request.duration
                    )
                else:
                    self._logger.error('Setting maximum duration failed')
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('HELICAL TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION')
            self._logger.info('============================')
            self._logger.info('Radius [m] = %.2f' % request.radius)
            self._logger.info(
                'Center [m] = (%.2f, %.2f, %.2f)'
                % (request.center.x, request.center.y, request.center.z)
            )
            self._logger.info('# of points = %d' % request.n_points)
            self._logger.info('Max. forward speed = %.2f' % request.max_forward_speed)
            self._logger.info('Delta Z = %.2f' % request.delta_z)
            self._logger.info('# of turns = %d' % request.n_turns)
            self._logger.info('Helix angle offset = %.2f' % request.angle_offset)
            self._logger.info('Heading offset = %.2f' % request.heading_offset)
            self._logger.info(
                '# waypoints = %d'
                % self._traj_interpolator.get_waypoints().num_waypoints
            )
            self._logger.info(
                'Starting from = '
                + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos)
            )
            self._logger.info(
                'Starting time [s] = %.2f'
                % (t_sec if not request.start_now else self._get_time())
            )
            self._logger.info('============================')
            self._lock.release()
            response.success = True
            return response
        except Exception as e:
            self._logger.error(
                'Error while setting helical trajectory, msg={}'.format(e)
            )
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self.set_trajectory_running(False)
            self._lock.release()
            response.success = False
            return response

    def init_waypoints_from_file(self, request, response):
        if len(request.filename.data) == 0 or not isfile(request.filename.data):
            self._logger.error('Invalid waypoint file')
            response.success = False
            return response

        t = rclpy.time.Time(
            seconds=request.start_time.data.secs,
            nanoseconds=request.start_time.data.nsecs,
        )
        t_sec = t.nanoseconds * 1e-9

        if t_sec < self._get_time() and not request.start_now:
            self._logger.error(
                'The trajectory starts in the past, correct the starting time!'
            )
            response.success = False
            return response
        else:
            self._logger.info('Start waypoint trajectory now!')

        self._lock.acquire()
        self.set_station_keeping(True)
        self._traj_interpolator.set_interp_method(request.interpolator.data)

        wp_set = uuv_waypoints.WaypointSet()
        if not wp_set.read_from_file(request.filename.data):
            self._logger.info('Error occurred while parsing waypoint file')
            response.success = False
            return response
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time(
                t_sec if not request.start_now else self._get_time()
            )
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('IMPORT WAYPOINTS FROM FILE')
            self._logger.info('============================')
            self._logger.info('Filename = ' + request.filename.data)
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info(
                '# waypoints = %d'
                % self._traj_interpolator.get_waypoints().num_waypoints
            )
            self._logger.info(
                'Starting time = %.2f'
                % (t_sec if not request.start_now else self._get_time())
            )
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            response.success = True
            return response
        else:
            self._logger.error('Error occurred while parsing waypoint file')
            self._lock.release()
            response.success = False
            return response

    def go_to(self, request, response):
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            response.success = False
            return response
        if request.waypoint.max_forward_speed <= 0.0:
            self._logger.error('Max. forward speed must be greater than zero')
            response.success = False
            return response
        self.set_station_keeping(True)
        self._lock.acquire()
        wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        init_wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.waypoint.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            use_fixed_heading=request.waypoint.use_fixed_heading,
            inertial_frame_id=self.inertial_frame_id,
        )
        wp_set.add_waypoint(init_wp)
        wp_set.add_waypoint_from_msg(request.waypoint)
        wp_set = self._transform_waypoint_set(wp_set)
        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            response.success = False
            return response

        self._station_keeping_center = None
        t = self._get_time()
        self._traj_interpolator.set_start_time(t)
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_circle_center = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO')
        self._logger.info('============================')
        self._logger.info(
            'Heading offset [rad] = %.2f' % request.waypoint.heading_offset
        )
        self._logger.info(
            '# waypoints = %d'
            % self._traj_interpolator.get_waypoints().num_waypoints
        )
        self._logger.info(
            'Starting from = '
            + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos)
        )
        self._logger.info('Start time [s] = %.2f ' % t)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        response.success = True
        return response

    def go_to_incremental(self, request, response):
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            response.success = False
            return response
        if request.max_forward_speed <= 0:
            self._logger.error('Max. forward speed must be positive')
            response.success = False
            return response

        self._lock.acquire()
        self.set_station_keeping(True)
        wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        init_wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            inertial_frame_id=self.inertial_frame_id,
        )
        wp_set.add_waypoint(init_wp)

        wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0] + request.step.x,
            y=self._vehicle_pose.pos[1] + request.step.y,
            z=self._vehicle_pose.pos[2] + request.step.z,
            max_forward_speed=request.max_forward_speed,
            inertial_frame_id=self.inertial_frame_id,
        )
        wp_set.add_waypoint(wp)

        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            response.success = False
            return response

        self._station_keeping_center = None
        # rospy.Time.now().to_sec() → self._get_time()
        self._traj_interpolator.set_start_time(self._get_time())
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_circle_center = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO INCREMENTAL')
        self._logger.info('============================')
        self._logger.info(str(wp_set))
        self._logger.info('# waypoints = %d' % wp_set.num_waypoints)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        response.success = True
        return response

    # -----------------------------------------------------------------------
    # Reference generation helpers (unchanged logic)
    # -----------------------------------------------------------------------
    def generate_reference(self, t):
        pnt = self._traj_interpolator.generate_reference(
            t, self._vehicle_pose.pos, self.get_vehicle_rot()
        )
        if pnt is None:
            return self._vehicle_pose
        return pnt

    def get_idle_circle_path(self, n_points, radius=30):
        pose = deepcopy(self._vehicle_pose)
        if self._idle_circle_center is None:
            frame = np.array([
                [np.cos(pose.rot[2]), -np.sin(pose.rot[2]), 0],
                [np.sin(pose.rot[2]),  np.cos(pose.rot[2]), 0],
                [0, 0, 1],
            ])
            self._idle_circle_center = (
                pose.pos + 0.8 * self._max_forward_speed * frame[:, 0].flatten()
            ) + radius * frame[:, 1].flatten()
            self._idle_z = pose.pos[2]

        phi = lambda u: 2 * np.pi * u + pose.rot[2] - np.pi / 2
        u = lambda angle: (angle - pose.rot[2] + np.pi / 2) / (2 * np.pi)

        vec = pose.pos - self._idle_circle_center
        vec /= np.linalg.norm(vec)
        u_init = u(np.arctan2(vec[1], vec[0]))

        wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
        for i in np.linspace(u_init, u_init + 1, n_points):
            wp = uuv_waypoints.Waypoint(
                x=self._idle_circle_center[0] + radius * np.cos(phi(i)),
                y=self._idle_circle_center[1] + radius * np.sin(phi(i)),
                z=self._idle_z,
                max_forward_speed=0.8 * self._max_forward_speed,
                inertial_frame_id=self.inertial_frame_id,
            )
            wp_set.add_waypoint(wp)
        return wp_set

    def interpolate(self, t):
        self._lock.acquire()
        if not self._station_keeping_on and self._traj_running:
            if self._smooth_approach_on:
                self._calc_smooth_approach()
                self._smooth_approach_on = False
                self._update_trajectory_info()
                time.sleep(0.5)
                self._logger.info(
                    'Adding waypoints to approach the given waypoint trajectory'
                )

            self._this_ref_pnt = self._traj_interpolator.interpolate(
                t, self._vehicle_pose.pos, self.get_vehicle_rot()
            )

            if self._look_ahead_delay > 0:
                self._this_ref_pnt = self.generate_reference(t + self._look_ahead_delay)

            # Float64(val) → Float64(data=val)
            self._max_time_pub.publish(
                Float64(data=self._traj_interpolator.get_max_time() - self._get_time())
            )

            if not self._traj_running:
                self._traj_running = True
                # rospy.get_namespace() → node.get_namespace()
                self._logger.info(
                    self._node.get_namespace() + ' - Trajectory running'
                )

            if self._traj_running and (
                self._traj_interpolator.has_finished() or self._station_keeping_on
            ):
                self._logger.info(
                    self._node.get_namespace() + ' - Trajectory completed!'
                )
                if self._this_ref_pnt is None:
                    if self._is_teleop_active:
                        self._this_ref_pnt = self._calc_teleop_reference()
                    else:
                        self._this_ref_pnt = deepcopy(self._vehicle_pose)
                self._this_ref_pnt.vel = np.zeros(6)
                self._this_ref_pnt.acc = np.zeros(6)
                self._start_count_idle = self._get_time()
                self.set_station_keeping(True)
                self.set_automatic_mode(False)
                self.set_trajectory_running(False)

        elif self._this_ref_pnt is None:
            self._traj_interpolator.set_interp_method('lipb')
            if self._is_teleop_active:
                self._this_ref_pnt = self._calc_teleop_reference()
            else:
                self._this_ref_pnt = deepcopy(self._vehicle_pose)
            yaw = self._this_ref_pnt.rot[2]
            self._this_ref_pnt.rot = [0, 0, yaw]
            self.set_automatic_mode(False)

        elif self._station_keeping_on:
            if self._is_teleop_active:
                self._this_ref_pnt = self._calc_teleop_reference()
            self._max_time_pub.publish(Float64(data=0.0))

            if (
                not self._thrusters_only
                and not self._is_teleop_active
                and self._get_time() - self._start_count_idle > self._timeout_idle_mode
            ):
                self._logger.info('AUV STATION KEEPING')
                if self._station_keeping_center is None:
                    self._station_keeping_center = self._this_ref_pnt

                wp_set = self.get_idle_circle_path(20, self._idle_radius)
                wp_set = self._apply_workspace_constraints(wp_set)
                if wp_set.is_empty:
                    # rospy.ROSException → plain Exception
                    raise Exception(
                        'Waypoints violate workspace constraints, '
                        'are you using world or world_ned as reference?'
                    )

                self.set_station_keeping(True)
                self._traj_interpolator.set_interp_method('cubic')
                self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot())
                self._traj_interpolator.set_start_time(self._get_time())
                self._update_trajectory_info()
                self.set_station_keeping(False)
                self.set_automatic_mode(True)
                self.set_trajectory_running(True)
                self._smooth_approach_on = False

        self._lock.release()
        return self._this_ref_pnt
