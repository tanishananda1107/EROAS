# Copyright (c) 2016-2019 The UUV Simulator Authors.
# Licensed under Apache License 2.0

import numpy as np

from copy import deepcopy

from builtin_interfaces.msg import Time

from geometry_msgs.msg import (
    Vector3,
    PoseStamped,
    Quaternion
)

from nav_msgs.msg import Path

import uuv_control_msgs.msg as uuv_control_msgs

from uuv_waypoints import WaypointSet

from tf_quaternion.transformations import (
    euler_from_quaternion
)

from .wp_trajectory_generator import (
    WPTrajectoryGenerator
)

from .trajectory_point import (
    TrajectoryPoint
)

from ._log import get_logger


class TrajectoryGenerator(object):

    def __init__(
        self,
        node=None,
        full_dof=False,
        stamped_pose_only=False
    ):

        self._node = node

        self._logger = get_logger()

        self._points = None

        self._time = None

        self._this_pnt = None

        self._is_full_dof = full_dof

        self._stamped_pose_only = (
            stamped_pose_only
        )

        self._wp_interp_on = False

        self._wp_interp = (
            WPTrajectoryGenerator(
                full_dof=full_dof,
                stamped_pose_only=stamped_pose_only
            )
        )

        self._has_started = False

        self._is_finished = False

    @property
    def points(self):

        if self._wp_interp_on:
            return self._wp_interp.get_samples(
                0.001
            )

        return self._points

    @property
    def time(self):
        return self._time

    def use_finite_diff(self, flag):
        self._wp_interp.use_finite_diff = flag

    def is_using_finite_diff(self):
        return self._wp_interp.use_finite_diff

    def set_stamped_pose_only(
        self,
        flag
    ):
        self._wp_interp.stamped_pose_only = flag

    def is_using_stamped_pose_only(
        self
    ):
        return self._wp_interp.stamped_pose_only

    def set_interp_method(
        self,
        method
    ):
        return self._wp_interp.set_interpolation_method(
            method
        )

    def get_interp_method(self):

        return self._wp_interp.get_interpolation_method()

    def get_interpolator_tags(
        self
    ):
        return self._wp_interp.interpolator_tags

    def set_interpolator_parameters(
        self,
        method,
        params
    ):

        return self._wp_interp.set_interpolator_parameters(
            method,
            params
        )

    def get_visual_markers(self):

        if self._wp_interp_on:
            return self._wp_interp.get_visual_markers()

        return None

    def _reset(self):

        self._points = None

        self._time = None

        self._this_pnt = None

        self._has_started = False

        self._is_finished = False

    def get_trajectory_as_message(
        self
    ):

        if self.points is None:
            return None

        msg = uuv_control_msgs.Trajectory()

        if self._node is not None:

            msg.header.stamp = (
                self._node
                .get_clock()
                .now()
                .to_msg()
            )

        msg.header.frame_id = "world"

        for pnt in self.points:

            msg.points.append(
                pnt.to_message()
            )

        return msg

    def is_using_waypoints(self):
        return self._wp_interp_on

    def set_waypoints(
        self,
        waypoints,
        init_rot=(0,0,0,1)
    ):

        self._logger.info(
            f"Init quaternion={init_rot}"
        )

        self._logger.info(
            f"RPY={euler_from_quaternion(init_rot)}"
        )

        if self._wp_interp.init_waypoints(
            waypoints,
            init_rot
        ):

            self._wp_interp_on = True

            return True

        return False

    def get_waypoints(self):

        if not self._wp_interp_on:
            return None

        return self._wp_interp.get_waypoints()

    def add_waypoint(
        self,
        waypoint,
        add_to_beginning=False
    ):

        if not self._wp_interp_on:
            return False

        self._wp_interp.add_waypoint(
            waypoint,
            add_to_beginning
        )

        return True

    def add_trajectory_point(
        self,
        pnt
    ):

        if self._wp_interp_on:

            self._logger.error(
                "Waypoint mode active"
            )

            return False

        if self._points is None:

            self._points=[]

            self._time=[]

        if len(self._points)>1:

            if pnt.t<=self._points[-1].t:
                return False

        self._points.append(
            pnt
        )

        self._time.append(
            pnt.t
        )

        return True

    def set_duration(self,t):

        if not self._wp_interp_on:

            return False

        return self._wp_interp.set_duration(
            t
        )

    def get_max_time(self):

        if self._points is None and not self._wp_interp_on:
            return None

        if self._wp_interp_on:

            return self._wp_interp.get_max_time()

        return self._points[-1].t

    def is_running(self):

        return (
            self._has_started and
            not self._is_finished
        )

    def has_started(self):

        if self._wp_interp_on:

            return self._wp_interp.started

        return self._has_started

    def has_finished(self):

        if self._wp_interp_on:

            return self._wp_interp.is_finished()

        return self._is_finished

    def set_start_time(
        self,
        t
    ):

        if self._wp_interp_on:

            self._wp_interp.set_start_time(
                t
            )

            return True

        return False

    def generate_reference(
        self,
        t,
        *args
    ):

        if self._wp_interp_on:

            return self._wp_interp.generate_reference(
                t,
                *args
            )

        return None

    def interpolate(
        self,
        t,
        *args
    ):

        if self._wp_interp_on:

            self._this_pnt = (
                self._wp_interp.interpolate(
                    t,
                    *args
                )
            )

            return self._this_pnt

        self._this_pnt = (
            TrajectoryPoint()
        )

        if self._points is None:

            return None

        if len(self._points)==0:

            return None

        if isinstance(
            self._time,
            list
        ):

            self._time=np.array(
                self._time
            )

        self._this_pnt.t=t

        if t<=self._points[0].t:

            self._this_pnt.pos=deepcopy(
                self._points[0].pos
            )

            self._this_pnt.rotq=deepcopy(
                self._points[0].rotq
            )

            return self._this_pnt

        if t>=self._points[-1].t:

            self._this_pnt.pos=deepcopy(
                self._points[-1].pos
            )

            self._this_pnt.rotq=deepcopy(
                self._points[-1].rotq
            )

            return self._this_pnt

        idx=np.argmin(
            np.abs(
                self._time-t
            )
        )

        if idx==0:

            self._this_pnt=deepcopy(
                self._points[0]
            )

        else:

            if t<self._points[idx].t:

                p_this=self._points[idx]

                p_last=self._points[idx-1]

            else:

                p_this=self._points[idx+1]

                p_last=self._points[idx]

            dt=p_this.t-p_last.t

            w1=(t-p_last.t)/dt

            w0=(p_this.t-t)/dt

            self._this_pnt.pos=(
                w0*p_last.pos+
                w1*p_this.pos
            )

            self._this_pnt.rotq=(
                w0*p_last.rotq+
                w1*p_this.rotq
            )

            self._this_pnt.vel=(
                w0*p_last.vel+
                w1*p_this.vel
            )

            self._this_pnt.acc=(
                w0*p_last.acc+
                w1*p_this.acc
            )

        return self._this_pnt
