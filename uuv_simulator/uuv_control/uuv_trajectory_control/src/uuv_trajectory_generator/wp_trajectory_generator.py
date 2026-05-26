# Copyright (c) 2016-2019 The UUV Simulator Authors.
# Licensed under Apache License 2.0

import rclpy
import numpy as np

from copy import deepcopy

import logging
import sys
import time

from .trajectory_point import (
    TrajectoryPoint
)

from uuv_waypoints import (
    Waypoint,
    WaypointSet
)

from tf_quaternion.transformations import (
    quaternion_multiply,
    quaternion_inverse,
    quaternion_conjugate,
    quaternion_about_axis
)

from .path_generator import (
    PathGenerator
)


class WPTrajectoryGenerator(object):

    def __init__(
        self,
        full_dof=False,
        use_finite_diff=True,
        interpolation_method="cubic",
        stamped_pose_only=False
    ):

        self._logger = logging.getLogger(
            "wp_trajectory_generator"
        )

        handler = logging.StreamHandler(
            sys.stdout
        )

        handler.setFormatter(
            logging.Formatter(
                "%(asctime)s | %(levelname)s | %(module)s | %(message)s"
            )
        )

        self._logger.addHandler(
            handler
        )

        self._logger.setLevel(
            logging.INFO
        )

        self._path_generators = {}

        self._logger.info(
            "Available interpolators:"
        )

        for gen in PathGenerator.get_all_generators():

            self._logger.info(
                gen.get_label()
            )

            self._path_generators[
                gen.get_label()
            ] = gen

            self._path_generators[
                gen.get_label()
            ].set_full_dof(
                full_dof
            )

        self._dt = None

        self._last_t = None

        self._last_pnt = None

        self._this_pnt = None

        self._stamped_pose_only = (
            stamped_pose_only
        )

        self._t_step = 0.001

        self._interp_method = (
            interpolation_method
        )

        self._is_full_dof = (
            full_dof
        )

        self._use_finite_diff = (
            use_finite_diff
        )

        self._regression_window = 0.5

        if not self._use_finite_diff:

            self._t_step = (
                self._regression_window / 30
            )

        self._has_started = False

        self._has_ended = False

        self._cur_s = 0.0

        self._init_rot = (
            quaternion_about_axis(
                0.0,
                [0,0,1]
            )
        )

    def __del__(self):

        while self._logger.handlers:

            self._logger.handlers.pop()

    @property
    def started(self):

        return self._has_started

    @property
    def closest_waypoint(self):

        return self.interpolator.closest_waypoint

    @property
    def closest_waypoint_idx(self):

        return self.interpolator.closest_waypoint_idx

    @property
    def interpolator(self):

        return self._path_generators[
            self._interp_method
        ]

    @property
    def interpolator_tags(self):

        return [
            gen.get_label()
            for gen in PathGenerator.get_all_generators()
        ]

    @property
    def use_finite_diff(self):

        return self._use_finite_diff

    @use_finite_diff.setter
    def use_finite_diff(
        self,
        flag
    ):

        self._use_finite_diff = flag

    @property
    def stamped_pose_only(self):

        return self._stamped_pose_only

    @stamped_pose_only.setter
    def stamped_pose_only(
        self,
        flag
    ):

        self._stamped_pose_only = flag

    def get_interpolation_method(
        self
    ):

        return self._interp_method

    def get_visual_markers(
        self
    ):

        return self.interpolator.get_visual_markers()

    def set_interpolation_method(
        self,
        method
    ):

        if method in self._path_generators:

            self._interp_method = method

            return True

        return False

    def set_interpolator_parameters(
        self,
        method,
        params
    ):

        if method not in self.interpolator_tags:

            return False

        return self._path_generators[
            method
        ].set_parameters(
            params
        )

    def is_full_dof(self):

        return self._is_full_dof

    def get_max_time(self):

        return self.interpolator.max_time

    def set_duration(
        self,
        t
    ):

        if t <= 0:

            return False

        self.interpolator.duration = t

        self.interpolator.s_step = (
            self._t_step /
            self.interpolator.duration
        )

        return True

    def is_finished(self):

        return self._has_ended

    def reset(self):

        self._dt = None

        self._last_t = None

        self._last_pnt = None

        self._this_pnt = None

        self._has_started = False

        self._has_ended = False

        self._cur_s = 0

    def init_waypoints(
        self,
        waypoint_set,
        init_rot=(0,0,0,1)
    ):

        self.reset()

        self.interpolator.reset()

        return self.interpolator.init_waypoints(
            waypoint_set,
            init_rot
        )

    def add_waypoint(
        self,
        waypoint,
        add_to_beginning=False
    ):

        return self.interpolator.add_waypoint(
            waypoint,
            add_to_beginning
        )

    def get_waypoints(self):

        return self.interpolator.waypoints

    def update_dt(
        self,
        t
    ):

        if self._last_t is None:

            self._last_t=t

            self._dt=0

            if self.interpolator.start_time is None:

                self.interpolator.start_time=t

            return False

        self._dt=t-self._last_t

        self._last_t=t

        return self._dt>0

    def get_samples(
        self,
        step=0.005
    ):

        return self.interpolator.get_samples(
            0.0,
            step
        )

    def set_start_time(
        self,
        t
    ):

        self.interpolator.start_time=t

    def generate_reference(
        self,
        t,
        *args
    ):

        t=max(
            t,
            self.interpolator.start_time
        )

        t=min(
            t,
            self.interpolator.max_time
        )

        pnt=self.generate_pnt(
            t,
            *args
        )

        pnt.t=t

        return pnt

    def _generate_vel(
        self,
        s=None
    ):

        if self._stamped_pose_only:

            return np.zeros(
                6
            )

        cur_s=(
            self._cur_s
            if s is None
            else s
        )

        last_s=(
            cur_s-
            self.interpolator.s_step
        )

        if last_s<0 or cur_s>1:

            return np.zeros(
                6
            )

        q_cur=self.interpolator.generate_quat(
            cur_s
        )

        q_last=self.interpolator.generate_quat(
            last_s
        )

        cur_pos=self.interpolator.generate_pos(
            cur_s
        )

        last_pos=self.interpolator.generate_pos(
            last_s
        )

        q_diff=quaternion_multiply(
            q_cur,
            quaternion_inverse(
                q_last
            )
        )

        ang_vel=(
            2*q_diff[0:3]/
            self._t_step
        )

        vel=np.array([
            (
                cur_pos[0]-
                last_pos[0]
            )/self._t_step,

            (
                cur_pos[1]-
                last_pos[1]
            )/self._t_step,

            (
                cur_pos[2]-
                last_pos[2]
            )/self._t_step,

            ang_vel[0],
            ang_vel[1],
            ang_vel[2]
        ])

        return vel

    def generate_pnt(
        self,
        t,
        pos,
        rot
    ):

        cur_s=(
            t-
            self.interpolator.start_time
        )/(
            self.interpolator.max_time-
            self.interpolator.start_time
        )

        pnt=self.interpolator.generate_pnt(
            cur_s,
            t,
            pos,
            rot
        )

        pnt.vel=self._generate_vel(
            cur_s
        )

        last_vel=self._generate_vel(
            cur_s-
            self.interpolator.s_step
        )

        pnt.acc=(
            pnt.vel-
            last_vel
        )/self._t_step

        return pnt

    def interpolate(
        self,
        t,
        *args
    ):

        if not self._has_started:

            tic=time.time()

            if not self.interpolator.init_interpolator():

                self._logger.error(
                    "Interpolator init failed"
                )

                return None

            if self.interpolator.start_time is None:

                self.set_start_time(
                    t+
                    (
                        time.time()-tic
                    )
                )

            self.interpolator.s_step=(
                self._t_step/
                (
                    self.interpolator.max_time-
                    self.interpolator.start_time
                )
            )

            self.update_dt(
                t
            )

            self._cur_s=0

            self._has_started=True

            self._has_ended=False

        if self.interpolator.is_finished(t):

            self._has_ended=True

            self._cur_s=1

        else:

            self._cur_s=(
                t-
                self.interpolator.start_time
            )/(
                self.interpolator.max_time-
                self.interpolator.start_time
            )

        self._this_pnt=self.generate_pnt(
            t,
            *args
        )

        self._this_pnt.t=t

        self._last_pnt=deepcopy(
            self._this_pnt
        )

        return self._this_pnt
