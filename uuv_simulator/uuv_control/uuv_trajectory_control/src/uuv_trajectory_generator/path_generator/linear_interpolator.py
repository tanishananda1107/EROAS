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

# ROS2 + Gazebo Sim 8 (Harmonic) conversion
# Changes from ROS1:
#   - `tf_quaternion.transformations` → `tf_transformations`
#   - `from visualization_msgs.msg import MarkerArray` unchanged (valid in ROS2)
#   - No rospy usage in this file; no further changes needed beyond the import

from scipy.interpolate import splrep, splev
import numpy as np
from copy import deepcopy

from uuv_waypoints import Waypoint, WaypointSet

# ROS2: visualization_msgs is unchanged
from visualization_msgs.msg import MarkerArray

# ROS2: tf_transformations replaces tf_quaternion.transformations
from tf_transformations import (
    quaternion_multiply,
    quaternion_about_axis,
    quaternion_conjugate,
    quaternion_from_matrix,
    euler_from_matrix,
)

from ..trajectory_point import TrajectoryPoint
from .line_segment import LineSegment
from .bezier_curve import BezierCurve
from .path_generator import PathGenerator


class LinearInterpolator(PathGenerator):
    """Simple interpolator that generates a parametric line connecting
    the input waypoints.

    > *Example*

    ```python
    from uuv_waypoints import Waypoint, WaypointSet
    from uuv_trajectory_generator import LinearInterpolator
    import numpy as np

    q_x = [0, 1, 2, 4, 5, 6]
    q_y = [0, 2, 3, 3, 2, 0]
    q_z = [0, 1, 0, 0, 2, 2]
    q = np.vstack((q_x, q_y, q_z)).T

    waypoints = WaypointSet()
    for i in range(q.shape[0]):
        waypoints.add_waypoint(
            Waypoint(q[i, 0], q[i, 1], q[i, 2], max_forward_speed=0.5))

    interpolator = LinearInterpolator()
    interpolator.init_waypoints(waypoints)
    interpolator.init_interpolator()

    pnts = interpolator.get_samples(max_time=None, step=0.01)
    pos   = interpolator.generate_pos(s=0.2)
    ```
    """
    LABEL = 'linear'

    def __init__(self):
        super(LinearInterpolator, self).__init__(self)

        self._interp_fcns = dict(pos=None, heading=None)
        self._heading_spline = None

    def init_interpolator(self):
        """Initialize the interpolator.

        > *Returns*

        `True` if the path segments were successfully generated.
        """
        if self._waypoints is None:
            return False

        if self._waypoints.num_waypoints < 2:
            return False

        self._markers_msg = MarkerArray()
        self._marker_id = 0

        self._interp_fcns['pos'] = list()
        self._segment_to_wp_map = [0]

        for i in range(1, self._waypoints.num_waypoints):
            self._interp_fcns['pos'].append(
                LineSegment(self._waypoints.get_waypoint(i - 1).pos,
                            self._waypoints.get_waypoint(i).pos))

        # Reparametrize the curves
        lengths = [seg.get_length() for seg in self._interp_fcns['pos']]
        lengths = [0] + lengths
        self._s = np.cumsum(lengths) / np.sum(lengths)
        mean_vel = np.mean(
            [self._waypoints.get_waypoint(k).max_forward_speed
             for k in range(self._waypoints.num_waypoints)])
        if self._duration is None:
            self._duration = np.sum(lengths) / mean_vel
        if self._start_time is None:
            self._start_time = 0.0

        heading = [self._waypoints.get_waypoint(k).heading_offset
                   for k in range(self._waypoints.num_waypoints)]
        self._heading_spline = splrep(self._s, heading, k=3, per=False)
        self._interp_fcns['heading'] = lambda x: splev(x, self._heading_spline)

        return True

    def set_parameters(self, params):
        """Not implemented for this interpolator."""
        return True

    def get_samples(self, max_time, step=0.001):
        """Sample the full path for position and quaternion vectors."""
        if self._waypoints is None:
            return None
        if self._interp_fcns['pos'] is None:
            return None
        s = np.arange(0, 1 + step, step)

        pnts = list()
        for i in s:
            pnt = TrajectoryPoint()
            pnt.pos = self.generate_pos(i).tolist()
            pnt.t = 0.0
            pnts.append(pnt)
        return pnts

    def generate_pos(self, s):
        """Generate a 3D position vector at parametric input `s`."""
        if self._interp_fcns['pos'] is None:
            return None
        idx = self.get_segment_idx(s)
        if idx == 0:
            u_k = 0
            pos = self._interp_fcns['pos'][idx].interpolate(u_k)
        else:
            u_k = (s - self._s[idx - 1]) / (self._s[idx] - self._s[idx - 1])
            pos = self._interp_fcns['pos'][idx - 1].interpolate(u_k)
        return pos

    def generate_pnt(self, s, t, *args):
        """Compute a trajectory point at parametric input `s`."""
        pnt = TrajectoryPoint()
        pnt.t = t
        pnt.pos = self.generate_pos(s).tolist()
        pnt.rotq = self.generate_quat(s)
        return pnt

    def generate_quat(self, s):
        """Compute the orientation quaternion at parametric input `s`."""
        s = max(0, s)
        s = min(s, 1)

        if s == 0:
            self._last_rot = deepcopy(self._init_rot)
            return self._init_rot

        last_s = max(0, s - self._s_step)

        this_pos = self.generate_pos(s)
        last_pos = self.generate_pos(last_s)

        dx = this_pos[0] - last_pos[0]
        dy = this_pos[1] - last_pos[1]
        dz = this_pos[2] - last_pos[2]

        rotq = self._compute_rot_quat(dx, dy, dz)
        self._last_rot = rotq

        # Apply heading offset
        q_step = quaternion_about_axis(
            self._interp_fcns['heading'](s),
            np.array([0, 0, 1]))
        rotq = quaternion_multiply(rotq, q_step)
        self._last_rot = rotq
        return rotq
