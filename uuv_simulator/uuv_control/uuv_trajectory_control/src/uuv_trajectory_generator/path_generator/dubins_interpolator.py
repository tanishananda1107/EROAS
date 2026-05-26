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
#   - `import rospy` removed; rospy.Time.now() / rospy.is_shutdown() replaced
#     with rclpy equivalents inside _get_circle_marker
#   - `tf_quaternion.transformations` → `tf_transformations`
#   - `from visualization_msgs.msg import MarkerArray, Marker` unchanged
#   - `from geometry_msgs.msg import Point` unchanged
#   - Marker stamp now uses rclpy.clock.Clock().now().to_msg()

import numpy as np
from copy import deepcopy

# ROS2 imports
import rclpy
from rclpy.clock import Clock

# ROS2: tf_transformations replaces tf_quaternion.transformations
from tf_transformations import (
    quaternion_multiply,
    quaternion_about_axis,
    quaternion_conjugate,
    quaternion_from_matrix,
    euler_from_matrix,
)

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from uuv_waypoints import Waypoint, WaypointSet

from ..trajectory_point import TrajectoryPoint
from .._log import get_logger
from .helical_segment import HelicalSegment
from .bezier_curve import BezierCurve
from .line_segment import LineSegment
from .path_generator import PathGenerator


class DubinsInterpolator(PathGenerator):
    """3D Dubins path interpolator.

    !!! note

        Owen, Mark, Randal W. Beard, and Timothy W. McLain.
        "Implementing Dubins Airplane Paths on Fixed-Wing UAVs."
        Handbook of Unmanned Aerial Vehicles (2014): 1677-1701.

        Cai, Wenyu, Meiyan Zhang, and Yahong Zheng. "Task Assignment
        and Path Planning for Multiple Autonomous Underwater Vehicles
        Using 3D Dubins Curves." Sensors 17.7 (2017):1607.

        Hansen, Karl D., and Anders La Cour-Harbo. "Waypoint Planning
        with Dubins Curves Using Genetic Algorithms." 2016 European
        Control Conference (ECC) (2016).

        Lin, Yucong, and Srikanth Saripalli. "Path Planning Using
        3D Dubins Curve for Unmanned Aerial Vehicles." 2014
        International Conference on Unmanned Aircraft Systems (ICUAS)
        (2014).
    """
    LABEL = 'dubins'

    def __init__(self):
        super(DubinsInterpolator, self).__init__(self)

        self._radius = 5
        self._max_pitch_angle = 5.0 * np.pi / 180
        self._interp_fcns = list()
        self._logger = get_logger()

    def init_interpolator(self):
        """Initialize the interpolator. To have the path segments generated,
        `init_waypoints()` must be called beforehand by providing a set of
        waypoints as `uuv_waypoints.WaypointSet` type.

        > *Returns*

        `bool`: `True` if the path segments were successfully generated.
        """
        if self._waypoints is None:
            self._logger.error('List of waypoints is empty')
            return False

        if self._waypoints.num_waypoints < 2:
            self._logger.error('At least 2 waypoints are necessary')
            return False

        self._markers_msg = MarkerArray()
        self._marker_id = 0

        self._interp_fcns = list()
        path = list()
        last_heading = self._waypoints.get_waypoint(0).heading_offset

        dist = lambda x, y: np.sqrt(np.sum((x - y)**2))

        for i in range(1, self._waypoints.num_waypoints):
            heading_init = 0.0
            heading_final = 0.0

            if i - 1 == 0:
                heading_init = self._waypoints.get_waypoint(i - 1).heading_offset
            else:
                if not np.isclose(
                        dist(self._waypoints.get_waypoint(i - 1).pos[0:2],
                             self._waypoints.get_waypoint(i).pos[0:2]), 0):
                    heading_init = self._waypoints.get_waypoint(i - 1).calculate_heading(
                        self._waypoints.get_waypoint(i))
                else:
                    heading_init = last_heading

            if i == self._waypoints.num_waypoints - 1:
                if not np.isclose(
                        dist(self._waypoints.get_waypoint(i - 1).pos[0:2],
                             self._waypoints.get_waypoint(i).pos[0:2]), 0):
                    heading_final = self._waypoints.get_waypoint(i - 1).calculate_heading(
                        self._waypoints.get_waypoint(i))
                else:
                    heading_final = last_heading
            else:
                if not np.isclose(
                        dist(self._waypoints.get_waypoint(i + 1).pos[0:2],
                             self._waypoints.get_waypoint(i).pos[0:2]), 0):
                    heading_final = self._waypoints.get_waypoint(i).calculate_heading(
                        self._waypoints.get_waypoint(i + 1))
                else:
                    heading_final = last_heading

            last_heading = heading_final

            path += self._generate_path(
                self._waypoints.get_waypoint(i - 1), heading_init,
                self._waypoints.get_waypoint(i), heading_final)

        inter_pnts = list()
        for i in range(len(path) - 1):
            if not np.isclose(np.sqrt(np.sum((path[i + 1] - path[i])**2)), 0):
                inter_pnts += [path[i]]

        if not np.isclose(np.sqrt(np.sum((path[-1] - path[-2])**2)), 0):
            inter_pnts += [path[-1]]

        self._interp_fcns, tangent = BezierCurve.generate_cubic_curve(inter_pnts)

        # Reparametrize the curves
        lengths = [seg.get_length() for seg in self._interp_fcns]
        lengths = [0] + lengths
        self._s = np.cumsum(lengths) / np.sum(lengths)
        mean_vel = np.mean(
            [self._waypoints.get_waypoint(k).max_forward_speed
             for k in range(self._waypoints.num_waypoints)])

        if self._duration is None:
            self._duration = np.sum(lengths) / mean_vel
        if self._start_time is None:
            self._start_time = 0.0

        return True

    def _get_frame(self, heading):
        """Return the 2D rotation matrix for a desired heading."""
        return np.array([
            [np.cos(heading), -np.sin(heading)],
            [np.sin(heading),  np.cos(heading)]
        ])

    def _get_distance(self, pnt_1, pnt_2):
        """Compute the distance between two points."""
        return np.sqrt(np.sum((pnt_1 - pnt_2)**2))

    def _get_circles_center_pos(self, waypoint, heading, radius=None):
        """Return the centers of the circles on the left and right of the
        waypoint with respect to the direction described the desired heading.
        """
        r = self._radius if radius is None else radius

        if r <= 0:
            msg = 'Radius must be greater than zero'
            self._logger.error(msg)
            raise ValueError(msg)

        frame = self._get_frame(heading)
        circles = dict(
            R=waypoint.pos[0:2] - r * frame[:, 1].flatten(),
            L=waypoint.pos[0:2] + r * frame[:, 1].flatten()
        )
        return circles

    def _get_phi(self, u, delta, heading):
        return 2 * np.pi * u * delta + heading - delta * np.pi / 2

    def _compute_u(self, angle, delta, heading):
        """Compute the parametric input for the circle path."""
        u = (angle - heading + delta * np.pi / 2) / (delta * 2 * np.pi)
        if u < 0:
            u += 1
        return u

    def _get_tangent(self, u, delta, radius, heading):
        return np.cross(
            np.array([0, 0, 1]),
            np.array([
                delta * radius * np.cos(self._get_phi(u, delta, heading)),
                delta * radius * np.sin(self._get_phi(u, delta, heading)),
                0
            ])
        )[0:2]

    def _get_circle(self, u, center, radius, delta, heading):
        """Compute the 2D coordinates for a circle."""
        return center + radius * np.array([
            np.cos(self._get_phi(u, delta, heading)),
            np.sin(self._get_phi(u, delta, heading))
        ])

    def _get_2d_dubins_path(self, center_1, radius_1, heading_1, delta_1,
                             center_2, radius_2, heading_2, delta_2):
        """Compute the 2D Dubins path (shortest curve between two oriented points)."""
        output = list()

        u1_func = lambda angle: self._compute_u(angle, delta_1, heading_1)
        u2_func = lambda angle: self._compute_u(angle, delta_2, heading_2)

        tan_func_1 = lambda u: self._get_tangent(u, delta_1, radius_1, heading_1)
        tan_func_2 = lambda u: self._get_tangent(u, delta_2, radius_2, heading_2)

        circle_1 = lambda u: self._get_circle(u, center_1, radius_1, delta_1, heading_1)
        circle_2 = lambda u: self._get_circle(u, center_2, radius_2, delta_2, heading_2)

        # --- Outer tangents ---
        d = center_2 - center_1
        n = np.dot(self._get_frame(np.pi / 2), d / np.linalg.norm(d))
        n_angle = np.arctan2(n[1], n[0])

        # First tangent
        u1 = u1_func(n_angle)
        u2 = u2_func(n_angle)
        c1 = circle_1(u1)
        c2 = circle_2(u2)
        t1 = tan_func_1(u1) / np.linalg.norm(tan_func_1(u1))
        t2 = tan_func_2(u2) / np.linalg.norm(tan_func_2(u2))
        tangent_1 = (c2 - c1) / np.linalg.norm(c2 - c1)

        if np.isclose(np.linalg.norm(tangent_1 - t1) + np.linalg.norm(tangent_1 - t2), 0):
            dist = 0.0
            output = []
            if not np.isclose(u1, 0):
                u = np.arange(0, u1, u1 / 10.0)
                output = [circle_1(ui) for ui in u]
                dist = 2 * radius_1 * np.pi * u1
            output += [circle_1(u1)]
            dist += np.linalg.norm(circle_2(u2) - circle_1(u1))
            if not np.isclose(u2, 1):
                u = np.arange(u2, 1, (1 - u2) / 10.0)
                output += [circle_2(ui) for ui in u]
                dist += 2 * radius_2 * np.pi * (1 - u2)
            output += [circle_2(1)]
            return output, dist

        # Second tangent
        n_angle = np.arctan2(-n[1], -n[0])
        u1 = u1_func(n_angle)
        u2 = u2_func(n_angle)
        c1 = circle_1(u1)
        c2 = circle_2(u2)
        t1 = tan_func_1(u1) / np.linalg.norm(tan_func_1(u1))
        t2 = tan_func_2(u2) / np.linalg.norm(tan_func_2(u2))
        tangent_2 = (c2 - c1) / np.linalg.norm(c2 - c1)

        if np.isclose(np.linalg.norm(tangent_2 - t1) + np.linalg.norm(tangent_2 - t2), 0):
            dist = 0.0
            output = []
            if not np.isclose(u1, 0):
                u = np.arange(0, u1, u1 / 10.0)
                output = [circle_1(ui) for ui in u]
                dist = 2 * radius_1 * np.pi * u1
            output += [circle_1(u1)]
            dist += np.linalg.norm(circle_2(u2) - circle_1(u1))
            if not np.isclose(u2, 1):
                u = np.arange(u2, 1, (1 - u2) / 10.0)
                output += [circle_2(ui) for ui in u]
                dist += 2 * radius_2 * np.pi * (1 - u2)
            output += [circle_2(1)]
            return output, dist

        # --- Inner tangents (only when circles don't overlap) ---
        if self._get_distance(center_1, center_2) > (radius_1 + radius_2):
            xp = (center_1[0] * radius_1 + center_2[0] * radius_2) / (radius_1 + radius_2)
            yp = (center_1[1] * radius_1 + center_2[1] * radius_2) / (radius_1 + radius_2)

            def _tangent_pts(cx, cy, r, xp, yp):
                dx = xp - cx
                dy = yp - cy
                d2 = dx**2 + dy**2
                sq = np.sqrt(d2 - r**2)
                xt1 = (r**2 * dx + r * dy * sq) / d2 + cx
                xt2 = (r**2 * dx - r * dy * sq) / d2 + cx
                yt1 = (r**2 * dy - r * dx * sq) / d2 + cy
                yt2 = (r**2 * dy + r * dx * sq) / d2 + cy
                return xt1, xt2, yt1, yt2

            xt1, xt2, yt1, yt2 = _tangent_pts(
                center_1[0], center_1[1], radius_1, xp, yp)
            xt3, xt4, yt3, yt4 = _tangent_pts(
                center_2[0], center_2[1], radius_2, xp, yp)

            for (xa, ya, xb, yb) in [(xt1, yt1, xt3, yt3), (xt2, yt2, xt4, yt4)]:
                u1 = u1_func(np.arctan2(ya - center_1[1], xa - center_1[0]))
                u2 = u2_func(np.arctan2(yb - center_2[1], xb - center_2[0]))
                c1 = circle_1(u1)
                c2 = circle_2(u2)
                t1 = tan_func_1(u1) / np.linalg.norm(tan_func_1(u1))
                t2 = tan_func_2(u2) / np.linalg.norm(tan_func_2(u2))
                tang = np.array([xb - xa, yb - ya])
                tang /= np.linalg.norm(tang)
                diff = np.linalg.norm(tang - t1) + np.linalg.norm(tang - t2)

                if np.isclose(diff, 0):
                    dist = 0.0
                    output = []
                    if not np.isclose(u1, 0):
                        u = np.arange(0, u1, u1 / 10.0)
                        output = [circle_1(ui) for ui in u]
                        dist = 2 * radius_1 * np.pi * u1
                    output += [circle_1(u1)]
                    dist += np.linalg.norm(circle_2(u2) - circle_1(u1))
                    if not np.isclose(u2, 1):
                        u = np.arange(u2, 1, (1 - u2) / 10.0)
                        output += [circle_2(ui) for ui in u]
                        dist += 2 * radius_2 * np.pi * (1 - u2)
                    output += [circle_2(1)]
                    return output, dist

        return output, 0

    def _get_center(self, side, y_vec, wp):
        if side == 'R':
            return wp.pos - self._radius * y_vec
        else:
            return wp.pos + self._radius * y_vec

    def _get_circle_marker(self, center, radius, heading, delta, frame_id,
                            circle_color=None):
        """Build RViz marker for a Dubins circle arc.

        ROS2 change: rospy.Time.now() replaced with
        rclpy.clock.Clock().now().to_msg(); rospy.is_shutdown() guard removed.
        """
        if circle_color is None:
            circle_color = [0, 1, 0]

        # ROS2 timestamp
        now_msg = Clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now_msg          # ROS2: was rospy.Time.now()
        marker.ns = 'dubins'
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = float(circle_color[0])
        marker.color.g = float(circle_color[1])
        marker.color.b = float(circle_color[2])

        for i in np.linspace(0, 1, 50):
            c_pnt = self._get_circle(i, center[0:2], radius, delta, heading)
            p = Point()
            p.x = float(c_pnt[0])
            p.y = float(c_pnt[1])
            p.z = float(center[2])
            marker.points.append(p)

        marker_pnt = Marker()
        marker_pnt.header.frame_id = frame_id
        marker_pnt.header.stamp = now_msg      # ROS2
        marker_pnt.ns = 'dubins'
        self._marker_id += 1
        marker_pnt.id = self._marker_id
        self._marker_id += 1
        marker_pnt.type = Marker.SPHERE
        marker_pnt.action = Marker.ADD

        marker_pnt.scale.x = 0.2
        marker_pnt.scale.y = 0.2
        marker_pnt.scale.z = 0.2
        marker_pnt.color.a = 1.0
        marker_pnt.color.r = 1.0
        marker_pnt.color.g = 0.2
        marker_pnt.color.b = 0.0

        c_pnt = self._get_circle(0, center[0:2], radius, delta, heading)
        marker_pnt.pose.position.x = float(c_pnt[0])
        marker_pnt.pose.position.y = float(c_pnt[1])
        marker_pnt.pose.position.z = float(center[2])

        return [marker, marker_pnt]

    def _generate_path(self, wp_init, heading_init, wp_final, heading_final):
        pnts = list()

        max_step_z = 2 * np.pi * self._radius * np.cos(self._max_pitch_angle)

        frame_init = np.array([
            [np.cos(heading_init), -np.sin(heading_init), 0],
            [np.sin(heading_init),  np.cos(heading_init), 0],
            [0, 0, 1]
        ])
        frame_final = np.array([
            [np.cos(heading_final), -np.sin(heading_final), 0],
            [np.sin(heading_final),  np.cos(heading_final), 0],
            [0, 0, 1]
        ])

        heading_diff = (heading_final - heading_init + np.pi) % (2.0 * np.pi) - np.pi
        dist_xy = np.sqrt(np.sum((wp_init.pos[0:2] - wp_final.pos[0:2])**2))

        if np.isclose(heading_diff, 0) or np.isclose(dist_xy, 0):
            if abs(wp_init.z - wp_final.z) <= max_step_z and not np.isclose(dist_xy, 0):
                pnts = [wp_init.pos, wp_final.pos]
            else:
                z = wp_final.z - wp_init.z
                if z >= max_step_z:
                    delta_z = z / (
                        np.floor(abs(z) / max_step_z) + np.ceil(abs(z) % max_step_z))
                else:
                    delta_z = z
                n = z / delta_z

                center = self._get_center('R', frame_final[:, 1].flatten(), wp_final)
                center[2] = wp_init.z

                delta = -1
                helix = HelicalSegment(
                    center, self._radius, n,
                    wp_final.z - wp_init.z,
                    heading_final - delta * np.pi / 2,
                    False)

                for i in np.linspace(0, 1, int(n * 10)):
                    pnts.append(helix.interpolate(i))
            return pnts

        modes = ['RSR', 'RSL', 'LSR', 'LSL']

        # Visual markers for all candidate circles
        for side, delta_val in [('R', -1), ('L', 1)]:
            for frame, heading, wp in [
                    (frame_init, heading_init, wp_init),
                    (frame_final, heading_final, wp_final)]:
                c = self._get_center(side, frame[:, 1].flatten(), wp)
                self._markers_msg.markers += self._get_circle_marker(
                    c, self._radius, heading, delta_val, wp_final.inertial_frame_id)

        path = None
        min_dist = None
        mode = None

        for c in modes:
            center_1 = self._get_center(c[0], frame_init[:, 1].flatten(), wp_init)
            center_2 = self._get_center(c[-1], frame_final[:, 1].flatten(), wp_final)

            delta_1 = -1 if c[0] == 'R' else 1
            delta_2 = -1 if c[-1] == 'R' else 1

            output, dist = self._get_2d_dubins_path(
                center_1[0:2], self._radius, heading_init, delta_1,
                center_2[0:2], self._radius, heading_final, delta_2)

            if len(output) > 0:
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    path = output
                    mode = c

        pnts = list()

        if np.isclose(abs(wp_init.z - wp_final.z), 0):
            for pnt in path:
                pnts.append(np.array([pnt[0], pnt[1], wp_final.z]))
        elif abs(wp_init.z - wp_final.z) <= max_step_z and \
                not np.isclose(abs(wp_init.z - wp_final.z), 0):
            d = [0.0] + [
                np.sqrt(np.sum((path[i] - path[i - 1])**2))
                for i in range(1, len(path))]
            dz = float(wp_final.z - wp_init.z) * np.cumsum(d) / np.sum(d)
            for i in range(len(path)):
                pnts.append(np.array([path[i][0], path[i][1], wp_init.z + dz[i]]))
        else:
            z = wp_final.z - wp_init.z
            delta_z = z / (
                np.floor(abs(z) / max_step_z) + np.ceil(abs(z) % max_step_z))
            n = z / delta_z

            d = [0.0] + [
                np.sqrt(np.sum((path[i] - path[i - 1])**2))
                for i in range(1, len(path))]
            dz = delta_z * np.cumsum(d) / np.sum(d)

            for i in range(len(path)):
                pnts.append(np.array([path[i][0], path[i][1], wp_init.z + dz[i]]))

            center = self._get_center(
                mode[-1], frame_final[:, 1].flatten(), wp_final)
            center[2] = wp_init.z + delta_z

            delta = -1 if mode[-1] == 'R' else 1
            helix = HelicalSegment(
                center, self._radius, n - 1,
                wp_final.z - center[2],
                heading_final - delta * np.pi / 2,
                False if mode[-1] == 'R' else True)

            for i in np.linspace(0, 1, int((n - 1) * 10)):
                pnts.append(helix.interpolate(i))

        return pnts

    def set_parameters(self, params):
        """Set interpolator parameters.

        ```python
        params=dict(radius=0.0, max_pitch=0.0)
        ```
        """
        if 'radius' in params:
            assert params['radius'] > 0, 'Radius must be greater than zero'
            self._radius = params['radius']
        if 'max_pitch' in params:
            assert 0 < params['max_pitch'] <= np.pi, 'Invalid max. pitch'
            self._max_pitch_angle = params['max_pitch']
        return True

    def get_samples(self, max_time, step=0.001):
        """Sample the full path for position and quaternion vectors."""
        if self._waypoints is None:
            return None
        if len(self._interp_fcns) == 0:
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
        if len(self._interp_fcns) == 0:
            return None
        idx = self.get_segment_idx(s)
        if idx == 0:
            u_k = 0
            pos = self._interp_fcns[idx].interpolate(u_k)
        else:
            u_k = (s - self._s[idx - 1]) / (self._s[idx] - self._s[idx - 1])
            pos = self._interp_fcns[idx - 1].interpolate(u_k)
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
        return rotq
