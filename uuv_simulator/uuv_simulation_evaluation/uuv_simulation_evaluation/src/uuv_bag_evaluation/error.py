# Copyright (c) 2016 The UUV Simulator Authors.
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

# ROS2 / Gazebo Harmonic (gz-sim 8) migration notes:
#   - Removed: `import tf.transformations as trans`  (ROS1-only)
#   - Added:   scipy.spatial.transform.Rotation for quaternion math
#   - Removed: `from recording import Recording` (bare import, ROS1 style)
#   - Added:   `from .recording import Recording` (relative import, ROS2 style)

import numpy as np
from scipy.spatial.transform import Rotation
from .recording import Recording


def _quaternion_matrix_3x3(q):
    """Return the 3x3 rotation matrix for a quaternion [x, y, z, w]."""
    return Rotation.from_quat(q).as_matrix()


def _quaternion_conjugate(q):
    """Conjugate of quaternion [x, y, z, w] → [-x, -y, -z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quaternion_multiply(q1, q2):
    """Hamilton product of two quaternions, both in [x, y, z, w] format."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def _euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to (roll, pitch, yaw) in radians."""
    r = Rotation.from_quat(q)
    # 'xyz' extrinsic = roll, pitch, yaw
    return r.as_euler('xyz')


class TrajectoryError(object):
    def __init__(self, p_des, p_act):
        self.p_des = p_des
        self.p_act = p_act

        self._time = p_act.t
        self._errors = dict()

        self._errors['x'] = p_des.p[0] - p_act.p[0]
        self._errors['y'] = p_des.p[1] - p_act.p[1]
        self._errors['z'] = p_des.p[2] - p_act.p[2]

        self._errors['position'] = p_des.p - p_act.p
        self._errors['linear_velocity'] = p_des.v - p_act.v
        self._errors['angular_velocity'] = p_des.w - p_act.w

        # Cross-track error: project inertial position error into the
        # desired body frame and take the lateral (y) component.
        frame = _quaternion_matrix_3x3(p_des.q)          # 3x3 rotation matrix
        e_pos_inertial = p_des.pos - p_act.pos
        e_pos_des = np.dot(frame.T, e_pos_inertial)
        self._errors['cross_track'] = e_pos_des[1]

        # Overall orientation error from the error quaternion wrt body frame.
        # q_err = conj(q_des) ⊗ q_act
        err_quat = _quaternion_multiply(
            _quaternion_conjugate(p_des.q),
            p_act.q
        )
        ca = err_quat[3]                        # w component
        sa = np.linalg.norm(err_quat[0:3])      # ||[x,y,z]||
        self._errors['angle'] = np.arctan2(sa, ca)

        # Euler angle errors (roll, pitch, yaw)
        roll_des,  pitch_des,  yaw_des  = _euler_from_quaternion(p_des.q)
        roll_act,  pitch_act,  yaw_act  = _euler_from_quaternion(p_act.q)

        self._errors['roll']  = self.wrap(roll_des  - roll_act)
        self._errors['pitch'] = self.wrap(pitch_des - pitch_act)
        self._errors['yaw']   = self.wrap(yaw_des   - yaw_act)

    @staticmethod
    def wrap(x):
        """Wrap angle to (-π, π]."""
        return (x + np.pi) % (2.0 * np.pi) - np.pi

    @property
    def t(self):
        return self._time

    @property
    def tags(self):
        return self._errors.keys()

    def get_data(self, tag):
        if tag in self._errors:
            return self._errors[tag]
        return None


class ErrorSet(object):
    """Singleton that holds and computes trajectory errors.

    ROS2 changes:
      - No ROS1-specific dependencies; pure Python / NumPy / SciPy.
      - Works with the ROS2 Recording singleton (rosbag2_py based).
    """
    __instance = None

    TAGS = [
        'x', 'y', 'z',
        'position',
        'cross_track',
        'linear_velocity',
        'angular_velocity',
        'roll', 'pitch', 'yaw',
        'quaternion',
    ]

    def __init__(self):
        self._bag = None
        self._errors = list()
        self.compute_errors()
        ErrorSet.__instance = self

    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            cls.__instance = ErrorSet()
        return cls.__instance

    def compute_errors(self):
        self._bag = Recording.get_instance()
        assert self._bag is not None, 'Recording has not been created'

        if self._bag.parsers['error'].error is None:
            t_start = self._bag.parsers['trajectory'].start_time
            t_end   = self._bag.parsers['trajectory'].end_time

            self._errors = list()

            for p_act in self._bag.parsers['trajectory'].odometry.points:
                if t_start <= p_act.t <= t_end:
                    if self._errors and p_act.t <= self._errors[-1].t:
                        continue
                    p_des = self._bag.parsers['trajectory'].reference.interpolate(p_act.t)
                    self._errors.append(TrajectoryError(p_des, p_act))

    @property
    def errors(self):
        return self._errors

    def get_time(self, tag='error'):
        if tag == 'error':
            if self._bag.parsers['error'].error is None:
                return np.array([e.t for e in self._errors])
            else:
                return np.array([e.t for e in self._bag.parsers['error'].error.points])
        else:
            return np.array([e.t for e in self._bag.parsers['trajectory'].odometry.points])

    def get_tags(self):
        return self.TAGS

    def get_data(self, tag, time_offset=0.0):
        if tag not in self.TAGS:
            return None

        if self._bag.parsers['error'].error is None and len(self._errors):
            assert 0.0 <= time_offset <= self._errors[-1].t, \
                'Time offset is off limits'
            return [e.get_data(tag) for e in self._errors if e.t >= time_offset]

        elif self._bag.parsers['error'].error is not None:
            assert 0.0 <= time_offset <= self._bag.parsers['error'].error.time[-1], \
                'Time offset is off limits'

            vec = None
            pts = self._bag.parsers['error'].error.points

            if tag == 'x':
                vec = [e.pos[0] for e in pts if e.t >= time_offset]
            elif tag == 'y':
                vec = [e.pos[1] for e in pts if e.t >= time_offset]
            elif tag == 'z':
                vec = [e.pos[2] for e in pts if e.t >= time_offset]
            elif tag == 'position':
                vec = [e.pos for e in pts if e.t >= time_offset]
            elif tag == 'linear_velocity':
                vec = [e.vel[0:3] for e in pts if e.t >= time_offset]
            elif tag == 'angular_velocity':
                vec = [e.vel[3:6] for e in pts if e.t >= time_offset]
            elif tag == 'roll':
                vec = [e.rot[0] for e in pts if e.t >= time_offset]
            elif tag == 'pitch':
                vec = [e.rot[1] for e in pts if e.t >= time_offset]
            elif tag == 'yaw':
                vec = [e.rot[2] for e in pts if e.t >= time_offset]
            elif tag == 'cross_track':
                vec = []
                for p_act in self._bag.parsers['trajectory'].odometry.points:
                    p_des = self._bag.parsers['trajectory'].reference.interpolate(p_act.t)
                    if p_des.t >= time_offset:
                        frame = _quaternion_matrix_3x3(p_des.q)
                        e_pos_inertial = p_des.pos - p_act.pos
                        e_pos_des = np.dot(frame.T, e_pos_inertial)
                        vec.append(e_pos_des[1])
            elif tag == 'quaternion':
                vec = [e.rotq[0:3] for e in pts if e.t >= time_offset]

            return vec
