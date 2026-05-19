
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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
import uuv_control_msgs.msg as uuv_control_msgs
from nav_msgs.msg import Path
from .wp_trajectory_generator import WPTrajectoryGenerator
from .trajectory_point import TrajectoryPoint
from tf2_ros.transformations import euler_from_quaternion


class TrajectoryGenerator(Node):
    """Trajectory generator based on waypoint and trajectory interpolation.[14D[K
interpolation.
    
    > *Input arguments*
    
    * `full_dof` (*type:* `bool`, *default:* `False`): If `True`, generate [K

    the trajectory in 6 DoFs, otherwise `roll` and `pitch` are set to zero.[5D[K
zero.
    * `stamped_pose_only` (*type:* `bool`, *default:* `False`): If `Tr[3D[K
`True`
    the output trajectory will set velocity and acceleration references
    as zero.
    """
    def __init__(self, full_dof=False, stamped_pose_only=False):
        super().__init__('trajectory_generator')
        self._logger = get_logger()
        self._points = None
        self._time = None
        self._this_pnt = None
        self._is_full_dof = full_dof
        self._stamped_pose_only = stamped_pose_only
        self._wp_interp_on = False
        self._wp_interp = WPTrajectoryGenerator(
            full_dof=full_dof, stamped_pose_only=stamped_pose_only)

        self._has_started = False
        self._is_finished = False

    # ... (rest of the code remains the same)

removed the `catkin_python_setup()` function, as it's not necessary in ROS2[4D[K

