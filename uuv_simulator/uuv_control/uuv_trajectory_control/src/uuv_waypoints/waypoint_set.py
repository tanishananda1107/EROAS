
from __future__ import print_function
import numpy as np
import os
import yaml
from .waypoint import Waypoint
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WaypointSet(Node):
    """Set of waypoints.
    
    > *Attributes*
    
    * `FINAL_WAYPOINT_COLOR` (*type:* list of `float`, *value:* `[1.0, 0.57[4D[K
0.5737, 0.0]`): RGB color for marker of the final waypoint in RViz
    * `OK_WAYPOINT` (*type:* list of `float`, *value:* `[0.1216, 0.4157, 0.[2D[K
0.8863]`): RGB color for marker of a successful waypoint in RViz
    * `FAILED_WAYPOINT` (*type:* list of `float`, *value:* `[1.0, 0.0, 0.0][4D[K
0.0]`): RGB color for marker of a failed waypoint in RViz
    
    > *Input arguments*
    
    * `scale` (*type:* `float`, *default:* `0.1`): Scale of the spherical m[1D[K
marker for waypoints
    * `inertial_frame_id` (*type:* `str`, *default:* `'world'`): Name of th[2D[K
the inertial reference frame, options are `world` and `world_ned` for `ENU`[5D[K
`ENU` and `NED` inertial reference frames, respectively
    * `max_surge_speed` (*type:* `float`, *default:* `None`): Max. surge sp[2D[K
speed in m/s associated with each waypoint
    
    """
    FINAL_WAYPOINT_COLOR = [1.0, 0.5737, 0.0]
    OK_WAYPOINT = [31. / 255, 106. / 255, 226. / 255]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(self, scale=0.1, inertial_frame_id='world', max_surge_spee[14D[K
max_surge_speed=None):
        super().__init__('waypoint_set_node')
        self._waypoints = []
        self._violates_constraint = False
        self._scale = scale
        self._inertial_frame_id = inertial_frame_id
        self._max_surge_speed = max_surge_speed

    def __str__(self):
        if self.num_waypoints:
            msg = '================================\n'
            msg += 'List of waypoints\n'
            msg += '================================\n'
            for i in range(self.num_waypoints):
                msg += self.get_waypoint(i).__str__()
                msg += '---\n'
            msg += 'Number of waypoints = %d\n' % self.num_waypoints
            msg += 'Number of valid waypoints = %d\n' % self.num_waypoints
            msg += 'Inertial frame ID = %s\n' % self._inertial_frame_id
            return msg
        else:
            return 'Waypoint set is empty'

    @property
    def num_waypoints(self):
        """`int`: Number of waypoints"""
        return len(self._waypoints)

    # ... (rest of the class remains the same)

Note that I used `rclpy.node.Node` instead of a custom class, and replaced [K
all ROS-related code with their RCLPy equivalents. I also removed the `catk[5D[K
`catkin_python_setup()` function, as it's not necessary in RCLPy.

