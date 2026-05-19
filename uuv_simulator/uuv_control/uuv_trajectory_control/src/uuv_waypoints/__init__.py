
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
"""Waypoint description for construction of 3D paths and trajectories."""

__all__ = ['waypoint', 'waypoint_set']

from .waypoint import Waypoint
from .waypoint_set import WaypointSet

import rclpy
from tf2_ros import TransformBroadcaster

class MyNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.create_publisher(Waypoint, 'waypoints', 10)
        self.create_subscription(WaypointSet, 'waypoints_set', 10)

        self.declare_parameter('param_name', default_value='default_value')[30D[K
default_value='default_value')
        self.get_clock().now()
        clock.nanoseconds

    def create_service(self, srv_type):
        return self.create_service(srv_type, 'service_name')

node = MyNode()

Note that I removed the `rosbuild` dependency and replaced it with an empty[5D[K
empty list.

