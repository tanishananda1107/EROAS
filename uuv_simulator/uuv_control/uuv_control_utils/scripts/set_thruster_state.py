Here's the converted code:

#!/usr/bin/env python
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
import os
import rclpy
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState
from tf2_ros import TransformException

class ThrusterController:
    def __init__(self):
        self.create_publisher()

    def main(self, *args, **kwargs):
        print('Set the state of thrusters for vehicle, namespace=', self.ge[7D[K
self.get_namespace())
        rclpy.init()
        node = rclpy.node.Node('set_thrusters_states')

        if node.is_shutdown():
            raise rclpy.exceptions.ROSException('ROS master not running!')

        starting_time = 0.0
        if node.has_parameter('~starting_time'):
            starting_time = node.get_parameter '~starting_time').value

        print('Starting time={} s'.format(starting_time))

        duration = 0.0
        if node.has_parameter('~duration'):
            duration = node.get_parameter '~duration').value

        if duration == 0.0:
            raise rclpy.exceptions.ROSException('Duration not set, leaving [K
node...')

        print('Duration [s]=', ('Inf.' if duration < 0 else duration))

        is_on = None
        if node.has_parameter('~is_on'):
            is_on = node.get_parameter '~is_on').value
        else:
            raise rclpy.exceptions.ROSException('State flag not provided')

        thruster_id = None
        if node.has_parameter('~thruster_id'):
            thruster_id = node.get_parameter '~thruster_id').value
        else:
            raise rclpy.exceptions.ROSException('Thruster ID not given')

        if thruster_id < 0:
            raise rclpy.exceptions.ROSException('Invalid thruster ID')

        print('Setting state of thruster #{} as {}'.format(thruster_id, 'ON[3D[K
'ON' if is_on else 'OFF'))

        vehicle_name = self.get_namespace().replace('/', '')

        srv_name = '/%s/thrusters/%d/set_thruster_state' % (vehicle_name, t[1D[K
thruster_id)

        try:
            set_state = node.create_service(srv_name, SetThrusterState)
        except rclpy.exceptions.ROSException as e:
            raise rclpy.exceptions.ROSException('Service not available! Clo[3D[K
Closing node...')

        rate = node.get_clock().get_nanoseconds()
        while node.get_clock().now() < starting_time:
            rate.sleep()

        success = set_state(is_on)

        if success:
            print('Time={} s'.format(node.get_clock().now()))
            print('Current state of thruster #{}={}'.format(thruster_id, 'O[2D[K
'ON' if is_on else 'OFF'))

        if duration > 0:
            rate = node.get_clock().get_nanoseconds()
            while node.get_clock().now() < starting_time + duration:
                rate.sleep()

            success = set_state(not is_on)

            if success:
                print('Time={} s'.format(node.get_clock().now()))
                print('Returning to previous state of thruster #{}={}'.form[12D[K
#{}={}'.format(thruster_id, 'ON' if not is_on else 'OFF'))

        print('Leaving node...')

    def get_namespace(self):
        return os.environ['ROS_NAMESPACE']

if __name__ == '__main__':
    controller = ThrusterController()
    controller.main()

