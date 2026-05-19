
import rclpy
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import WrenchStamped, Vector3
from uuv_world_ros_plugins_msgs.srv import *
from gazebo_msgs.srv import ApplyBodyWrench
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState, SetThrusterEf[13D[K
SetThrusterEfficiency

class DisturbanceManager:

    def __init__(self):
        self._logger = logging.getLogger('dp_local_planner')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)[12D[K
%(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        # Load disturbances and check for missing information
        specs = dict(current=['starting_time', 'velocity', 'horizontal_angl[16D[K
'horizontal_angle',
                              'vertical_angle'],
                     wrench=['starting_time', 'duration', 'force', 'torque'[8D[K
'torque'],
                     thruster_state=['starting_time', 'thruster_id', 'is_on[6D[K
'is_on',
                                     'duration'],
                     propeller_efficiency=['starting_time', 'thruster_id', [K
'duration',
                                    'efficiency'],
                     thrust_efficiency=['starting_time', 'thruster_id', 'du[3D[K
'duration',
                                 'efficiency'])

        thruster_ids = list()

        if self._disturbances is not None:
            for i in range(len(self._disturbances)):
                item = self._disturbances[i]
                if type(item) != dict:
                    raise rclpy.exceptions.ROSException('Disturbance descri[6D[K
description must be'
                                             ' given as a dict')
                if 'type' not in item:
                    raise rclpy.exceptions.ROSException('Type of disturbanc[10D[K
disturbance not '
                                             'specified')
                if item['type'] not in specs:
                    raise rclpy.exceptions.ROSException(
                        'Invalid type of disturbance, value=%s' % item['typ[9D[K
item['type'])

                for spec in specs[item['type']]:
                    if spec not in item:
                        raise rclpy.exceptions.ROSException(
                            'Invalid current model specification, '
                            'missing tag=%s' % spec)

                if item['type'] == 'thruster_state':
                    thruster_ids.append(item['thruster_id'])

                # Create flag to indicate that perturbation has been applie[6D[K
applied
                self._disturbances[i]['is_applied'] = False
                self._disturbances[i]['ended'] = False

        # List all disturbances to be applied
        for i in range(len(self._disturbances)):
            self._logger.info('Disturbance #%d: %s' % (i, self._disturbance[17D[K
self._disturbances[i]))

        self._body_force = np.zeros(3)
        self._body_torque = np.zeros(3)
        self._body_wrench_msg = WrenchStamped()

        # For body wrench disturbances, publish a topic
        self._wrench_topic = self.create_publisher(WrenchStamped, 'wrench_p[9D[K
'wrench_perturbation', 1)

        vehicle_name = self.get_namespace().replace('/', '')

        # Test if services are reachable
        try:
            services = ['/hydrodynamics/set_current_velocity',
                        '/gazebo/apply_body_wrench']
            for item in self._disturbances:
                if item['type'] == 'thruster_state':
                    services.append('/%s/thrusters/%d/set_thruster_state' %[1D[K
% (vehicle_name, item['thruster_id']))
                elif item['type'] == 'propeller_efficiency':
                    services.append('/%s/thrusters/%d/set_dynamic_state_eff[55D[K
services.append('/%s/thrusters/%d/set_dynamic_state_efficiency' % (vehicle_[9D[K
(vehicle_name, item['thruster_id']))
                elif item['type'] == 'thrust_efficiency':
                    services.append('/%s/thrusters/%d/set_thrust_force_effi[55D[K
services.append('/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_n[10D[K
(vehicle_name, item['thruster_id']))
            for s in services:
                self.wait_for_service(s)
        except Exception as e:
            self._logger.error('Some services are not available! message=' [K
+ str(e))
            self._logger.error('Closing node...')
            sys.exit(-1)

        # Obtain service proxy
        self._service_cb = dict()
        try:
            self._service_cb['current_velocity'] = self.create_serviceProxy[24D[K
self.create_serviceProxy('/hydrodynamics/set_current_velocity', SetCurrentV[11D[K
SetCurrentVelocity)
            self._service_cb['wrench'] = self.create_serviceProxy('/gazebo/[34D[K
self.create_serviceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            self._service_cb['thrusters'] = dict()
            for item in self._disturbances:
                if item['type'] == 'thruster_state':
                    if 'state' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['state'] = dict()
                    self._service_cb['thrusters']['state'][item['thruster_i[55D[K
self._service_cb['thrusters']['state'][item['thruster_id']] = self.create_s[13D[K
self.create_serviceProxy('/%s/thrusters/%d/set_thruster_state' % (vehicle_n[10D[K
(vehicle_name, item['thruster_id']), SetThrusterState)
                elif item['type'] == 'propeller_efficiency':
                    if 'propeller_efficiency' not in self._service_cb['thru[22D[K
self._service_cb['thrusters']:
                        self._service_cb['thrusters']['propeller_efficiency[51D[K
self._service_cb['thrusters']['propeller_efficiency'] = dict()
                    self._service_cb['thrusters']['propeller_efficiency'][i[55D[K
self._service_cb['thrusters']['propeller_efficiency'][item['thruster_id']] [K
= self.create_serviceProxy('/%s/thrusters/%d/set_dynamic_state_efficiency' [K
% (vehicle_name, item['thruster_id']), SetThrusterEfficiency)
                elif item['type'] == 'thrust_efficiency':
                    if 'thrust_efficiency' not in self._service_cb['thruste[25D[K
self._service_cb['thrusters']:
                        self._service_cb['thrusters']['thrust_efficiency'] [K
= dict()
                    self._service_cb['thrusters']['thrust_efficiency'][item[55D[K
self._service_cb['thrusters']['thrust_efficiency'][item['thruster_id']] = s[1D[K
self.create_serviceProxy('/%s/thrusters/%d/set_thrust_force_efficiency' % ([1D[K
(vehicle_name, item['thruster_id']), SetThrusterEfficiency)
        except rclpy.exceptions.ServiceException as e:
            self._logger.info('Service call failed, error=%s' % str(e))
            sys.exit(-1)

        self._wrench_timer = self.create_timer(rospy.Duration(0.1), self._p[7D[K
self._publish_wrench_disturbance)

        rate = Rate(100)
        while not self.is_shutdown():
            t = node.get_clock().now()
            for i in range(len(self._disturbances)):
                d = self._disturbances[i]
                if t > d['starting_time'] and not d['is_applied']:
                    #######################################################[55D[K
###########################################################
                    if d['type'] == 'current':
                        self.set_current(d['velocity'], d['horizontal_angle[19D[K
d['horizontal_angle'],
                                         d['vertical_angle'])
                    #######################################################[55D[K
###########################################################
                    elif d['type'] == 'wrench':
                        self.set_body_wrench(d['force'],
                                             d['torque'],
                                             -1,
                                             d['starting_time'])
                    #######################################################[55D[K
###########################################################
                    elif d['type'] == 'thruster_state':
                        self.set_thruster_state(d['thruster_id'], bool(d['i[9D[K
bool(d['is_on']))
                    #######################################################[55D[K
###########################################################
                    elif d['type'] == 'propeller_efficiency':
                        self.set_propeller_efficiency(d['thruster_id'], d['[3D[K
d['efficiency'])
                    #######################################################[55D[K
###########################################################
                    elif d['type'] == 'thrust_efficiency':
                        self.set_thrust_efficiency(d['thruster_id'], d['eff[6D[K
d['efficiency'])
                    # Set applied flag to true
                    self._disturbances[i]['is_applied'] = True

                    if 'duration' in d:
                        if d['duration'] == -1:
                            self._disturbances[i]['ended'] = True
                    else:
                        self._disturbances[i]['ended'] = True
                elif d['is_applied'] and 'duration' in d and not d['ended'][10D[K
d['ended']:
                    if d['duration'] > 0:
                        if node.get_clock().now() > d['starting_time'] + d[[2D[K
d['duration']:
                            ###############################################[47D[K
###########################################################
                            if d['type'] == 'current':
                                # Set current to zero
                                self.set_current(0, d['horizontal_angle'],
                                                 d['vertical_angle'])
                            ###############################################[47D[K
###########################################################
                            elif d['type'] == 'wrench':
                                # Cancel out force and torque
                                self.set_body_wrench([-1 * d['force'][n] fo[2D[K
for n in range(3)],
                                                     [-1 * d['torque'][n] f[1D[K
for n in range(3)],
                                                     -1,
                                                     node.get_clock().now()[22D[K
node.get_clock().now())
                            ###############################################[47D[K
###########################################################
                            elif d['type'] == 'thruster_state':
                                self.set_thruster_state(d['thruster_id'], n[1D[K
not bool(d['is_on']))
                            ###############################################[47D[K
###########################################################
                            elif d['type'] == 'propeller_efficiency':
                                self.set_propeller_efficiency(d['thruster_i[43D[K
self.set_propeller_efficiency(d['thruster_id'], 1.0)
                            ###############################################[47D[K
###########################################################
                            elif d['type'] == 'thrust_efficiency':
                                self.set_thrust_efficiency(d['thruster_id'][43D[K
self.set_thrust_efficiency(d['thruster_id'], 1.0)

                            self._disturbances[i]['ended'] = True
            rate.sleep()

    def _publish_wrench_disturbance(self, event):
        msg = WrenchStamped()
        msg.header.stamp = node.get_clock().now()
        msg.header.frame_id = 'world'
        msg.wrench.force = Vector3(*self._body_force)
        msg.wrench.torque = Vector3(*self._body_torque)
        # Publish the applied body wrench
        self._wrench_topic.publish(msg)

    def set_current(self, velocity, horizontal_angle, vertical_angle):
        self._logger.info('Appying current velocity model...')
        if self._service_cb['current_velocity'](velocity, horizontal_angle,[17D[K
horizontal_angle, vertical_angle):
            self._logger.info('Current velocity changed successfully at %f [K
s! vel= %f m/s' % (node.get_clock().now(), velocity))
        else:
            self._logger.error('Failed to change current velocity')

    def set_body_wrench(self, force, torque, duration, starting_time):
        ns = node.get_namespace().replace('/', '')
        body_name = '%s/base_link' % ns

        self._body_force = np.array([self._body_force[i] + force[i] for i i[1D[K
in range(3)])
        self._body_torque = np.array([self._body_torque[i] + torque[i] for [K
i in range(3)])

        self._body_wrench_msg = WrenchStamped()
        self._body_wrench_msg.header.stamp = node.get_clock().now()
        self._body_wrench_msg.header.frame_id = 'world'
        self._body_wrench_msg.wrench.force = Vector3(*self._body_force)
        self._body_wrench_msg.wrench.torque = Vector3(*self._body_torque)

        success = self._service_cb['wrench'](
            body_name,
            'world',
            Point(0, 0, 0),
            self._body_wrench_msg.wrench,
            node.get_clock().now(),
            rospy.Duration(duration))

        if success:
            self._logger.info('Body wrench perturbation applied!, body_name[9D[K
body_name=%s, t=%.2f s' % (body_name, node.get_clock().now()))
        else:
            self._logger.error('Failed to apply body wrench!, body_name=%s,[13D[K
body_name=%s, t=%.2f s' % (body_name, node.get_clock().now()))

    def set_thruster_state(self, thruster_id, is_on):
        if self._service_cb['thrusters']['state'][thruster_id](is_on):
            self._logger.info('Setting state of thruster #%d, state=%s, t=%[3D[K
t=%.2f s' % (thruster_id, 'ON' if is_on else 'OFF', node.get_clock().now())[23D[K
node.get_clock().now()))
        else:
            self._logger.error('Setting state of thruster #%d failed! t=%.2[5D[K
t=%.2f s' % (thruster_id, node.get_clock().now()))

    def set_propeller_efficiency(self, thruster_id, eff):
        if self._service_cb['thrusters']['propeller_efficiency'][thruster_i[64D[K
self._service_cb['thrusters']['propeller_efficiency'][thruster_id](eff):
            self._logger.info('Setting propeller efficiency of thruster #%d[3D[K
#%d, eff=%s, t=%.2f s' % (thruster_id, eff, node.get_clock().now()))
        else:
            self._logger.error('Setting propeller efficiency of thruster #%[2D[K
#%d failed! t=%.2f s' % (thruster_id, node.get_clock().now()))

    def set_thrust_efficiency(self, thruster_id, eff):
        if self._service_cb['thrusters']['thrust_efficiency'][thruster_id]([64D[K
self._service_cb['thrusters']['thrust_efficiency'][thruster_id](eff):
            self._logger.info('Setting thrust efficiency of thruster #%d, e[1D[K
eff=%s, t=%.2f s' % (thruster_id, eff, node.get_clock().now()))
        else:
            self._logger.error('Setting thrust efficiency of thruster #%d f[1D[K
failed! t=%.2f s' % (thruster_id, node.get_clock().now()))

if __name__ == '__main__':
    print('Starting disturbance manager')
    node = rclpy.Node('disturbance_manager')

    try:
        node = DisturbanceManager()
        node.spin()
    except rclpy.exceptions.ROSInterruptException:
        print('caught exception')
    print('exiting')

