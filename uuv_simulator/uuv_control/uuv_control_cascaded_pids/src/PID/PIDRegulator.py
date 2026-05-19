
import rclpy
from tf2_ros import Buffer, TransformListener

class PIDRegulator:
    """A very basic 1D PID Regulator."""
    def __init__(self, p, i, d, sat):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        msg += '\n\tsat=%f' % self.sat
        return msg

    def regulate(self, err, t):
        derr_dt = 0.0
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = t

        if numpy.linalg.norm(u) > self.sat:
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/numpy.linalg.norm(u)
            self.integral = 0.0

        return u

class PIDRegulatorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pid_regulator_node')
        
        self.p = self.declare_parameter('p').get_value()
        self.i = self.declare_parameter('i').get_value()
        self.d = self.declare_parameter('d').get_value()
        self.sat = self.declare_parameter('sat').get_value()

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def regulate(self, err):
        derr_dt = 0.0
        dt = self.get_clock().now().nanoseconds - self.prev_t
        if self.prev_t > 0 and dt > 0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = self.get_clock().now().nanoseconds

        if numpy.linalg.norm(u) > self.sat:
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/numpy.linalg.norm(u)
            self.integral = 0.0

        return u

    def main(self):
        err_pub = self.create_publisher('pid_regulator_err', 10)
        err_sub = self.create_subscription('pid_regulator_err', 10, self.er[7D[K
self.err_callback)

        while rclpy.node.get_time() < self.get_clock().now().nanoseconds + [K
1000:
            err = # get your error here
            u = self.regulate(err)
            err_pub.publish(u)

    def err_callback(self, msg):
        # callback for errors from the PID regulator node
        pass

if __name__ == '__main__':
    rclpy.init()
    node = PIDRegulatorNode()
    try:
        node.main()
    except KeyboardInterrupt:
        pass


