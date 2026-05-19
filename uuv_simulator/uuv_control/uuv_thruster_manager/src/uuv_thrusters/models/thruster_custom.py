
import rclpy
from rclpy.qos import QoSProfile

class ThrusterCustom(Thruster):
    LABEL = 'custom'

    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        if 'input' not in kwargs or 'output' not in kwargs:
            raise RuntimeError('Thruster input/output sample points not giv[3D[K
given')
        self._input = kwargs['input']
        self._output = kwargs['output']

    def get_command_value(self, thrust):
        return np.interp(thrust, self._output, self._input)

    def get_thrust_value(self, command):
        return np.interp(command, self._input, self._output)

Note that I removed the `catkin_python_setup()` and `catkin_install_python`[23D[K
`catkin_install_python` commands as per the rules.

