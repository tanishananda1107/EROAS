
import numpy as np
from .vehicle import Vehicle, cross_product_operator
from uuv_thrusters import ThrusterManager
from uuv_auv_actuator_interface import ActuatorManager
try: 
    import casadi
    CASADI_IMPORTED = True
except ImportError:
    CASADI_IMPORTED = False


class SymVehicle(Vehicle):
    def __init__(self, inertial_frame_id='world'):
        super().__init__(inertial_frame_id)
    
        if CASADI_IMPORTED:
            # Declaring state variables
            ## Generalized position vector
            self.eta = rclpy.serialization.get_parameter('eta', 6)
            ## Generalized velocity vector
            self.nu = rclpy.serialization.get_parameter('nu', 6)

            # Build the Coriolis matrix
            self.CMatrix = tf2_ros.transform_to_matrix(cross_product_operat[48D[K
tf2_ros.transform_to_matrix(cross_product_operator(
                np.matmul(self._Mtotal[0:3, 0:3], self.nu[0:3]) +
                np.matmul(self._Mtotal[0:3, 3:6], self.nu[3:6])))

            S_12 = - cross_product_operator(
                np.matmul(self._Mtotal[0:3, 0:3], self.nu[0:3]) +
                np.matmul(self._Mtotal[0:3, 3:6], self.nu[3:6]))
            S_22 = - cross_product_operator(
                np.matmul(self._Mtotal[3:6, 0:3], self.nu[0:3]) +
                np.matmul(self._Mtotal[3:6, 3:6], self.nu[3:6]))

            self.CMatrix[0:3, 3:6] = S_12
            self.CMatrix[3:6, 0:3] = S_12
            self.CMatrix[3:6, 3:6] = S_22

            # Build the damping matrix (linear and nonlinear elements)
            self.DMatrix = - np.diag(self._linear_damping)        
            self.DMatrix -= np.diag(self._linear_damping_forward_speed)
            self.DMatrix -= np.diag(self._quad_damping * self.nu)      

            # Build the restoring forces vectors wrt the BODY frame
            Rx = np.array([[1, 0, 0],
                        [0, np.cos(self.eta[3]), -np.sin(self.eta[3])],
                        [0, np.sin(self.eta[3]), np.cos(self.eta[3])]])
            Ry = np.array([[np.cos(self.eta[4]), 0, np.sin(self.eta[4])],
                        [0, 1, 0],
                        [-np.sin(self.eta[4]), 0, np.cos(self.eta[4])]])
            Rz = np.array([[np.cos(self.eta[5]), -np.sin(self.eta[5]), 0],
                        [np.sin(self.eta[5]), np.cos(self.eta[5]), 0],
                        [0, 0, 1]])

            R_n_to_b = np.transpose(np.matmul(Rz, np.matmul(Ry, Rx)))

            if inertial_frame_id == 'world_ned':
                Fg = np.array([0, 0, -self.mass * self.gravity])
                Fb = np.array([0, 0, self.volume * self.gravity * self.dens[9D[K
self.density])
            else:
                Fg = np.array([0, 0, self.mass * self.gravity])
                Fb = np.array([0, 0, -self.volume * self.gravity * self.den[8D[K
self.density])

            self.gVec = np.zeros(6)

            self.gVec[0:3] = -1 * np.matmul(R_n_to_b, Fg + Fb)  
            self.gVec[3:6] = -1 * np.matmul(
                R_n_to_b, np.cross(self._cog, Fg) + np.cross(self._cob, Fb)[3D[K
Fb))
            
            # Build Jacobian
            T = 1 / np.cos(self.eta[4]) * np.array(
                [[0, np.sin(self.eta[3]) * np.sin(self.eta[4]), np.cos(self[11D[K
np.cos(self.eta[3]) * np.sin(self.eta[4])],
                [0, np.cos(self.eta[3]) * np.cos(self.eta[4]), -np.cos(self[12D[K
-np.cos(self.eta[4]) * np.sin(self.eta[3])],
                [0, np.sin(self.eta[3]), np.cos(self.eta[3])]])

            self.eta_dot = np.concatenate(
                (np.matmul(np.transpose(R_n_to_b), self.nu[0:3]),
                np.matmul(T, self.nu[3::])))

            self.u = np.array('u', 6)
            
            self.nu_dot = casadi.solve(
                self._Mtotal, 
                self.u - np.matmul(self.CMatrix, self.nu) - np.matmul(self.[15D[K
np.matmul(self.DMatrix, self.nu) - self.gVec)

the `catkin_python_setup()` function. I also updated the imports to use `am[3D[K
`ament_cmake` instead of `catkin`.

