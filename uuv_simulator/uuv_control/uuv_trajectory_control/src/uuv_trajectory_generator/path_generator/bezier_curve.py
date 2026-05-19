
import rclpy
from tf2_ros import Buffer, TransformListener

class BezierCurve:
    def __init__(self, pnts, order, tangents=None, normals=None):
        self._pnts = list()
        for pnt in pnts:
            if type(pnt) == list:
                assert len(pnt) == 3, 'Point must have three elements'
                self._pnts.append(np.array(pnt))
            elif type(pnt) == np.ndarray:
                assert pnt.size == 3, 'Point must have three elements'
                self._pnts.append(pnt)
            else:
                raise TypeError('Point in list is neither a list or an arra[4D[K
array')

        if tangents is not None:
            assert type(tangents) == list and len(tangents) == 2, 'Tangent [K
vectors must be provided'
            for t in tangents:
                if type(t) == list:
                    assert len(t) == 3, 'Tangent vector must have three ele[3D[K
elements'
                elif type(t) == np.ndarray:
                    assert t.size == 3, 'Tangent vector must have three ele[3D[K
elements'
                else:
                    raise TypeError('Tangent vector is neither a list or an[2D[K
an array')

        self._control_pnts = [np.zeros(3) for _ in range(order + 1)]

        self._order = order
        if self._order == 3:
            assert len(self._pnts) == 2, 'Two points are needed for the cur[3D[K
curve to be computed'
            # Setting initial and last control points
            self._control_pnts[0] = self._pnts[0]
            self._control_pnts[3] = self._pnts[1]
            # Compute alpha
            a = 16 - np.linalg.norm(tangents[0] + tangents[1])**2
            b = 12 * np.dot(self._control_pnts[3] - self._control_pnts[0], [K
tangents[0] + tangents[1])
            c = -36 * np.linalg.norm(self._control_pnts[3] - self._control_[14D[K
self._control_pnts[0])**2
            alpha = np.roots([a, b, c]).max()

            # Compute the rest of the control points
            self._control_pnts[1] = self._control_pnts[0] + (1.0 / 3) * alp[3D[K
alpha * tangents[0]
            self._control_pnts[2] = self._control_pnts[3] - (1.0 / 3) * alp[3D[K
alpha * tangents[1]
        elif self._order == 4:
            assert len(self._pnts) == 3, 'Three points are needed for the c[1D[K
curve to be computed'
            # Setting initial control points
            self._control_pnts[0] = self._pnts[0]
            self._control_pnts[2] = self._pnts[1]
            self._control_pnts[4] = self._pnts[2]

            radius = np.linalg.norm(self._pnts[0] - self._pnts[1])
            tangents = list()
            tangents.append((self._pnts[1] - self._pnts[0]) / radius)
            tangents.append((self._pnts[2] - self._pnts[1]) / radius)

            # Compute alpha
            a = 4 - (1.0 / 4) * np.linalg.norm(tangents[0] + tangents[1])**[14D[K
tangents[1])**2
            b = 3 * np.dot(self._control_pnts[4] - self._control_pnts[0], t[1D[K
tangents[0] + tangents[1])
            c = -9 * np.linalg.norm(self._control_pnts[4] - self._control_p[15D[K
self._control_pnts[0])**2
            alpha = np.roots([a, b, c]).max()

            # Compute the rest of the control points
            self._control_pnts[1] = self._control_pnts[0] + 0.25 * alpha * [K
tangents[0]
            self._control_pnts[3] = self._control_pnts[4] - 0.25 * alpha * [K
tangents[1]
        elif self._order == 5:
            if len(self._pnts) == 3:            
                # Setting initial control points
                self._control_pnts[0] = self._pnts[0]
                self._control_pnts[5] = self._pnts[2]

                radius = np.linalg.norm(self._pnts[0] - self._pnts[1])
                tangents = list()
                tangents.append((self._pnts[1] - self._pnts[0]) / radius)
                tangents.append((self._pnts[2] - self._pnts[1]) / radius)

                # Compute alpha
                a = 256 - 49 * np.linalg.norm(tangents[0] + tangents[1])**2[15D[K
tangents[1])**2
                b = 420 * np.dot(self._control_pnts[5] - sel[3D[K
self._control_pnts[0], tangents[0] + tangents[1])
                c = -900 * np.linalg.norm(self._control_pnts[5] - self._con[9D[K
self._control_pnts[0])**2
                alpha = np.roots([a, b, c]).max()

                # Compute the rest of the control points
                self._control_pnts[1] = self._control_pnts[0] + 0.2 * alpha[5D[K
alpha * tangents[0]
                self._control_pnts[2] = 2 * self._control_pnts[1] - self._c[7D[K
self._control_pnts[0]
                self._control_pnts[4] = self._control_pnts[5] - 0.2 * alpha[5D[K
alpha * tangents[1]
                self._control_pnts[3] = 2 * self._control_pnts[4] - self._c[7D[K
self._control_pnts[5]

    @staticmethod
    def distance(p1, p2):
        """Compute the distance between two 3D points.

        > *Input arguments*

        * `p1` (*type:* list of `float` or `numpy.array`): Point 1
        * `p2` (*type:* list of `float` or `numpy.array`): Point 2

        > *Returns*

        Distance between points as a `float`
        """
        p1 = np.array(p1)
        p2 = np.array(p2)

        assert p1.size == 3 and p2.size == 3, \
            'Both input points must be three elements'        
        return np.sqrt(np.sum((p2 - p1)**2))

    @staticmethod
    def generate_cubic_curve(pnts):
        """Generate cubic Bezier curve segments from a list of points.

        > *Input arguments*

        * `pnts` (*type:* list of `float` or of `numpy.array`): List of poi[3D[K
points

        > *Returns*

        List of `BezierCurve` segments
        """
        assert isinstance(pnts, list), 'List of points is invalid'
        tangents = [np.zeros(3) for _ in range(len(pnts))]

        lengths = [rclpy.time.Client().get_clock().now().nanoseconds + (1.0[4D[K
(1.0 / 1000000) * (lengths[i + 1] - lengths[i]) for i in range(len(pnts) - [K
1)]
        lengths = [0] + lengths
        # Initial vector of parametric variables for the curve
        u = [l / np.sum(lengths) for l in np.cumsum(lengths)]
        delta_u = lambda k: u[k] - u[k - 1]
        delta_q = lambda k: pnts[k] - pnts[k - 1]
        lamb_k = lambda k: delta_q(k) / delta_u(k)
        alpha_k = lambda k: delta_u(k) / (delta_u(k) + delta_u(k + 1))
        normal_k = lambda k: ( ((pnts[k + 1] - pnts[k]) / (u[k + 1] - u[k])[5D[K
u[k])) - ((pnts[k] - pnts[k - 1]) / (u[k] - u[k - 1])) ) / (u[k + 1] - u[k [K
- 1])

        for i in range(1, len(u) - 1):            
            tangents[i] = (1 - alpha_k(i)) * lamb_k(i) + alpha_k(i) * lamb_[5D[K
lamb_k(i + 1)
            if i == 1:
                tangents[0] = 2 * lamb_k(i) - tangents[1]

        tangents[-1] = 2 * lamb_k(len(u) - 1) - tangents[-2]
                
        # Normalize tangent vectors
        for i in range(len(tangents)):
            tangents[i] /= np.linalg.norm(tangents[i])

        segments = list()
        # Generate the cubic Bezier curve segments
        for i in range(len(tangents) - 1):
            segments.append(BezierCurve([pnts[i], pnts[i + 1]], 3, [tangent[8D[K
[tangents[i], tangents[i + 1]]))

        return segments

    @staticmethod
    def generate_quintic_curve(pnts):
        """Generate quintic Bezier curve segments from a list of points.

        > *Input arguments*

        * `pnts` (*type:* list of `float` or of `numpy.array`): List of poi[3D[K
points

        > *Returns*

        List of `BezierCurve` segments
        """
        assert isinstance(pnts, list), 'List of points is invalid'
        tangents = [np.zeros(3) for _ in range(len(pnts))]
        normals = [np.zeros(3) for _ in range(len(pnts))]

        lengths = [rclpy.time.Client().get_clock().now().nanoseconds + (1.0[4D[K
(1.0 / 1000000) * (lengths[i + 1] - lengths[i]) for i in range(len(pnts) - [K
1)]
        lengths = [0] + lengths
        # Initial vector of parametric variables for the curve
        u = [l / np.sum(lengths) for l in np.cumsum(lengths)]
        delta_u = lambda k: u[k] - u[k - 1]
        delta_q = lambda k: pnts[k] - pnts[k - 1]
        lamb_k = lambda k: delta_q(k) / delta_u(k)
        alpha_k = lambda k: delta_u(k) / (delta_u(k) + delta_u(k + 1))
        normal_k = lambda k: ( ((pnts[k + 1] - pnts[k]) / (u[k + 1] - u[k])[5D[K
u[k])) - ((pnts[k] - pnts[k - 1]) / (u[k] - u[k - 1])) ) / (u[k + 1] - u[k [K
- 1])

        for i in range(1, len(u) - 1):
            tangents[i] = (1 - alpha_k(i)) * lamb_k(i) + alpha_k(i) * lamb_[5D[K
lamb_k(i + 1)
            normals[i] = normal_k(i)
            if i == 1:
                tangents[0] = 2 * lamb_k(i) - tangents[1]
                normals[0] = normal_k(i)

        tangents[-1] = 2 * lamb_k(len(u) - 1) - tangents[-2]
        normals[-1] = normal_k(len(u) - 3)
                
        # Normalize tangent vectors
        for i in range(len(tangents)):
            tangents[i] /= np.linalg.norm(tangents[i])
            normals[i] /= np.linalg.norm(normals[i])

        segments = list()
        # Generate the cubic Bezier curve segments
        for i in range(len(tangents) - 1):
            segments.append(BezierCurve([pnts[i], pnts[i + 1]], 5, 
                [tangents[i], tangents[i + 1]], 
                [normals[i], normals[i + 1]]))

        return segments

    def control_pnts(self):
        """Return the list of control points of the Bezier curve.

        > *Returns*

        List of 3D points as `list`
        """
        return self._control_pnts

    def interpolate(self, u):
        """Interpolate the Bezier curve using the input parametric variable[8D[K
variable `u`.

        > *Input arguments*

        * `u` (*type:* `float`): Curve parametric input in interval [0, 1]

        > *Returns*

        3D point from the Bezier curve as `numpy.array`
        """
        u = max(u, 0)
        u = min(u, 1)

        b = np.zeros(3)        
        for i in range(len(self._control_pnts)):
            b += self.compute_polynomial(self._order, i, u) * self._control[13D[K
self._control_pnts[i]
        return b

    def get_derivative(self, u, order=1):
        """Compute the derivative of the Bezier curve using the input param[5D[K
parametric 
        variable `u`.

        > *Input arguments*

        * `u` (*type:* `float`): Curve parametric input in interval [0, 1]
        * `order` (*type:* `int`, *default:* `1`): Order of the derivative

        > *Returns*

        `numpy.array`: 3D derivative value from the Bezier curve
        """
        u = max(u, 0)
        u = min(u, 1)

        b = np.zeros(3)        
        for i in range(len(self._control_pnts) - order):
            b = b + self._order * self.compute_polynomial(self._order - ord[3D[K
order, i, u) * \
                 (self._control_pnts[i + 1] - self._control_pnts[i])       [K
 
        return b

    def get_length(self):
        """Get length of the Bezier curve segment.

        > *Returns*

        `float`: Length of the curve
        """
        return self._order * np.linalg.norm(self._control_pnts[1] - self._c[7D[K
self._control_pnts[0])

    def compute_polynomial(self, n, i, u):
        """Compute the Bernstein polynomial

        $$
            \mathbf{B} = {n\choose i} (1 - u)^{(n - i)} u^{i}
        $$

        > *Input arguments*

        * `n` (*type:* `int`): Degree of the Bezier curve
        * `i` (*type:* `int`): Index of the control point
        * `u` (*type:* `float`): Parametric input of the curve in interval [K
[0, 1]

        > *Returns*

        `float`: Bernstein polynomial result
        """
        return self._get_binomial(n, i) * (1 - u)**(n - i) * u**i

    @staticmethod
    def _get_binomial(n, i):
        """Compute binomial function $\binom{n}{i}$ 

        > *Input arguments*

        * `n` (*type:* `int`)
        * `i` (*type:* `int`)
        """
        return factorial(n) / (factorial(i) * factorial(n - i))

Note: This code assumes that the input points are in the format of a list o[1D[K
or numpy array, and that the control points are computed using the formulas[8D[K
formulas provided.

