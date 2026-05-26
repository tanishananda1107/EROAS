import numpy as np

from uuv_control_msgs.msg import Waypoint as WaypointMessage


class Waypoint:

    FINAL_WAYPOINT_COLOR = [1.0, 131.0 / 255.0, 0.0]
    OK_WAYPOINT = [31.0 / 255.0, 106.0 / 255.0, 226.0 / 255.0]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(
        self,
        x=0.0,
        y=0.0,
        z=0.0,
        max_forward_speed=0.0,
        heading_offset=0.0,
        use_fixed_heading=False,
        inertial_frame_id="world",
        radius_acceptance=0.0,
    ):

        assert inertial_frame_id in ["world", "world_ned"]

        self._x = x
        self._y = y
        self._z = z

        self._max_forward_speed = max_forward_speed

        self._heading_offset = heading_offset
        self._heading = 0.0

        self._use_fixed_heading = use_fixed_heading

        self._violates_constraint = False

        self._radius_acceptance = radius_acceptance

        self._inertial_frame_id = inertial_frame_id

    def __eq__(self, other):
        return (
            self._x == other.x
            and self._y == other.y
            and self._z == other.z
        )

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):

        msg = (
            f"(x,y,z)=({self._x:.2f}, "
            f"{self._y:.2f}, {self._z:.2f})\n"
        )

        msg += (
            f"Max forward speed={self._max_forward_speed:.2f}\n"
        )

        if self._use_fixed_heading:
            msg += (
                f"Heading="
                f"{self._heading_offset*180/np.pi:.2f}\n"
            )

        return msg

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def pos(self):
        return np.array([self._x, self._y, self._z])

    @pos.setter
    def pos(self, new_pos):

        if isinstance(new_pos, list):
            assert len(new_pos) == 3

        elif isinstance(new_pos, np.ndarray):
            assert new_pos.shape == (3,)

        self._x = new_pos[0]
        self._y = new_pos[1]
        self._z = new_pos[2]

    @property
    def max_forward_speed(self):
        return self._max_forward_speed

    @max_forward_speed.setter
    def max_forward_speed(self, vel):
        self._max_forward_speed = vel

    @property
    def heading(self):
        return self._heading

    @heading.setter
    def heading(self, angle):
        self._heading = angle

    @property
    def heading_offset(self):
        return self._heading_offset

    @property
    def violates_constraint(self):
        return self._violates_constraint

    @violates_constraint.setter
    def violates_constraint(self, flag):
        self._violates_constraint = flag

    @property
    def radius_of_acceptance(self):
        return self._radius_acceptance

    @radius_of_acceptance.setter
    def radius_of_acceptance(self, radius):

        assert radius >= 0

        self._radius_acceptance = radius

    def get_color(self):

        if self._violates_constraint:
            return self.FAILED_WAYPOINT

        return self.OK_WAYPOINT

    def get_final_color(self):

        return self.FINAL_WAYPOINT_COLOR

    def from_message(self, msg):

        self._inertial_frame_id = msg.header.frame_id

        if len(self._inertial_frame_id) == 0:
            self._inertial_frame_id = "world"

        self._x = msg.point.x
        self._y = msg.point.y
        self._z = msg.point.z

        self._max_forward_speed = msg.max_forward_speed

        self._use_fixed_heading = msg.use_fixed_heading

        self._heading_offset = msg.heading_offset

        self._radius_acceptance = msg.radius_of_acceptance

    def to_message(self):

        wp = WaypointMessage()

        wp.point.x = self._x
        wp.point.y = self._y
        wp.point.z = self._z

        wp.max_forward_speed = self._max_forward_speed

        wp.use_fixed_heading = self._use_fixed_heading

        wp.heading_offset = self._heading_offset

        wp.radius_of_acceptance = self._radius_acceptance

        wp.header.frame_id = self._inertial_frame_id

        return wp

    def dist(self, pos):

        return np.sqrt(
            (self._x-pos[0])**2 +
            (self._y-pos[1])**2 +
            (self._z-pos[2])**2
        )

    def calculate_heading(self, target):

        dy = target.y-self.y
        dx = target.x-self.x

        return np.arctan2(dy, dx)
