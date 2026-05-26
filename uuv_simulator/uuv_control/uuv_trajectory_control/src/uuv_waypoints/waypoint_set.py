# Copyright (c) 2016-2019 The UUV Simulator Authors.
# ROS2 Jazzy conversion

import os
import yaml
import numpy as np

from rclpy.clock import Clock

from .waypoint import Waypoint

from uuv_control_msgs.msg import WaypointSet as WaypointSetMessage

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointSet:
    """
    ROS2 waypoint set implementation
    """

    FINAL_WAYPOINT_COLOR = [1.0, 0.5737, 0.0]

    OK_WAYPOINT = [
        31.0 / 255.0,
        106.0 / 255.0,
        226.0 / 255.0
    ]

    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(
        self,
        scale=0.1,
        inertial_frame_id="world",
        max_surge_speed=None
    ):

        assert inertial_frame_id in [
            "world",
            "world_ned"
        ]

        self._waypoints = []

        self._violates_constraint = False

        self._scale = scale

        self._inertial_frame_id = (
            inertial_frame_id
        )

        self._max_surge_speed = (
            max_surge_speed
        )

    def __str__(self):

        if self.num_waypoints == 0:
            return "Waypoint set empty"

        msg = "=====================\n"

        msg += "Waypoint list\n"

        msg += "=====================\n"

        for wp in self._waypoints:

            msg += str(wp)

            msg += "\n---\n"

        msg += (
            f"Num waypoints="
            f"{self.num_waypoints}\n"
        )

        msg += (
            f"Inertial frame="
            f"{self._inertial_frame_id}\n"
        )

        return msg

    @property
    def num_waypoints(self):

        return len(self._waypoints)

    @property
    def x(self):

        return [wp.x for wp in self._waypoints]

    @property
    def y(self):

        return [wp.y for wp in self._waypoints]

    @property
    def z(self):

        return [wp.z for wp in self._waypoints]

    @property
    def is_empty(self):

        return len(
            self._waypoints
        ) == 0

    @property
    def inertial_frame_id(self):

        return self._inertial_frame_id

    @inertial_frame_id.setter
    def inertial_frame_id(
        self,
        frame_id
    ):

        assert frame_id in [
            "world",
            "world_ned"
        ]

        self._inertial_frame_id = (
            frame_id
        )

    def clear_waypoints(self):

        self._waypoints = []

    def get_waypoint(
        self,
        index
    ):

        if (
            index < 0
            or
            index >= len(
                self._waypoints
            )
        ):

            return None

        return self._waypoints[
            index
        ]

    def add_waypoint(
        self,
        waypoint,
        add_to_beginning=False
    ):

        if len(
            self._waypoints
        ):

            if (
                self._waypoints[-1]
                != waypoint
            ):

                if not add_to_beginning:

                    self._waypoints.append(
                        waypoint
                    )

                else:

                    self._waypoints = (
                        [waypoint]
                        +
                        self._waypoints
                    )

            else:

                print(
                    "Repeated waypoint"
                )

                return False

        else:

            if not add_to_beginning:

                self._waypoints.append(
                    waypoint
                )

            else:

                self._waypoints = (
                    [waypoint]
                )

        return True

    def add_waypoint_from_msg(
        self,
        msg
    ):

        wp = Waypoint()

        wp.from_message(msg)

        return self.add_waypoint(
            wp
        )

    def remove_waypoint(
        self,
        waypoint
    ):

        new_list = []

        for wp in self._waypoints:

            if wp == waypoint:

                continue

            new_list.append(
                wp
            )

        self._waypoints = new_list

    def get_start_waypoint(
        self
    ):

        if len(
            self._waypoints
        ):

            return self._waypoints[0]

        return None

    def get_last_waypoint(
        self
    ):

        if len(
            self._waypoints
        ):

            return self._waypoints[-1]

        return None

    def set_constraint_status(
        self,
        index,
        flag
    ):

        if (
            index < 0
            or
            index >= len(
                self._waypoints
            )
        ):

            return False

        self._waypoints[
            index
        ].violates_constraint = flag

        return True

    def read_from_file(
        self,
        filename
    ):

        if not os.path.isfile(
            filename
        ):

            print(
                "File invalid"
            )

            return False

        try:

            self.clear_waypoints()

            with open(
                filename,
                "r"
            ) as wp_file:

                wps = yaml.safe_load(
                    wp_file
                )

            if isinstance(
                wps,
                list
            ):

                for data in wps:

                    wp = Waypoint(
                        x=data["point"][0],
                        y=data["point"][1],
                        z=data["point"][2],
                        max_forward_speed=data[
                            "max_forward_speed"
                        ],
                        heading_offset=data[
                            "heading"
                        ],
                        use_fixed_heading=data[
                            "use_fixed_heading"
                        ]
                    )

                    self.add_waypoint(
                        wp
                    )

            else:

                self._inertial_frame_id = (
                    wps[
                        "inertial_frame_id"
                    ]
                )

                for data in wps[
                    "waypoints"
                ]:

                    wp = Waypoint(
                        x=data["point"][0],
                        y=data["point"][1],
                        z=data["point"][2],
                        max_forward_speed=data[
                            "max_forward_speed"
                        ],
                        heading_offset=data[
                            "heading"
                        ],
                        use_fixed_heading=data[
                            "use_fixed_heading"
                        ],
                        inertial_frame_id=
                        self._inertial_frame_id
                    )

                    self.add_waypoint(
                        wp
                    )

        except Exception as e:

            print(e)

            return False

        return True

    def export_to_file(
        self,
        path,
        filename
    ):

        output = dict(
            inertial_frame_id=
            self._inertial_frame_id,

            waypoints=[]
        )

        for wp in self._waypoints:

            output[
                "waypoints"
            ].append(

                dict(

                    point=[
                        float(wp.x),
                        float(wp.y),
                        float(wp.z)
                    ],

                    max_forward_speed=
                    float(
                        wp.max_forward_speed
                    ),

                    heading=float(
                        wp.heading_offset
                    ),

                    use_fixed_heading=
                    bool(
                        wp.using_heading_offset
                    )
                )
            )

        with open(

            os.path.join(
                path,
                filename
            ),

            "w"

        ) as f:

            yaml.dump(
                output,
                f,
                default_flow_style=False
            )

        return True

    def to_message(
        self
    ):

        msg = WaypointSetMessage()

        msg.header.stamp = (
            Clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self._inertial_frame_id
        )

        msg.waypoints = []

        for wp in self._waypoints:

            wp_msg = wp.to_message()

            wp_msg.header.frame_id = (
                self._inertial_frame_id
            )

            msg.waypoints.append(
                wp_msg
            )

        return msg

    def from_message(
        self,
        msg
    ):

        self.clear_waypoints()

        self.inertial_frame_id = (
            msg.header.frame_id
        )

        for wp in msg.waypoints:

            self.add_waypoint_from_msg(
                wp
            )

    def dist_to_waypoint(
        self,
        pos,
        index=0
    ):

        wp = self.get_waypoint(
            index
        )

        if wp is None:

            return None

        return wp.dist(
            pos
        )

    def to_path_marker(
        self,
        clear=False
    ):

        path = Path()

        path.header.stamp = (
            Clock()
            .now()
            .to_msg()
        )

        path.header.frame_id = (
            self._inertial_frame_id
        )

        if (
            self.num_waypoints > 1
            and
            not clear
        ):

            for wp in self._waypoints:

                pose = PoseStamped()

                pose.header.stamp = (
                    Clock()
                    .now()
                    .to_msg()
                )

                pose.header.frame_id = (
                    self._inertial_frame_id
                )

                pose.pose.position.x = wp.x

                pose.pose.position.y = wp.y

                pose.pose.position.z = wp.z

                path.poses.append(
                    pose
                )

        return path

    def to_marker_list(
        self,
        clear=False
    ):

        markers = MarkerArray()

        t = (
            Clock()
            .now()
            .to_msg()
        )

        if (
            self.num_waypoints == 0
            or
            clear
        ):

            marker = Marker()

            marker.header.stamp = t

            marker.header.frame_id = (
                self._inertial_frame_id
            )

            marker.action = (
                Marker.DELETEALL
            )

            markers.markers.append(
                marker
            )

            return markers

        for i, wp in enumerate(
            self._waypoints
        ):

            marker = Marker()

            marker.header.stamp = t

            marker.header.frame_id = (
                self._inertial_frame_id
            )

            marker.id = i

            marker.type = (
                Marker.SPHERE
            )

            marker.action = (
                Marker.ADD
            )

            marker.pose.position.x = wp.x

            marker.pose.position.y = wp.y

            marker.pose.position.z = wp.z

            marker.scale.x = (
                self._scale
            )

            marker.scale.y = (
                self._scale
            )

            marker.scale.z = (
                self._scale
            )

            marker.color.a = 1.0

            if (
                wp ==
                self.get_last_waypoint()
            ):

                color = (
                    wp.get_final_color()
                )

            else:

                color = (
                    wp.get_color()
                )

            marker.color.r = color[0]

            marker.color.g = color[1]

            marker.color.b = color[2]

            markers.markers.append(
                marker
            )

        return markers

    def generate_circle(

        self,

        radius,

        center,

        num_points,

        max_forward_speed,

        theta_offset=0.0,

        heading_offset=0.0,

        append=False
    ):

        if not append:

            self.clear_waypoints()

        step = (
            2*np.pi
            /
            num_points
        )

        for i in range(
            num_points
        ):

            angle = (
                i*step
                +
                theta_offset
            )

            x = (
                np.cos(angle)
                *
                radius
                +
                center.x
            )

            y = (
                np.sin(angle)
                *
                radius
                +
                center.y
            )

            z = center.z

            wp = Waypoint(

                x,

                y,

                z,

                max_forward_speed,

                heading_offset
            )

            self.add_waypoint(
                wp
            )

        return True

    def generate_helix(

        self,

        radius,

        center,

        num_points,

        max_forward_speed,

        delta_z,

        num_turns,

        theta_offset=0.0,

        heading_offset=0.0,

        append=False
    ):

        if not append:

            self.clear_waypoints()

        total = (
            2*np.pi*num_turns
        )

        step_angle = (
            total
            /
            num_points
        )

        step_z = (
            delta_z
            /
            num_points
        )

        for i in range(
            num_points
        ):

            angle = (
                theta_offset
                +
                i*step_angle
            )

            x = (
                radius
                *
                np.cos(angle)
                +
                center.x
            )

            y = (
                radius
                *
                np.sin(angle)
                +
                center.y
            )

            z = (
                center.z
                +
                i*step_z
            )

            wp = Waypoint(

                x,

                y,

                z,

                max_forward_speed,

                heading_offset
            )

            self.add_waypoint(
                wp
            )

        return True
