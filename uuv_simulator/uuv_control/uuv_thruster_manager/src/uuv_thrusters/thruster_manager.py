import numpy as np
import yaml
import xml.etree.ElementTree as etree

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf_transformations import quaternion_matrix

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException

from .models import Thruster


class ThrusterManager(Node):

    MAX_THRUSTERS = 16

    def __init__(self):

        super().__init__(
            "thruster_manager"
        )

        self._ready = False

        self.namespace = self.get_namespace()

        if self.namespace != "/":

            if not self.namespace.endswith("/"):

                self.namespace += "/"

        self.declare_parameter(
            "thruster_manager",
            {}
        )

        self.config = self.get_parameter(
            "thruster_manager"
        ).value

        if len(self.config) == 0:

            raise RuntimeError(
                "Thruster manager config missing"
            )

        robot_description_param = (
            self.namespace +
            "robot_description"
        )

        self.use_robot_descr = False

        self.axes = {}

        try:

            robot_desc = self.get_parameter(
                robot_description_param
            ).value

            self.use_robot_descr = True

            self.parse_urdf(
                robot_desc
            )

        except:

            pass

        if self.config["update_rate"] < 0:

            self.config[
                "update_rate"
            ] = 50

        self.base_link_ned_to_enu = None

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        try:

            target = (
                self.namespace +
                "base_link"
            ).replace(
                "//",
                "/"
            )

            source = (
                self.namespace +
                "base_link_ned"
            ).replace(
                "//",
                "/"
            )

            tf_msg = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(
                    seconds=1
                )
            )

            q = tf_msg.transform.rotation

            self.base_link_ned_to_enu = quaternion_matrix(

                [
                    q.x,
                    q.y,
                    q.z,
                    q.w
                ]

            )[0:3,0:3]

            self.get_logger().info(

                str(
                    self.base_link_ned_to_enu
                )

            )

        except Exception as e:

            self.get_logger().info(

                str(e)

            )

        self.output_dir = None

        self.declare_parameter(
            "output_dir",
            ""
        )

        output_dir = self.get_parameter(
            "output_dir"
        ).value

        if output_dir != "":

            p = Path(
                output_dir
            )

            if p.exists():

                self.output_dir = output_dir

        self.n_thrusters = 0

        self.thrusters = []

        self.thrust = None

        self.configuration_matrix = None

        self.declare_parameter(
            "tam",
            []
        )

        tam = self.get_parameter(
            "tam"
        ).value

        if len(tam) > 0:

            self.configuration_matrix = np.array(
                tam
            )

            self.n_thrusters = (
                self.configuration_matrix.shape[1]
            )

            params = self.config[
                "conversion_fcn_params"
            ]

            conv = self.config[
                "conversion_fcn"
            ]

            for i in range(
                self.n_thrusters
            ):

                topic = (

                    self.config[
                        "thruster_topic_prefix"
                    ]

                    +

                    str(i)

                    +

                    self.config[
                        "thruster_topic_suffix"
                    ]

                )

                thruster = Thruster.create_thruster(

                    conv,

                    self,

                    i,

                    topic,

                    None,

                    None,

                    **params

                )

                self.thrusters.append(
                    thruster
                )

            self.thrust = np.zeros(
                self.n_thrusters
            )

        if not self.update_tam():

            raise RuntimeError(
                "No thrusters found"
            )

        self.inverse_configuration_matrix = np.linalg.pinv(

            self.configuration_matrix

        )

        if self.output_dir is not None:

            with open(

                str(
                    Path(
                        self.output_dir
                    ) /
                    "TAM.yaml"
                ),

                "w"

            ) as f:

                yaml.dump(

                    {

                        "tam":

                        self.configuration_matrix.tolist()

                    },

                    f

                )

        self._ready = True

        self.get_logger().info(
            "ThrusterManager ready"
        )

    def parse_urdf(
        self,
        urdf
    ):

        root = etree.fromstring(
            urdf
        )

        for joint in root.findall(
            "joint"
        ):

            if joint.get(
                "type"
            ) == "fixed":

                continue

            axis = joint.find(
                "axis"
            )

            child = joint.find(
                "child"
            )

            xyz = axis.get(
                "xyz"
            ).split()

            link = child.get(
                "link"
            )

            self.axes[
                link
            ] = np.array(

                [

                    float(
                        xyz[0]
                    ),

                    float(
                        xyz[1]
                    ),

                    float(
                        xyz[2]
                    ),

                    0.0

                ]

            )

    def update_tam(
        self,
        recalculate=False
    ):

        if (

            self.configuration_matrix
            is not None

            and

            not recalculate

        ):

            self._ready = True

            return True

        self._ready = False

        base = (

            self.namespace +

            self.config[
                "base_link"
            ]

        )

        self.thrusters = []

        for i in range(

            self.MAX_THRUSTERS

        ):

            frame = (

                self.namespace +

                self.config[
                    "thruster_frame_base"
                ]

                +

                str(i)

            )

            try:

                tf_msg = self.tf_buffer.lookup_transform(

                    base,

                    frame,

                    rclpy.time.Time(),

                    timeout=Duration(
                        seconds=1
                    )

                )

                pos = [

                    tf_msg.transform.translation.x,

                    tf_msg.transform.translation.y,

                    tf_msg.transform.translation.z

                ]

                quat = [

                    tf_msg.transform.rotation.x,

                    tf_msg.transform.rotation.y,

                    tf_msg.transform.rotation.z,

                    tf_msg.transform.rotation.w

                ]

                topic = (

                    self.config[
                        "thruster_topic_prefix"
                    ]

                    +

                    str(i)

                    +

                    self.config[
                        "thruster_topic_suffix"
                    ]

                )

                thruster = Thruster.create_thruster(

                    self.config[
                        "conversion_fcn"
                    ],

                    self,

                    i,

                    topic,

                    pos,

                    quat,

                    self.axes.get(
                        frame,
                        np.array(
                            [
                                1,
                                0,
                                0,
                                0
                            ]
                        )
                    ),

                    **self.config[
                        "conversion_fcn_params"
                    ]

                )

                self.thrusters.append(
                    thruster
                )

            except TransformException:

                break

        if len(
            self.thrusters
        ) == 0:

            return False

        self.n_thrusters = len(
            self.thrusters
        )

        self.thrust = np.zeros(
            self.n_thrusters
        )

        self.configuration_matrix = np.zeros(

            (

                6,

                self.n_thrusters

            )

        )

        for i in range(
            self.n_thrusters
        ):

            self.configuration_matrix[:,i] = (

                self.thrusters[i]
                .tam_column

            )

        self.inverse_configuration_matrix = (

            np.linalg.pinv(

                self.configuration_matrix

            )

        )

        self._ready = True

        return True

    def command_thrusters(
        self
    ):

        if self.thrust is None:

            return

        for i in range(
            self.n_thrusters
        ):

            self.thrusters[
                i
            ].publish_command(

                self.thrust[i]

            )

    def publish_thrust_forces(

        self,

        control_forces,

        control_torques,

        frame_id=None

    ):

        if not self._ready:

            return

        gen_forces = np.hstack(

            (

                control_forces,

                control_torques

            )

        )

        self.thrust = (

            self.compute_thruster_forces(

                gen_forces

            )

        )

        self.command_thrusters()

    def compute_thruster_forces(
        self,
        gen_forces
    ):

        thrust = (

            self.inverse_configuration_matrix.dot(

                gen_forces

            )

        )

        max_thrust = self.config[
            "max_thrust"
        ]

        for i in range(
            self.n_thrusters
        ):

            if abs(
                thrust[i]
            ) > max_thrust:

                thrust[i] = (

                    np.sign(
                        thrust[i]
                    )

                    *

                    max_thrust

                )

        return thrust
