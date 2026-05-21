from .fin_model import FinModel

import rclpy
import numpy as np
import tf2_ros
import yaml
import os

from rclpy.node import Node

from tf_quaternion.transformations import quaternion_matrix

from uuv_thrusters.models import Thruster

from uuv_auv_control_allocator.msg import AUVCommand

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class ActuatorManager(Node):

    MAX_FINS=4

    def __init__(self):

        super().__init__(
            "actuator_manager"
        )

        self.namespace = (
            self.get_namespace()
            .replace(
                "/",
                ""
            )
        )

        self.get_logger().info(
            f"Vehicle={self.namespace}"
        )

        self.tf_buffer = tf2_ros.Buffer()

        self.listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.base_link = (
            self.declare_parameter(
                "base_link",
                "base_link"
            )
            .value
        )

        self.thruster_config = (
            self.declare_parameter(
                "thruster_config",
                {}
            )
            .value
        )

        self.fin_config = (
            self.declare_parameter(
                "fin_config",
                {}
            )
            .value
        )

        if len(
            self.thruster_config
        )==0:

            raise RuntimeError(
                "thruster config missing"
            )

        if len(
            self.fin_config
        )==0:

            raise RuntimeError(
                "fin config missing"
            )

        self.fin_lower_limit = (
            self.fin_config.get(
                "lower_limit",
                -np.pi/2
            )
        )

        self.fin_upper_limit = (
            self.fin_config.get(
                "upper_limit",
                np.pi/2
            )
        )

        self.fins={}

        self.n_fins=0

        self.find_actuators()
            def find_actuators(self):

        base = (
            f"{self.namespace}/"
            f"{self.base_link}"
        )

        thruster_frame=(
            f"{self.namespace}/"
            f"{self.thruster_config['frame_base']}0"
        )

        trans = self.tf_buffer.lookup_transform(
            base,
            thruster_frame,
            rclpy.time.Time()
        )

        pos=np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])

        quat=np.array([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ])

        topic=(
            f"/{self.namespace}/"
            f"{self.thruster_config['topic_prefix']}"
            f"/0/"
            f"{self.thruster_config['topic_suffix']}"
        )

        self.thruster = Thruster.create_thruster(

            self.thruster_config[
                "conversion_fcn"
            ],

            0,

            topic,

            pos,

            quat,

            **self.thruster_config[
                "conversion_fcn_params"
            ]
        )

        for i in range(
            self.MAX_FINS
        ):

            try:

                frame=(
                    f"{self.namespace}/"
                    f"{self.fin_config['frame_base']}"
                    f"{i}"
                )

                trans=(
                    self.tf_buffer
                    .lookup_transform(
                        base,
                        frame,
                        rclpy.time.Time()
                    )
                )

                pos=np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                quat=np.array([
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ])

                topic=(
                    f"/{self.namespace}/"
                    f"{self.fin_config['topic_prefix']}"
                    f"/{i}/"
                    f"{self.fin_config['topic_suffix']}"
                )

                self.fins[i]=FinModel(
                    self,
                    i,
                    pos,
                    quat,
                    topic
                )

            except Exception:

                break

        self.n_fins=len(
            self.fins
        )

        return True
            def publish_commands(
        self,
        command
    ):

        self.thruster.publish_command(
            command[0]
        )

        for i in range(
            self.n_fins
        ):

            self.fins[i].publish_command(
                command[i+1]
            )
