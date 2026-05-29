# wrench_perturbation_data.py - ROS2 / Gazebo Harmonic

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt

from simulation_data import (
    SimulationData,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
)


class WrenchPerturbationData(SimulationData):

    LABEL = "wrench_perturbation"

    def __init__(self, bag):
        super().__init__()

        self._time = []

        self._recorded_data = {
            "force": [],
            "torque": []
        }

        self._topic_name = None

        try:
            topics = bag.get_topic_names_and_types()

            for topic_name, topic_types in topics.items():

                msg_type = topic_types[0]

                if (
                    "wrench_perturbation" in topic_name and
                    "geometry_msgs/msg/WrenchStamped" in msg_type
                ):
                    self._topic_name = topic_name

                    self._logger.info(
                        f"Wrench perturbation topic found "
                        f"<{topic_name}>"
                    )
                    break

            if self._topic_name is None:
                self._logger.warning(
                    "No wrench perturbation topic found"
                )
                return

            messages = bag.read_messages(self._topic_name)

            for msg, t in messages:

                stamp = bag.get_time_in_seconds(msg)

                self._time.append(stamp)

                self._recorded_data["force"].append([
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z
                ])

                self._recorded_data["torque"].append([
                    msg.wrench.torque.x,
                    msg.wrench.torque.y,
                    msg.wrench.torque.z
                ])

            self._logger.info(
                f"{self._topic_name}=loaded"
            )

        except Exception as e:
            self._logger.error(
                f"Error loading wrench perturbation data: {e}"
            )

    @property
    def disturbances(self):
        return (
            self._time,
            self._recorded_data["force"],
            self._recorded_data["torque"]
        )

    def plot(self, output_dir):

        if not os.path.isdir(output_dir):
            raise RuntimeError("Invalid output directory")

        fig = self.get_figure()

        # ----------------------------------------------------------
        # FORCE
        # ----------------------------------------------------------
        ax1 = fig.add_subplot(211)

        ax1.plot(
            self._time,
            [f[0] for f in self._recorded_data["force"]],
            color=COLOR_RED,
            label=r"$F_X$"
        )

        ax1.plot(
            self._time,
            [f[1] for f in self._recorded_data["force"]],
            color=COLOR_GREEN,
            label=r"$F_Y$"
        )

        ax1.plot(
            self._time,
            [f[2] for f in self._recorded_data["force"]],
            color=COLOR_BLUE,
            label=r"$F_Z$"
        )

        self.config_2dplot(
            ax=ax1,
            title="",
            xlabel="Time [s]",
            ylabel="Force [N]",
            legend_on=True
        )

        # ----------------------------------------------------------
        # TORQUE
        # ----------------------------------------------------------
        ax2 = fig.add_subplot(212)

        ax2.plot(
            self._time,
            [t[0] for t in self._recorded_data["torque"]],
            color=COLOR_RED,
            label=r"$\tau_X$"
        )

        ax2.plot(
            self._time,
            [t[1] for t in self._recorded_data["torque"]],
            color=COLOR_GREEN,
            label=r"$\tau_Y$"
        )

        ax2.plot(
            self._time,
            [t[2] for t in self._recorded_data["torque"]],
            color=COLOR_BLUE,
            label=r"$\tau_Z$"
        )

        self.config_2dplot(
            ax=ax2,
            title="",
            xlabel="Time [s]",
            ylabel="Torque [Nm]",
            legend_on=True
        )

        plt.tight_layout()

        fig.savefig(
            os.path.join(
                output_dir,
                "disturbance_wrenches.pdf"
            )
        )

        plt.close(fig)
