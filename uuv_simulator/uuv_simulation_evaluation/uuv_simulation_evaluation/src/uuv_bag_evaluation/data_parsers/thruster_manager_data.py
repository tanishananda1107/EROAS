# thruster_manager_data.py - ROS2 / Gazebo Harmonic

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


class ThrusterManagerData(SimulationData):
    LABEL = "thruster_manager"

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
                    "thruster_manager" in topic_name and
                    "geometry_msgs/msg/WrenchStamped" in msg_type
                ):
                    self._topic_name = topic_name

                    self._logger.info(
                        f"Thruster manager topic found <{topic_name}>"
                    )
                    break

            if self._topic_name is None:
                self._logger.warning(
                    "No thruster manager topic found"
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
                f"Error loading thruster manager data: {e}"
            )

    def get_as_dataframe(self, add_group_name=None):

        try:
            import pandas as pd

            data = {
                f"{self.LABEL}_time": self._time,

                f"{self.LABEL}_force_x":
                    [x[0] for x in self._recorded_data["force"]],

                f"{self.LABEL}_force_y":
                    [x[1] for x in self._recorded_data["force"]],

                f"{self.LABEL}_force_z":
                    [x[2] for x in self._recorded_data["force"]],

                f"{self.LABEL}_torque_x":
                    [x[0] for x in self._recorded_data["torque"]],

                f"{self.LABEL}_torque_y":
                    [x[1] for x in self._recorded_data["torque"]],

                f"{self.LABEL}_torque_z":
                    [x[2] for x in self._recorded_data["torque"]],
            }

            if add_group_name is not None:
                data["group"] = [
                    add_group_name
                    for _ in range(len(self._time))
                ]

            return pd.DataFrame(data)

        except Exception as ex:
            self._logger.error(
                f"Error exporting dataframe: {ex}"
            )
            return None

    def plot(self, output_dir):

        if not os.path.isdir(output_dir):
            raise RuntimeError("Invalid output directory")

        fig, ax = plt.subplots(
            2,
            1,
            figsize=(
                self._plot_configs["figsize"][0],
                2 * self._plot_configs["figsize"][1]
            )
        )

        # ----------------------------------------------------------
        # FORCES
        # ----------------------------------------------------------
        ax[0].plot(
            self._time,
            [f[0] for f in self._recorded_data["force"]],
            color=COLOR_RED,
            label="X"
        )

        ax[0].plot(
            self._time,
            [f[1] for f in self._recorded_data["force"]],
            color=COLOR_GREEN,
            label="Y"
        )

        ax[0].plot(
            self._time,
            [f[2] for f in self._recorded_data["force"]],
            color=COLOR_BLUE,
            label="Z"
        )

        self.config_2dplot(
            ax=ax[0],
            title="",
            xlabel="Time [s]",
            ylabel="Force [N]",
            legend_on=True
        )

        # ----------------------------------------------------------
        # TORQUES
        # ----------------------------------------------------------
        ax[1].plot(
            self._time,
            [t[0] for t in self._recorded_data["torque"]],
            color=COLOR_RED,
            label="K"
        )

        ax[1].plot(
            self._time,
            [t[1] for t in self._recorded_data["torque"]],
            color=COLOR_GREEN,
            label="M"
        )

        ax[1].plot(
            self._time,
            [t[2] for t in self._recorded_data["torque"]],
            color=COLOR_BLUE,
            label="N"
        )

        self.config_2dplot(
            ax=ax[1],
            title="",
            xlabel="Time [s]",
            ylabel="Torque [Nm]",
            legend_on=True
        )

        plt.tight_layout()

        fig.savefig(
            os.path.join(
                output_dir,
                "thruster_manager_input.pdf"
            )
        )

        plt.close(fig)
