# thruster_data.py - ROS2 / Gazebo Harmonic / gz-sim8
#
# Compatible with:
# - ROS 2 Humble / Jazzy
# - rosbag2_py
# - Gazebo Harmonic
# - gz-sim8

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


class ThrusterData(SimulationData):
    LABEL = "thrusters"

    def __init__(self, bag):
        super().__init__()

        self._recorded_data = {}

        self._prefix = None

        try:
            topics = bag.get_topic_names_and_types()

            for topic_name, topic_types in topics.items():
                if "thrusters" in topic_name:
                    idx = topic_name.find("thrusters") + len("thrusters")
                    self._prefix = topic_name[:idx]

                    self._logger.info(
                        f"Thruster topic prefix found <{self._prefix}>"
                    )
                    break

            if self._prefix is None:
                self._logger.warning("No thruster topics found")
                return

            # ----------------------------------------------------------
            # THRUST OUTPUT
            # ----------------------------------------------------------
            for i in range(16):
                topic = f"{self._prefix}/{i}/thrust"

                messages = bag.read_messages(topic)

                for msg, t in messages:

                    if i not in self._recorded_data:
                        self._recorded_data[i] = {
                            "thrust": {
                                "time": [],
                                "values": []
                            }
                        }

                    stamp = bag.get_time_in_seconds(msg)

                    self._recorded_data[i]["thrust"]["time"].append(stamp)
                    self._recorded_data[i]["thrust"]["values"].append(
                        float(msg.data)
                    )

                if i in self._recorded_data:
                    self._logger.info(f"{topic}=loaded")

            # ----------------------------------------------------------
            # INPUT COMMANDS
            # ----------------------------------------------------------
            for i in range(16):
                topic = f"{self._prefix}/{i}/input"

                messages = bag.read_messages(topic)

                for msg, t in messages:

                    if i not in self._recorded_data:
                        continue

                    if "input" not in self._recorded_data[i]:
                        self._recorded_data[i]["input"] = {
                            "time": [],
                            "values": []
                        }

                    stamp = bag.get_time_in_seconds(msg)

                    self._recorded_data[i]["input"]["time"].append(stamp)
                    self._recorded_data[i]["input"]["values"].append(
                        float(msg.data)
                    )

                if (
                    i in self._recorded_data and
                    "input" in self._recorded_data[i]
                ):
                    self._logger.info(f"{topic}=loaded")

        except Exception as e:
            self._logger.error(
                f"Error loading thruster data: {e}"
            )

    @property
    def n_thrusters(self):
        return len(self._recorded_data.keys())

    def get_input_data(self, idx):
        if idx not in self._recorded_data:
            return None

        if "input" not in self._recorded_data[idx]:
            return None

        return (
            self._recorded_data[idx]["input"]["time"],
            self._recorded_data[idx]["input"]["values"]
        )

    def get_thrust_data(self, idx):
        if idx not in self._recorded_data:
            return None

        return (
            self._recorded_data[idx]["thrust"]["time"],
            self._recorded_data[idx]["thrust"]["values"]
        )

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas as pd

            # ------------------------------------------------------
            # OUTPUT DATAFRAME
            # ------------------------------------------------------
            output_data = {
                f"{self.LABEL}_id": [],
                f"{self.LABEL}_output_time": [],
                f"{self.LABEL}_output_values": []
            }

            if add_group_name is not None:
                output_data["group"] = []

            for idx in self._recorded_data:

                n = len(
                    self._recorded_data[idx]["thrust"]["time"]
                )

                output_data[f"{self.LABEL}_id"] += [idx] * n

                output_data[
                    f"{self.LABEL}_output_time"
                ] += self._recorded_data[idx]["thrust"]["time"]

                output_data[
                    f"{self.LABEL}_output_values"
                ] += self._recorded_data[idx]["thrust"]["values"]

                if add_group_name is not None:
                    output_data["group"] += [add_group_name] * n

            df_output = pd.DataFrame(output_data)

            # ------------------------------------------------------
            # INPUT DATAFRAME
            # ------------------------------------------------------
            input_data = {
                f"{self.LABEL}_id": [],
                f"{self.LABEL}_input_time": [],
                f"{self.LABEL}_input_values": []
            }

            if add_group_name is not None:
                input_data["group"] = []

            for idx in self._recorded_data:

                if "input" not in self._recorded_data[idx]:
                    continue

                n = len(
                    self._recorded_data[idx]["input"]["time"]
                )

                input_data[f"{self.LABEL}_id"] += [idx] * n

                input_data[
                    f"{self.LABEL}_input_time"
                ] += self._recorded_data[idx]["input"]["time"]

                input_data[
                    f"{self.LABEL}_input_values"
                ] += self._recorded_data[idx]["input"]["values"]

                if add_group_name is not None:
                    input_data["group"] += [add_group_name] * n

            df_input = pd.DataFrame(input_data)

            return {
                "input": df_input,
                "output": df_output
            }

        except Exception as ex:
            self._logger.error(
                f"Error exporting dataframe: {ex}"
            )
            return None

    def plot(self, output_dir):

        if not os.path.isdir(output_dir):
            raise RuntimeError("Invalid output directory")

        if self.n_thrusters == 0:
            self._logger.warning("No thruster data available")
            return

        # --------------------------------------------------------------
        # INDIVIDUAL THRUSTER OUTPUTS
        # --------------------------------------------------------------
        fig, ax = plt.subplots(
            self.n_thrusters,
            1,
            figsize=(
                self._plot_configs["figsize"][0],
                self.n_thrusters *
                self._plot_configs["figsize"][1]
            )
        )

        if self.n_thrusters == 1:
            ax = [ax]

        max_y = 0.0

        for i in self._recorded_data:

            vals = np.abs(
                self._recorded_data[i]["thrust"]["values"]
            )

            max_y = max(max_y, np.max(vals))

        for i in self._recorded_data:

            t = self._recorded_data[i]["thrust"]["time"]
            y = self._recorded_data[i]["thrust"]["values"]

            ax[i].plot(
                t,
                y,
                linewidth=self._plot_configs["linewidth"],
                label=f"{i}"
            )

            ax[i].set_xlim(np.min(t), np.max(t))
            ax[i].set_ylim(-max_y, max_y)

            self.config_2dplot(
                ax=ax[i],
                title="",
                xlabel="Time [s]",
                ylabel=rf"$\tau_{i}$ [N]",
                legend_on=False
            )

        plt.tight_layout()

        fig.savefig(
            os.path.join(output_dir, "thrusts.pdf")
        )

        plt.close(fig)

        # --------------------------------------------------------------
        # ALL THRUSTERS
        # --------------------------------------------------------------
        fig_all = self.get_figure()
        ax_all = fig_all.gca()

        for i in self._recorded_data:

            ax_all.plot(
                self._recorded_data[i]["thrust"]["time"],
                self._recorded_data[i]["thrust"]["values"],
                linewidth=self._plot_configs["linewidth"],
                label=f"{i}"
            )

        ax_all.legend()

        self.config_2dplot(
            ax=ax_all,
            title="",
            xlabel="Time [s]",
            ylabel="Thrust Output [N]",
            legend_on=True
        )

        plt.tight_layout()

        fig_all.savefig(
            os.path.join(output_dir, "thrusts_all.pdf")
        )

        plt.close(fig_all)

        # --------------------------------------------------------------
        # INPUT COMMANDS
        # --------------------------------------------------------------
        fig_in = self.get_figure()
        ax_in = fig_in.gca()

        for i in self._recorded_data:

            if "input" not in self._recorded_data[i]:
                continue

            ax_in.plot(
                self._recorded_data[i]["input"]["time"],
                self._recorded_data[i]["input"]["values"],
                linewidth=self._plot_configs["linewidth"],
                label=f"{i}"
            )

        self.config_2dplot(
            ax=ax_in,
            title="",
            xlabel="Time [s]",
            ylabel="Input [rad/s]",
            legend_on=True
        )

        plt.tight_layout()

        fig_in.savefig(
            os.path.join(output_dir, "thruster_input.pdf")
        )

        plt.close(fig_in)
