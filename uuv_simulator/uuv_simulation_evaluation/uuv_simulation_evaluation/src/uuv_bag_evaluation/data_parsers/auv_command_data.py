# auv_command_data.py

import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .simulation_data import (
    SimulationData,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
)


class AUVCommandData(SimulationData):
    LABEL = "auv_control"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._recorded_data = {
            "force": [],
            "torque": [],
            "surge_speed": []
        }

    def add_sample(self, time, force, torque, surge_speed):
        self._time.append(time)

        self._recorded_data["force"].append(force)
        self._recorded_data["torque"].append(torque)
        self._recorded_data["surge_speed"].append(surge_speed)

    def get_as_dataframe(self, add_group_name=None):
        if not self._recorded_data["force"]:
            return None

        data = {
            "time": self._time,
            "force_x": [x[0] for x in self._recorded_data["force"]],
            "force_y": [x[1] for x in self._recorded_data["force"]],
            "force_z": [x[2] for x in self._recorded_data["force"]],
            "torque_x": [x[0] for x in self._recorded_data["torque"]],
            "torque_y": [x[1] for x in self._recorded_data["torque"]],
            "torque_z": [x[2] for x in self._recorded_data["torque"]],
            "surge_speed": self._recorded_data["surge_speed"]
        }

        if add_group_name:
            data["group"] = [add_group_name] * len(self._time)

        return pd.DataFrame(data)

    def plot(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        fig, ax = plt.subplots(3, 1, figsize=(12, 16))

        ax[0].plot(
            self._time,
            [f[0] for f in self._recorded_data["force"]],
            color=COLOR_RED,
            label="Fx"
        )

        ax[0].plot(
            self._time,
            [f[1] for f in self._recorded_data["force"]],
            color=COLOR_GREEN,
            label="Fy"
        )

        ax[0].plot(
            self._time,
            [f[2] for f in self._recorded_data["force"]],
            color=COLOR_BLUE,
            label="Fz"
        )

        self.config_2dplot(
            ax[0],
            "",
            "Time [s]",
            "Force [N]"
        )

        ax[1].plot(
            self._time,
            [t[0] for t in self._recorded_data["torque"]],
            color=COLOR_RED,
            label="Tx"
        )

        ax[1].plot(
            self._time,
            [t[1] for t in self._recorded_data["torque"]],
            color=COLOR_GREEN,
            label="Ty"
        )

        ax[1].plot(
            self._time,
            [t[2] for t in self._recorded_data["torque"]],
            color=COLOR_BLUE,
            label="Tz"
        )

        self.config_2dplot(
            ax[1],
            "",
            "Time [s]",
            "Torque [Nm]"
        )

        ax[2].plot(
            self._time,
            self._recorded_data["surge_speed"],
            color=COLOR_BLUE,
            label="Surge"
        )

        self.config_2dplot(
            ax[2],
            "",
            "Time [s]",
            "Surge Speed [m/s]"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "auv_control_input.png")
        )

        plt.close(fig)
