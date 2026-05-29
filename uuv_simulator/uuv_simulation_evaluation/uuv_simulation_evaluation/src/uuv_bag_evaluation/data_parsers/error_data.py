# error_data.py

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from .simulation_data import (
    SimulationData,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
)


class ErrorData(SimulationData):
    LABEL = "error"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._recorded_data = {
            "position": [],
            "orientation": []
        }

    def add_sample(self, time, position_error, orientation_error):
        self._time.append(time)

        self._recorded_data["position"].append(position_error)
        self._recorded_data["orientation"].append(orientation_error)

    def get_as_dataframe(self, add_group_name=None):
        data = {
            "time": self._time,
            "pos_x": [x[0] for x in self._recorded_data["position"]],
            "pos_y": [x[1] for x in self._recorded_data["position"]],
            "pos_z": [x[2] for x in self._recorded_data["position"]],
            "roll": [x[0] for x in self._recorded_data["orientation"]],
            "pitch": [x[1] for x in self._recorded_data["orientation"]],
            "yaw": [x[2] for x in self._recorded_data["orientation"]]
        }

        if add_group_name:
            data["group"] = [add_group_name] * len(self._time)

        return pd.DataFrame(data)

    def plot(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        fig = self.get_figure()
        ax = fig.gca()

        ax.plot(
            self._time,
            [x[0] for x in self._recorded_data["position"]],
            color=COLOR_RED,
            label="X"
        )

        ax.plot(
            self._time,
            [x[1] for x in self._recorded_data["position"]],
            color=COLOR_GREEN,
            label="Y"
        )

        ax.plot(
            self._time,
            [x[2] for x in self._recorded_data["position"]],
            color=COLOR_BLUE,
            label="Z"
        )

        self.config_2dplot(
            ax,
            "Position Error",
            "Time [s]",
            "Error [m]"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "position_error.png")
        )

        plt.close(fig)
