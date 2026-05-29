# current_velocity_data.py

import os
import pandas as pd
import matplotlib.pyplot as plt

from .simulation_data import (
    SimulationData,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
)


class CurrentVelocityData(SimulationData):
    LABEL = "current_velocity"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._recorded_data = {
            "vel": []
        }

    def add_sample(self, time, velocity):
        self._time.append(time)
        self._recorded_data["vel"].append(velocity)

    def get_as_dataframe(self, add_group_name=None):
        data = {
            "time": self._time,
            "vx": [v[0] for v in self._recorded_data["vel"]],
            "vy": [v[1] for v in self._recorded_data["vel"]],
            "vz": [v[2] for v in self._recorded_data["vel"]]
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
            [v[0] for v in self._recorded_data["vel"]],
            color=COLOR_RED,
            label="u"
        )

        ax.plot(
            self._time,
            [v[1] for v in self._recorded_data["vel"]],
            color=COLOR_GREEN,
            label="v"
        )

        ax.plot(
            self._time,
            [v[2] for v in self._recorded_data["vel"]],
            color=COLOR_BLUE,
            label="w"
        )

        self.config_2dplot(
            ax,
            "",
            "Time [s]",
            "Velocity [m/s]"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "current_velocity.png")
        )

        plt.close(fig)
