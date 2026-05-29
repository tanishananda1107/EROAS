# concentration_sensor_data.py

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from .simulation_data import (
    SimulationData,
    COLOR_RED
)


class ConcentrationSensorData(SimulationData):
    LABEL = "concentration_sensor"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._recorded_data = {
            "conc": [],
            "pos": []
        }

    def add_sample(self, time, concentration, position):
        self._time.append(time)

        self._recorded_data["conc"].append(concentration)
        self._recorded_data["pos"].append(position)

    def get_as_dataframe(self, add_group_name=None):
        data = {
            "time": self._time,
            "concentration": self._recorded_data["conc"],
            "x": [p[0] for p in self._recorded_data["pos"]],
            "y": [p[1] for p in self._recorded_data["pos"]],
            "z": [p[2] for p in self._recorded_data["pos"]]
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
            self._recorded_data["conc"],
            color=COLOR_RED,
            linewidth=2
        )

        self.config_2dplot(
            ax,
            "",
            "Time [s]",
            "Concentration"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "particle_concentration.png")
        )

        plt.close(fig)
