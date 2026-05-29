# salinity_data.py

import os
import pandas as pd
import matplotlib.pyplot as plt

from .simulation_data import (
    SimulationData,
    COLOR_RED
)


class SalinityData(SimulationData):
    LABEL = "salinity_sensor"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._unit = "ppt"

        self._recorded_data = {
            "salinity": []
        }

    def add_sample(self, time, salinity):
        self._time.append(time)
        self._recorded_data["salinity"].append(salinity)

    def get_as_dataframe(self, add_group_name=None):
        data = {
            "time": self._time,
            "salinity": self._recorded_data["salinity"]
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
            self._recorded_data["salinity"],
            color=COLOR_RED,
            linewidth=2
        )

        self.config_2dplot(
            ax,
            "",
            "Time [s]",
            f"Salinity [{self._unit}]"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "salinity.png")
        )

        plt.close(fig)
