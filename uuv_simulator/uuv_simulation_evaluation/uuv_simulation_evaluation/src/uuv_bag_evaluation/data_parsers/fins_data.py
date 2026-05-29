# fins_data.py

import os
import pandas as pd
import matplotlib.pyplot as plt

from .simulation_data import SimulationData


class FinsData(SimulationData):
    LABEL = "fins"

    def __init__(self, bag_reader=None):
        super().__init__()

        self._recorded_data = {}

    def add_fin_sample(self, fin_id, time, angle):
        if fin_id not in self._recorded_data:
            self._recorded_data[fin_id] = {
                "time": [],
                "angle": []
            }

        self._recorded_data[fin_id]["time"].append(time)
        self._recorded_data[fin_id]["angle"].append(angle)

    def get_as_dataframe(self, add_group_name=None):
        rows = []

        for fin_id in self._recorded_data:
            for t, angle in zip(
                self._recorded_data[fin_id]["time"],
                self._recorded_data[fin_id]["angle"]
            ):
                row = {
                    "fin_id": fin_id,
                    "time": t,
                    "angle": angle
                }

                if add_group_name:
                    row["group"] = add_group_name

                rows.append(row)

        return pd.DataFrame(rows)

    def plot(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        fig = self.get_figure()
        ax = fig.gca()

        for fin_id in self._recorded_data:
            ax.plot(
                self._recorded_data[fin_id]["time"],
                self._recorded_data[fin_id]["angle"],
                label=f"Fin {fin_id}"
            )

        self.config_2dplot(
            ax,
            "",
            "Time [s]",
            "Fin Angle [rad]"
        )

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "fin_angles.png")
        )

        plt.close(fig)
