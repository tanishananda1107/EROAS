# simulation_data.py

import logging
import sys
import matplotlib.pyplot as plt

try:
    import seaborn as sns

    plt.style.use("seaborn-v0_8-ticks")

    COLOR_RED = sns.xkcd_rgb["pale red"]
    COLOR_GREEN = sns.xkcd_rgb["medium green"]
    COLOR_BLUE = sns.xkcd_rgb["denim blue"]

except Exception:
    COLOR_RED = "#d62728"
    COLOR_GREEN = "#2ca02c"
    COLOR_BLUE = "#1f77b4"


class SimulationData:
    LABEL = ""

    def __init__(self, topic_name=None, message_type=None, prefix=None):
        self._logger = logging.getLogger(self.LABEL)

        if not self._logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(
                logging.Formatter(
                    "%(asctime)s | %(levelname)s | %(module)s | %(message)s"
                )
            )
            handler.setLevel(logging.INFO)

            self._logger.addHandler(handler)
            self._logger.setLevel(logging.INFO)

        self._topic_name = topic_name
        self._message_type = message_type
        self._prefix = prefix

        self._time = []
        self._recorded_data = {}

        self._output_dir = "/tmp"

        self._plot_configs = dict(
            figsize=[12, 6],
            linewidth=2,
            label_fontsize=16,
            title_fontsize=18,
            tick_labelsize=14,
            xlim=None,
            ylim=None,
            zlim=None,
            labelpad=10,
            legend=dict(
                loc="upper right",
                fontsize=14
            )
        )

    @staticmethod
    def get_all_parsers():
        return SimulationData.__subclasses__()

    @staticmethod
    def get_all_labels():
        return [p.LABEL for p in SimulationData.get_all_parsers()]

    def get_figure(self, n_rows=1):
        return plt.figure(
            figsize=(
                self._plot_configs["figsize"][0],
                n_rows * self._plot_configs["figsize"][1]
            )
        )

    def config_2dplot(self, ax, title, xlabel, ylabel, legend_on=True):
        if title:
            ax.set_title(
                title,
                fontsize=self._plot_configs["title_fontsize"]
            )

        ax.grid(True, alpha=0.3)

        ax.tick_params(
            axis="both",
            labelsize=self._plot_configs["tick_labelsize"]
        )

        ax.set_xlabel(
            xlabel,
            fontsize=self._plot_configs["label_fontsize"]
        )

        ax.set_ylabel(
            ylabel,
            fontsize=self._plot_configs["label_fontsize"]
        )

        if legend_on:
            legend = ax.legend(
                fancybox=True,
                framealpha=1,
                loc=self._plot_configs["legend"]["loc"],
                fontsize=self._plot_configs["legend"]["fontsize"]
            )

            legend.get_frame().set_facecolor("white")

    def get_data(self):
        return self._time, self._recorded_data

    def get_as_dataframe(self, add_group_name=None):
        raise NotImplementedError()

    def plot(self, output_dir):
        raise NotImplementedError()
