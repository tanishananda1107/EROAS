# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS2 / Gazebo Harmonic (gz-sim 8) migration notes:
#   - Removed: `import tf.transformations as trans`  (ROS1-only)
#     Not used directly in this file; tf math lives in error.py now.
#   - Changed: `from .recording import Recording`  (was bare import)
#   - Changed: `from .error import ErrorSet`        (was bare import)
#   - `yaml.load(...)` → `yaml.safe_load(...)` (security best-practice;
#     bare yaml.load raises a warning in modern PyYAML).
#   - `from __future__ import print_function` removed (Python 3 only).

from __future__ import annotations

import os
import sys
import logging
import numpy as np
import yaml
from copy import deepcopy

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)

from .recording import Recording
from .error import ErrorSet
from .metrics import KPI        # must be ported separately to ROS2

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception as e:
    print('Cannot use LaTeX configuration with matplotlib, message=' + str(e))


class Evaluation(object):
    """High-level evaluation wrapper for a single ROS2 bag.

    Parameters
    ----------
    filename : str
        Path to the ROS2 bag *directory* (see Recording for details).
    output_dir : str
        Directory where results (KPIs, plots) will be written.
    time_offset : float
        Simulation time (seconds) to skip before computing KPIs.
    storage_id : str
        rosbag2_py storage plugin: ``'sqlite3'`` (default) or ``'mcap'``.
    """

    def __init__(
        self,
        filename: str,
        output_dir: str = '.',
        time_offset: float = 0.0,
        storage_id: str = 'sqlite3',
    ):
        # ----------------------------------------------------------------
        # Logging (pure Python – no rospy)
        # ----------------------------------------------------------------
        self._logger = logging.getLogger('run_evaluation')
        if not self._logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(
                logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s')
            )
            handler.setLevel(logging.INFO)
            self._logger.addHandler(handler)
            self._logger.setLevel(logging.INFO)

        self._logger.info('Opening bag: %s', filename)

        # ROS2: pass storage_id so Recording picks the right plugin.
        self.recording = Recording(filename, storage_id=storage_id)
        self.recording.init_parsers()

        # ----------------------------------------------------------------
        # Error set
        # ----------------------------------------------------------------
        self._error_set = ErrorSet.get_instance()
        if self._error_set is None:
            self._logger.error('Error set has not been correctly initialized')
            raise RuntimeError('Error set has not been correctly initialized')

        self._error_set.compute_errors()

        # ----------------------------------------------------------------
        # Output directory
        # ----------------------------------------------------------------
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=%s', output_dir)
            raise NotADirectoryError('Invalid output directory: %s' % output_dir)

        # ----------------------------------------------------------------
        # Time offset
        # ----------------------------------------------------------------
        if time_offset >= 0.0:
            self._time_offset = time_offset
        else:
            self._logger.error('Invalid time offset, setting time offset to zero')
            self._time_offset = 0.0

        self._logger.info(
            'Time offset for KPI evaluation [s]=%s', str(self._time_offset)
        )

        self._output_dir = output_dir

        # ----------------------------------------------------------------
        # KPIs – build the full set by default
        # ----------------------------------------------------------------
        self._kpis: list[dict] = []
        for kpi_tag in KPI.get_all_kpi_tags():
            if KPI.get_kpi_target(kpi_tag) == 'error':
                for error_tag in self._error_set.get_tags():
                    self._kpis.append(
                        dict(func=KPI.get_kpi(kpi_tag, error_tag), value=0.0)
                    )
            else:
                self._kpis.append(
                    dict(func=KPI.get_kpi(kpi_tag), value=0.0)
                )

        self._cost_fcn_terms: dict = {}

        # Initial KPI computation
        self.compute_kpis()

    # ------------------------------------------------------------------

    def __del__(self):
        if self.recording is not None:
            del self.recording

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def error_set(self):
        return self._error_set

    # ------------------------------------------------------------------
    # Cost function
    # ------------------------------------------------------------------

    def calc_cost_fcn(self) -> float:
        cost = 0.0
        for tag in self._cost_fcn_terms:
            cost += self._cost_fcn_terms[tag] * self.get_kpi(tag)
        return cost

    def save_cost_fcn_config(self, filename: str) -> None:
        try:
            with open(filename, 'w') as cost_file:
                yaml.dump(self._cost_fcn_terms, cost_file, default_flow_style=False)
        except Exception as e:
            self._logger.error('Error exporting cost function config, message=%s', e)

    def load_cost_fcn(self, filename: str) -> bool:
        if not os.path.isfile(filename):
            self._logger.error('Invalid filename, file=%s', filename)
            return False
        # ROS1 bug fix: original code opened in 'w' mode (write), which
        # would truncate the file.  Corrected to 'r' (read) mode.
        with open(filename, 'r') as fcn_file:
            # ROS2: yaml.safe_load instead of yaml.load (security best-practice)
            fcn = yaml.safe_load(fcn_file)
        try:
            for item in fcn:
                self.add_cost_fcn_term(item, fcn[item])
                self._logger.info(
                    'Cost function term (tag, weight): (%s, %.4f)', item, fcn[item]
                )
        except Exception as e:
            self._logger.error('Error loading cost function configuration: %s', e)
            return False
        return True

    def add_cost_fcn_term(self, kpi: str, weight: float) -> bool:
        if weight <= 0:
            self._logger.error('Weight must be a positive value')
            return False
        if not self.has_kpi(kpi):
            self._logger.error('KPI tag is invalid, tag=%s', kpi)
            return False
        if kpi in self._cost_fcn_terms:
            self._logger.error('KPI already added to cost function, tag=%s', kpi)
            return False
        self._cost_fcn_terms[kpi] = weight
        return True

    # ------------------------------------------------------------------
    # KPI helpers
    # ------------------------------------------------------------------

    def has_kpi(self, tag: str) -> bool:
        for kpi in self._kpis:
            if tag == kpi['func'].full_tag:
                return True
        return False

    def set_kpis_from_file(self, filename: str) -> None:
        assert os.path.isfile(filename), 'Invalid evaluation configuration file'
        assert filename.endswith('.yaml') or filename.endswith('.yml'), \
            'Configuration file must be YAML'
        with open(filename, 'r') as config_file:
            # ROS2: yaml.safe_load
            config = yaml.safe_load(config_file)
        self.set_kpis(config)

    def set_kpis(self, config: list) -> None:
        assert isinstance(config, list), 'Invalid configuration structure for KPIs'
        kpi_tags = KPI.get_all_kpi_tags()

        self._kpis = []
        for item in config:
            if item['func'] not in kpi_tags:
                self._logger.error('Invalid KPI tag, value=%s', item)
            else:
                self._logger.info('KPI created: %s', item['func'])
                if 'args' in item:
                    self._kpis.append(
                        dict(func=KPI.get_kpi(item['func'], item['args']), value=0.0)
                    )
                    self._logger.info('\tArguments: %s', str(item['args']))
                else:
                    self._kpis.append(
                        dict(func=KPI.get_kpi(item['func']), value=0.0)
                    )

    def get_kpis(self) -> dict:
        kpis = {}
        for kpi in self._kpis:
            item = kpi['func']
            try:
                kpis[item.full_tag] = float(item.kpi_value)
            except Exception:
                kpis[item.full_tag] = -1000.0
        return kpis

    def get_kpi(self, tag: str):
        for kpi in self._kpis:
            if kpi['func'].full_tag == tag:
                return kpi['value']
        return None

    def compute_kpis(self) -> None:
        for i, kpi in enumerate(self._kpis):
            try:
                self._kpis[i]['value'] = kpi['func'].compute()
            except Exception as e:
                self._logger.error(
                    'Error calculating KPI %s, message=%s',
                    kpi['func'].full_tag, str(e),
                )

    def print_kpis(self) -> None:
        for item in self._kpis:
            # ROS1 bug fix: used item['value'] consistently (original mixed
            # item['func'].full_tag with item['value'] via print).
            print('%s = %s' % (item['func'].full_tag, str(item['value'])))

    def get_trajectory_coord(self, tag: str):
        return self.recording.get_trajectory_coord(tag)

    def export_to_txt(self, tag: str, output_dir: str) -> None:
        """Stub – implement per-tag CSV/TXT export here."""
        pass

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    def save_dataframes(self, output_dir: str | None = None) -> None:
        if output_dir is not None and not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=%s', output_dir)
            raise NotADirectoryError('Invalid output directory: %s' % output_dir)

        output_path = self._output_dir if output_dir is None else output_dir

        try:
            for tag in self.recording.parsers:
                self._logger.info('Reading data frame for %s', tag)
                df = self.recording.parsers[tag].get_as_dataframe()

                if df is None:
                    continue

                data_dir = os.path.join(output_path, 'data')
                os.makedirs(data_dir, exist_ok=True)

                if isinstance(df, dict):
                    for k in df:
                        out_file = os.path.join(data_dir, '%s_%s.yaml' % (tag, k))
                        with open(out_file, 'w') as f:
                            yaml.dump(df[k].to_dict(), f, default_flow_style=False)
                else:
                    out_file = os.path.join(data_dir, '%s.yaml' % tag)
                    with open(out_file, 'w') as f:
                        yaml.dump(df.to_dict(), f, default_flow_style=False)

                self._logger.info('Data frame <%s> stored: %s', tag, out_file)

        except Exception as e:
            self._logger.error('Error storing dataframes, message=%s', str(e))

    def save_evaluation(self, output_dir: str | None = None) -> None:
        if output_dir is not None and not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=%s', output_dir)
            raise NotADirectoryError('Invalid output directory: %s' % output_dir)

        self.save_kpis(output_dir)

        for tag in self.recording.parsers:
            self.recording.parsers[tag].plot(self._output_dir)

        self._logger.info('Evaluation stored!')

    def save_kpis(self, output_dir: str | None = None) -> None:
        if output_dir is not None and not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=%s', output_dir)
            raise NotADirectoryError('Invalid output directory: %s' % output_dir)

        try:
            output_path = self._output_dir if output_dir is None else output_dir

            kpis: dict = {}
            kpi_labels: dict = {}

            for kpi in self._kpis:
                item = kpi['func']
                try:
                    value = float(item.kpi_value)
                except Exception:
                    value = 0.0
                kpis[item.full_tag] = value
                kpi_labels[item.full_tag] = item.label

            kpi_file_path = os.path.join(output_path, 'computed_kpis.yaml')
            with open(kpi_file_path, 'w') as kpi_file:
                yaml.dump(kpis, kpi_file, default_flow_style=False)
            self._logger.info('Calculated KPIs stored in <%s>', kpi_file_path)

            label_file_path = os.path.join(output_path, 'kpi_labels.yaml')
            with open(label_file_path, 'w') as label_file:
                yaml.dump(kpi_labels, label_file, default_flow_style=False)
            self._logger.info('KPI labels stored in <%s>', label_file_path)

        except Exception as e:
            self._logger.error('Error storing KPIs file, message=%s', str(e))
