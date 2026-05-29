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
#   - Removed: `import rospy`  → rclpy is not strictly needed here (no node),
#               but rclpy.logging is available if desired.
#   - Removed: `import rosbag` → replaced with rosbag2_py (ROS2 bag API).
#   - rosbag2_py reads bags in the new SQLite3 / MCAP storage format.
#     For legacy ROS1 bags (.bag) use `rosbags` (pip install rosbags) or
#     convert them first with `rosbags-convert --src foo.bag --dst foo/`.
#   - `from data_parsers import SimulationData`
#     → `from .data_parsers import SimulationData`  (relative, ROS2 style)
#   - `from uuv_trajectory_generator import ...`
#     stays the same if the package has been ported; adjust as needed.
#   - Gazebo Harmonic (gz-sim 8) publishes on gz-transport topics.
#     If you bridge with ros_gz_bridge the topic names and message types
#     change; update SimulationData parsers accordingly.

import logging
import sys
import numpy as np

# ---------------------------------------------------------------------------
# ROS2 bag reading
# ---------------------------------------------------------------------------
# rosbag2_py is the official ROS2 Python API (ships with ros-<distro>-rosbag2).
# For reading serialised messages you also need rclpy and the message packages.
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _ROSBAG2_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROSBAG2_AVAILABLE = False
    import warnings
    warnings.warn(
        'rosbag2_py / rclpy not found. Recording will not be able to open bags. '
        'Install the ros-<distro>-rosbag2-py package.',
        stacklevel=2,
    )

from .data_parsers import SimulationData  # relative import (ROS2 package)

# uuv_trajectory_generator must be ported separately to ROS2.
from uuv_trajectory_generator import TrajectoryGenerator, TrajectoryPoint


class Recording:
    """Singleton wrapper around a ROS2 bag file.

    Parameters
    ----------
    filename : str
        Path to the ROS2 bag *directory* (which contains metadata.yaml and
        the storage file, e.g. bag_0.db3 or bag_0.mcap).
        Pass the directory, not the .db3/.mcap file itself.
    storage_id : str
        Storage plugin identifier: ``'sqlite3'`` (default) or ``'mcap'``.
        Use ``'mcap'`` for bags recorded with the MCAP storage plugin.
    """

    __instance = None

    def __init__(self, filename: str, storage_id: str = 'sqlite3'):
        # ----------------------------------------------------------------
        # Logging (pure Python – no rospy.get_logger())
        # ----------------------------------------------------------------
        self._logger = logging.getLogger('read_rosbag2')
        if not self._logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(
                logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s')
            )
            handler.setLevel(logging.INFO)
            self._logger.addHandler(handler)
            self._logger.setLevel(logging.INFO)

        # ----------------------------------------------------------------
        # Open the bag
        # ----------------------------------------------------------------
        self._filename = filename

        if not _ROSBAG2_AVAILABLE:
            raise RuntimeError(
                'rosbag2_py is not installed. Cannot open bag: %s' % filename
            )

        # Storage options tell rosbag2_py which plugin to use.
        storage_options = rosbag2_py.StorageOptions(
            uri=filename,
            storage_id=storage_id,        # 'sqlite3' or 'mcap'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )

        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(storage_options, converter_options)

        # Build a topic → message-type map from the bag metadata.
        self._topic_types = {}
        for topic_metadata in self._reader.get_all_topics_and_types():
            self._topic_types[topic_metadata.name] = topic_metadata.type

        self.parsers = dict()
        self._is_init = False

        Recording.__instance = self

    # ------------------------------------------------------------------
    # Singleton access
    # ------------------------------------------------------------------
    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            # Cannot create a useful instance without a filename; callers
            # must instantiate Recording(filename) before calling get_instance().
            raise RuntimeError(
                'Recording has not been instantiated yet. '
                'Call Recording(filename) first.'
            )
        return cls.__instance

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def is_init(self):
        return self._is_init

    @property
    def topic_types(self):
        """Dict mapping topic name → ROS2 message type string."""
        return self._topic_types

    # ------------------------------------------------------------------
    # Parser initialisation
    # ------------------------------------------------------------------
    def init_parsers(self):
        """Instantiate all SimulationData parsers and pass the open reader."""
        self._logger.info('Initializing parsers')
        for parser_cls in SimulationData.get_all_parsers():
            self._logger.info('Initializing parser: %s', parser_cls.LABEL)
            # ROS2 change: pass the rosbag2_py reader instead of a
            # rosbag.Bag object; SimulationData subclasses must be
            # updated to use the rosbag2_py / rclpy deserialization API.
            self.parsers[parser_cls.LABEL] = parser_cls(
                self._reader,
                self._topic_types,
            )
        self._is_init = True

    # ------------------------------------------------------------------
    # Helpers for downstream code
    # ------------------------------------------------------------------
    def read_messages(self, topics=None):
        """Iterate over all messages in the bag (generator).

        Parameters
        ----------
        topics : list[str] | None
            If given, only yield messages on these topics.

        Yields
        ------
        tuple (topic, msg, timestamp_ns)
            *topic*         – topic name (str)
            *msg*           – deserialised ROS2 message object
            *timestamp_ns*  – nanosecond timestamp (int)
        """
        if not _ROSBAG2_AVAILABLE:
            return

        # Reset to the beginning of the bag.
        storage_filter = rosbag2_py.StorageFilter(topics=(topics or []))
        self._reader.set_filter(storage_filter)

        while self._reader.has_next():
            topic, data, timestamp_ns = self._reader.read_next()
            msg_type_str = self._topic_types.get(topic)
            if msg_type_str is None:
                continue
            try:
                msg_type = get_message(msg_type_str)
                msg = deserialize_message(data, msg_type)
            except Exception as exc:
                self._logger.warning(
                    'Could not deserialise message on %s: %s', topic, exc
                )
                continue
            yield topic, msg, timestamp_ns

    def get_trajectory_coord(self, tag):
        """Delegate to the recording's trajectory parser."""
        return self.parsers['trajectory'].get_coord(tag)

    def __del__(self):
        # rosbag2_py reader does not require an explicit close() call,
        # but release the reference to help GC.
        self._reader = None
