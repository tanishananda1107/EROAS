# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on
# an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

"""
ROS2 trajectory generator logger utilities.
"""

import logging
import sys

LOGGER = None


def get_logger():
    """
    Return logger instance for trajectory generator.

    Returns
    -------
    logging.Logger
        Configured logger object
    """

    global LOGGER

    if LOGGER is None:

        LOGGER = logging.getLogger(
            "uuv_trajectory_generator"
        )

        # Prevent duplicate handlers
        if not LOGGER.handlers:

            out_hdlr = logging.StreamHandler(
                sys.stdout
            )

            formatter = logging.Formatter(
                "%(asctime)s | "
                "%(levelname)s | "
                "%(module)s | "
                "%(message)s"
            )

            out_hdlr.setFormatter(
                formatter
            )

            out_hdlr.setLevel(
                logging.INFO
            )

            LOGGER.addHandler(
                out_hdlr
            )

        LOGGER.setLevel(
            logging.INFO
        )

        LOGGER.propagate = False

    return LOGGER
