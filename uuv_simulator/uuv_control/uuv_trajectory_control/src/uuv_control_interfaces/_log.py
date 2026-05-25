# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.

import logging
import sys

LOGGER = None


def get_logger():
    global LOGGER

    if LOGGER is None:
        LOGGER = logging.getLogger(
            "uuv_control_interfaces"
        )

        out_hdlr = logging.StreamHandler(
            sys.stdout
        )

        out_hdlr.setFormatter(
            logging.Formatter(
                "%(asctime)s | "
                "%(levelname)s | "
                "%(module)s | "
                "%(message)s"
            )
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

    return LOGGER
