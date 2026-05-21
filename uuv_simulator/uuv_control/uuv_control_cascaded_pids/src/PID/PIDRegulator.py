# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.

import numpy as np


class PIDRegulator:
    """
    Generic PID regulator supporting
    scalar and vector inputs.
    """

    def __init__(self, p, i, d, sat):

        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0.0
        self.prev_err = None
        self.prev_t = None

    def __str__(self):

        msg = "PID controller"

        msg += f"\n\tp={self.p}"
        msg += f"\n\ti={self.i}"
        msg += f"\n\td={self.d}"
        msg += f"\n\tsat={self.sat}"

        return msg

    def reset(self):

        self.integral = 0.0

        self.prev_err = None

        self.prev_t = None

    def regulate(self, err, t):

        err = np.asarray(err)

        if self.prev_t is None:

            self.prev_t = t

            self.prev_err = err

            return np.zeros_like(err)

        dt = t - self.prev_t

        if dt <= 0.0:

            return np.zeros_like(err)

        derr_dt = (
            err - self.prev_err
        ) / dt

        self.integral += (
            0.5 *
            (err + self.prev_err)
            * dt
        )

        u = (
            self.p * err
            +
            self.i * self.integral
            +
            self.d * derr_dt
        )

        self.prev_err = err

        self.prev_t = t

        norm_u = np.linalg.norm(u)

        if norm_u > self.sat:

            u = (
                self.sat *
                u /
                norm_u
            )

            self.integral = 0.0

        return u

