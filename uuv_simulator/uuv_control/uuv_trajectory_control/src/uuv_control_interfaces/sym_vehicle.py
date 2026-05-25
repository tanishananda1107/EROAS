import numpy as np
from .vehicle import (
    Vehicle,
    cross_product_operator
)

try:
    import casadi
    CASADI_IMPORTED=True
except:
    CASADI_IMPORTED=False


class SymVehicle(Vehicle):

    def __init__(
        self,
        node,
        inertial_frame_id='world'
    ):

        super().__init__(
            node,
            inertial_frame_id
        )

        if not CASADI_IMPORTED:
            return

        self.eta=casadi.SX.sym(
            "eta",
            6
        )

        self.nu=casadi.SX.sym(
            "nu",
            6
        )

        self.CMatrix=casadi.SX.zeros(
            6,
            6
        )

        self.u=casadi.SX.sym(
            "u",
            6
        )

        self.DMatrix=(
            -casadi.diag(
                self._linear_damping
            )
        )

        self.nu_dot=casadi.solve(
            self._Mtotal,
            self.u
            -
            casadi.mtimes(
                self.DMatrix,
                self.nu
            )
        )
