import numpy as np

from .dp_controller_base import (
    DPControllerBase
)

from uuv_control_msgs.srv import (
    SetPIDParams,
    GetPIDParams
)


class DPPIDControllerBase(
    DPControllerBase
):

    def __init__(
        self,
        node,
        *args
    ):

        super().__init__(
            node,
            *args
        )

        self.node=node

        self._Kp=np.zeros((6,6))
        self._Ki=np.zeros((6,6))
        self._Kd=np.zeros((6,6))

        self._int=np.zeros(6)

        self._error_pose=np.zeros(6)

        kp=node.declare_parameter(
            "Kp",
            [0.0]*6
        ).value

        kd=node.declare_parameter(
            "Kd",
            [0.0]*6
        ).value

        ki=node.declare_parameter(
            "Ki",
            [0.0]*6
        ).value

        self._Kp=np.diag(kp)
        self._Kd=np.diag(kd)
        self._Ki=np.diag(ki)

        node.create_service(
            SetPIDParams,
            "set_pid_params",
            self.set_pid_params_callback
        )

        node.create_service(
            GetPIDParams,
            "get_pid_params",
            self.get_pid_params_callback
        )

    def update_pid(self):

        if not self.odom_is_init:
            return None

        self._int+=(
            0.5*
            (
                self.error_pose_euler+
                self._error_pose
            )*
            self._dt
        )

        self._error_pose=(
            self.error_pose_euler
        )

        return (
            np.dot(
                self._Kp,
                self.error_pose_euler
            )
            +
            np.dot(
                self._Kd,
                self._errors["vel"]
            )
            +
            np.dot(
                self._Ki,
                self._int
            )
        )

    def set_pid_params_callback(
        self,
        req,
        res
    ):

        self._Kp=np.diag(
            req.kp
        )

        self._Kd=np.diag(
            req.kd
        )

        self._Ki=np.diag(
            req.ki
        )

        res.success=True

        return res

    def get_pid_params_callback(
        self,
        req,
        res
    ):

        res.kp=[
            self._Kp[i,i]
            for i in range(6)
        ]

        res.kd=[
            self._Kd[i,i]
            for i in range(6)
        ]

        res.ki=[
            self._Ki[i,i]
            for i in range(6)
        ]

        return res
