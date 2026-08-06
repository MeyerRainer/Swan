""" 1-Axis linear base

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import config
from backend.kinematics.trivial_kinematics_1d import TrivKins1D
from backend.linear_axis_state import LinearAxisState

import numpy as np


class LinearAxis:

    def __init__(self):

        self.BASE_OFFSET = np.array([0.1, 0.3, 0.0], dtype=np.float64)
        self.kinematics = TrivKins1D(self.BASE_OFFSET, config.LINEAR_AXIS)

        self.state = LinearAxisState(self.kinematics)


    # ================================= Public =================================

    @staticmethod
    def move_jnt(jnt_vec: np.ndarray) -> np.ndarray | None:

        for idx in range(config.N_LIN_JNT):
            if not config.MOTOR_LINEAR_LIMITS[f"ML{idx + 1}_MIN"] <= jnt_vec[idx] <= config.MOTOR_LINEAR_LIMITS[f"ML{idx + 1}_MAX"]:
                print(f"Linear motor value out of range")
                return None

        # Send to queue
        return jnt_vec.copy()

    def update_state(self, jnt_vec: np.ndarray):
        self.state.mcu.joint_state = jnt_vec.copy()

    def reset(self):
        self.state.mcu.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float32)
        self.state.queued.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float32)
        self.state.planned.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float32)
