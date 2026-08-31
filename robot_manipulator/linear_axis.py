""" 1-Axis linear base

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from typing import Optional
import numpy as np
from robot_manipulator.kinematics.trivial_kinematics_1d import TrivKins1D
from robot_manipulator.linear_axis_state import LinearAxisState
import config


class LinearAxis:

    def __init__(self, base_offset: np.ndarray):

        # World to base offset at zero joints.
        self.base_offset = base_offset

        # Linear axis kinematics.
        self.kinematics = TrivKins1D(self.base_offset, config.LINEAR_AXIS)

        # Linear axis internal state.
        self.state = LinearAxisState(self.kinematics)


    # ================================= Public =================================

    @staticmethod
    def request_joint_move(jnt_vec: np.ndarray) -> Optional[np.ndarray]:

        for idx in range(config.N_LIN_JNT):
            if not config.JOINT_LINEAR_LIMITS[f"JL{idx + 1}_MIN"] <= jnt_vec[idx] <= config.JOINT_LINEAR_LIMITS[f"JL{idx + 1}_MAX"]:
                print(f"Linear motor value out of range")
                return None

        # Send to queue
        return jnt_vec.copy()

    def move_joints(self, mcu: Optional[np.ndarray] = None, queued: Optional[np.ndarray] = None, planned: Optional[np.ndarray] = None):
        if mcu is not None:
            self.state.mcu.joint_state = mcu
        if queued is not None:
            self.state.mcu.joint_state = queued
        if planned is not None:
            self.state.mcu.joint_state = planned

    def reset(self):
        self.state.mcu.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float64)
        self.state.queued.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float64)
        self.state.planned.joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float64)
