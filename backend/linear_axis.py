""" 1-Axis linear base

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass

import config
from backend.kinematics.trivial_kinematics_1d import TrivKins1D

import numpy as np

@dataclass
class LinearAxisState:

    # TODO: Define axis in parameter?
    def __init__(self, kinematics):

        # World origin to robot base at zero joints transform
        self._kinematics = kinematics

        self._joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float64)        # Joint vector (=motor vector).
        self._jacobian = np.array([[1., 0., 0.]], dtype=np.float64).T     # Jacobian (constant). Column vector.
        self._position = np.zeros(3, dtype=np.float64)

        # Initialize state
        self.joint_state = self._joint_state

    @property
    def joint_state(self) -> np.ndarray:
        return self._joint_state.copy()

    @property
    def jacobian(self) -> np.ndarray:
        return self._jacobian.copy()

    @property
    def position(self) -> np.ndarray:
        return self._position.copy()

    @joint_state.setter
    def joint_state(self, jnt_vec: np.ndarray):
        self._joint_state = jnt_vec.copy()
        self._position = self._kinematics.forward(jnt_vec)


class State:

    def __init__(self, kinematics):

        self.mcu = LinearAxisState(kinematics=kinematics)
        self.queued = LinearAxisState(kinematics=kinematics)
        self.planned = LinearAxisState(kinematics=kinematics)


class LinearAxis:

    def __init__(self):

        self.BASE_OFFSET = np.array([0.1, 0.3, 0.0], dtype=np.float64)
        self.kinematics = TrivKins1D(self.BASE_OFFSET, config.LINEAR_AXIS)

        self.state = State(self.kinematics)


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
