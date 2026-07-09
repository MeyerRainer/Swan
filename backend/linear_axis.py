"""
2-Axis linear base

"""
from config import *

import numpy as np
import typing


class LinearAxis:

    def __init__(self):

        # World origin to robot base at zero joints transform
        self._world2base_offs = np.array((0.1, 0.4, 0.), dtype=np.float32)

        # Temporary state
        self._temp_jnt_coords = np.zeros(N_LIN_JNT, dtype=np.float32)

        # State
        self._jnt_coords = np.zeros(N_LIN_JNT, dtype=np.float32)
        self._base_in_world = np.zeros(3, dtype=np.float32)

        self._update_temp_state(self._temp_jnt_coords)

    # def move_base(self, ops_vec: np.ndarray) -> np.ndarray | None:
    #     """ Move base to world coordinates
    #     @param ops_vec: 3-vector (X, Y, Z). New base frame coordinates w.r.t. world frame (meters)
    #     @return np.ndarray of joint coordinates, None if out of reach
    #     """
    #     jnt_vec = ops_vec - self._world2base_offs
    #
    #     # Zero out if DOF not available
    #     if 'X' in LINEAR_AXIS:
    #         self._temp_jnt_coords
    #     if 'Y' in LINEAR_AXIS:
    #         jnt_vec[1] = 0.
    #     if 'Z' in LINEAR_AXIS:
    #         jnt_vec[2] = 0.
    #
    #     for idx in range(N_LIN_JNT):
    #         if not MOTOR_LINEAR_LIMITS[f"M{idx+1}_MIN"] <= ops_vec[idx] <= MOTOR_LINEAR_LIMITS[f"M{idx+1}_MAX"]:
    #             print(f"Linear motor value out of range")
    #             return None
    #
    #     return ops_vec

    def _update_temp_state(self, jnt_vec: np.ndarray) -> None:
        self._temp_jnt_coords = jnt_vec.copy()


    # ================================= Public =================================

    @property
    def temp_jnt_coords(self):
        return self._temp_jnt_coords.copy()

    @property
    def jnt_coords(self):
        return self._jnt_coords.copy()

    def move_jnt(self, jnt_vec: np.ndarray) -> np.ndarray | None:

        for idx in range(N_LIN_JNT):
            if not MOTOR_LINEAR_LIMITS[f"ML{idx + 1}_MIN"] <= jnt_vec[idx] <= MOTOR_LINEAR_LIMITS[f"ML{idx + 1}_MAX"]:
                print(f"Linear motor value out of range")
                return None

        self._update_temp_state(jnt_vec)

        return jnt_vec.copy()

    def update_state(self, mot_vec):
        self._jnt_coords = mot_vec.copy()

        self._update_temp_state(self._jnt_coords)