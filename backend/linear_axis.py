"""
2-Axis linear base

"""

import numpy as np
import typing


class LinearAxis:

    def __init__(self):

        # World origin to robot base at zero joints transform
        self._offset = np.zeros(3)

        # World origin to base zero position transformation
        self.transform = np.eye(4)
        self.transform[:3, 3] = self._offset
        self.transform[:3, :3] = [[0., 1., 0.],
                                  [-1., 0., 0.],
                                  [0., 0., 1.]]


        # State
        self._jnt_coords = np.zeros(2)

        # Position offset at zero joints

        pass

    # Pose in base frame -> pose in world frame
    #

    def _f_kin(self, jnt_vec: np.ndarray) -> np.ndarray:
        """ jnt -> ops """
        return self._offset + np.array((jnt_vec[0], 0., 0.))  # Only x is variable


    def _i_kin(self, ops_vec: np.ndarray) -> np.ndarray:
        """  ops -> jnt """
        return np.array((ops_vec[0], 0., 0.)) - self._offset # Only x is variable

    def move_lin(self, ops_vec: np.ndarray) -> np.ndarray:
        pass

    def update_state(self):
        pass


    # ================================= Public =================================
    @property
    def position(self):
        return self._position.copy()
