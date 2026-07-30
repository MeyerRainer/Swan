""" Trivial kinematics for 1D linear axis

Author: Rainer Meyer, r.meyer494@gmail.com

"""

import numpy as np
from typing import Tuple, List, Union

class TrivKins1D:

    def __init__(self, base_offset: np.ndarray, axis: List[str]):
        """
        :param base_offset: Base pose in world at zero joints. SE3 / Pose
        """

        self.base_offset: np.ndarray = base_offset
        self.axis: List[str] = axis

    def forward(self, jnt_vec: np.ndarray) -> np.ndarray:
        """ Forward kinematics. Map joint value to base position
        :param jnt_vec: 1-vector
        :return base_coords: 3-vector
        """
        base_coords = self.base_offset.copy()

        for idx, axis in enumerate(self.axis):
            if axis == 'X':
                base_coords[0] += jnt_vec[idx]
            elif axis == 'Y':
                base_coords[1] += jnt_vec[idx]
            elif axis == 'Z':
                base_coords[2] += jnt_vec[idx]

        print(f"Forward kins: Joints in: {jnt_vec}\tPosition out: {base_coords}")
        return base_coords

    def inverse(self, target: np.ndarray) -> np.ndarray:
        """ Inverse kinematics. Map base position to joint value.
        :param target: 1-3 vector
        :return: joint solution, 1-vector
        """

        jnt_vec = np.zeros(len(self.axis), dtype=np.float64)

        for idx, axis in enumerate(self.axis):
            if axis == 'X':
                jnt_vec[0] += target[idx] - self.base_offset[0]
            elif axis == 'Y':
                jnt_vec[1] += target[idx] - self.base_offset[1]
            elif axis == 'Z':
                jnt_vec[2] += target[idx] - self.base_offset[2]

        return jnt_vec
