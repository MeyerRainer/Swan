""" Reduced ZYZ Euler angle class for robot manipulator.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from __future__ import annotations
import numpy as np
import math


class ZYZEuler:

    def __init__(self, zyz: np.ndarray, condition: float = 0.):

        self.zyz: np.ndarray = np.asarray(zyz, dtype=np.float64)

        self.condition: float = condition

    @classmethod
    def identity(cls):
        return ZYZEuler(np.zeros(3, dtype=np.float64))

    @classmethod
    def from_rot_mat(cls, rot_mat: np.ndarray, phi_prev: float = 0, psi_prev: float = 0, flip: bool = False) -> ZYZEuler:
        """ Conversion from rotation matrix to ZYZ-Euler angles
        :param rot_mat: np.array, rotation matrix
        :param phi_prev: float, previous value for first Z-rotation.
            Needed in case of a singularity to determine phi and psi (first and last Z-angles)
        :param psi_prev: float, previous value for last Z-rotation
        :param flip: bool. True results in positive Y-rotation, False results in negative Y-rotation. Verify this!
        """
        zyz: np.ndarray = np.zeros(3, dtype=np.float64)

        z_i = rot_mat[0][2]  # i-component of Z-unit vector
        z_j = rot_mat[1][2]  # j-component of Z-unit vector
        z_hypo = math.sqrt(z_i * z_i + z_j * z_j)

        # Singularity if Z is vertical -> phi and psi dependent
        eps = 1e-3
        if z_hypo < eps:  # Singularity

            if flip:
                nu = math.atan2(-z_hypo, rot_mat[2][2])
            else:
                nu = math.atan2(z_hypo, rot_mat[2][2])
            zyz[1] = nu

            # Total rotation, sum of phi and psi
            gamma = math.atan2(rot_mat[1][0], rot_mat[0][0])

            # Solve phi and psi as least squares solution
            zyz[0] = 0.5 * (gamma + phi_prev - psi_prev)  # Phi / J4
            zyz[2] = 0.5 * (gamma - phi_prev + psi_prev)  # Psi / J6

            return cls(zyz, condition=z_hypo)

        # Fully defined rotation: Nu = [0, pi] or [-pi, 0] when flipped
        if flip:
            phi = math.atan2(-z_j, -z_i)
            nu = math.atan2(-z_hypo, rot_mat[2][2])
            psi = math.atan2(-rot_mat[2][1], rot_mat[2][0])
        else:
            phi = math.atan2(z_j, z_i)
            nu = math.atan2(z_hypo, rot_mat[2][2])
            psi = math.atan2(rot_mat[2][1], -rot_mat[2][0])

        zyz[0] = phi  # Around Z
        zyz[1] = nu  # Around new Y
        zyz[2] = psi  # Around new Z

        return cls(zyz, condition=z_hypo)

    @property
    def array(self) -> np.ndarray:
        return self.zyz.copy()

    @property
    def rot_mat(self) -> np.ndarray:
        """ Conversion from ZYZ-Euler angles  to a rotation matrix
        @param zyz: np.array, ZYZ-angles
        @param degrees: bool, degrees or radians
        """
        z1, y1, z2 = self.zyz[0], self.zyz[1], self.zyz[2]

        rot_alpha = np.array([
            [np.cos(z1), -np.sin(z1), 0.],
            [np.sin(z1), np.cos(z1), 0.],
            [0., 0., 1.]], dtype=np.float64)

        rot_nu = np.array([
            [np.cos(y1), 0, np.sin(y1)],
            [0, 1, 0],
            [-np.sin(y1), 0, np.cos(y1)]], dtype=np.float64)

        rot_psi = np.array([
            [np.cos(z2), -np.sin(z2), 0],
            [np.sin(z2), np.cos(z2), 0],
            [0, 0, 1]], dtype=np.float64)

        return rot_alpha @ rot_nu @ rot_psi

    def __getitem__(self, item):
        return self.zyz[item]

    def __sub__(self, other: ZYZEuler | np.ndarray) -> ZYZEuler | np.ndarray:
        if isinstance(other, ZYZEuler):
            return ZYZEuler(self.array - other.array)
        elif isinstance(other, np.ndarray):
            return np.array(self.zyz - other)
        else:
            raise TypeError(f"Expected type: ZYZEuler or np.ndarray, got {type(other)} instead.")

    def __repr__(self) -> str:
        return f"ZYZ(Z={self.zyz[0]:.3f}, x={self.zyz[1]:.3f}, y={self.zyz[2]:.3f})"
