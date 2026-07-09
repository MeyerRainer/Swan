""" Pose 6D (position + orientation in 3D) """

from backend import utils

import numpy as np


class Pose:

    def __init__(self, position: np.ndarray, rotation_matrix: np.ndarray):

        self._position = position.copy()
        self._rot_mat = rotation_matrix.copy()

    @classmethod
    def from_quaternion(cls, pos, quat):
        R = utils.quat2rot_mat(quat)
        return cls(position=pos, rotation_matrix=R)

    @classmethod
    def from_zyz_euler(cls, pos, zyz):
        R = utils.zyz2rot_mat(zyz)
        return cls(position=pos, rotation_matrix=R)

    @classmethod
    def identity(cls):
        return cls(position=np.zeros(3), rotation_matrix=np.eye(3))

    @classmethod
    def from_SE3(cls, T):
        # TODO: Add orthogonalization and normalization
        pos = T[:3, 3]
        R = T[:3, :3]
        return cls(position=pos, rotation_matrix=R)

    @property
    def position(self):
        return self._position.copy()

    @property
    def quaternion(self):
        return utils.rot_mat2quat(self._rot_mat)

    @property
    def rot_mat(self):
        return self._rot_mat.copy()

    @property
    def zyz_euler(self):
        return utils.rot2zyz(self._rot_mat)

    @property
    def SE3(self):
        T = np.eye(4)
        T[:3, :3] = self._rot_mat
        T[:3, 3] = self._position
        return T

    @position.setter
    def position(self, pos):
        if pos.shape != (3,):
            raise ValueError(f"Expected shape (3,), got {pos.shape} instead.")
        self._position = pos.copy()

    # Set orientation with quaternion
    @quaternion.setter
    def quaternion(self, quat):
        if quat.shape != (4,):
            raise ValueError(f"Expected shape (4,), got {quat.shape} instead.")
        self.rot_mat = utils.quat2rot_mat(quat)

    @rot_mat.setter
    def rot_mat(self, R: np.ndarray):
        if R.shape != (3, 3):
            raise ValueError(f"Expected shape (3, 3), got {R.shape} instead.")

        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt

        if np.linalg.det(R) < 0:
            U[:, 2] *= -1
            R = U @ Vt

        self._rot_mat = R

        # # Normalize and orthogonalize using Gram-Schmidt/Cross-Product method
        # # Extract rows (or columns) as vectors
        # x = rot[0, :]
        # y = rot[1, :]
        #
        # # 1. Normalize the first axis (X)
        # self._rot_mat[0, :] = x / np.linalg.norm(x)
        #
        # # 2. Force the second axis (Y) to be perpendicular to X, then normalize
        # y = y - np.dot(x, y) * x
        # self._rot_mat[1, :] = y / np.linalg.norm(y)
        #
        # # 3. Calculate the third axis (Z) using a cross product (guarantees det=1)
        # self._rot_mat[2, :] = np.cross(x, y)

    @zyz_euler.setter
    def zyz_euler(self, zyz: np.ndarray):
        if zyz.shape != (3,):
            raise ValueError(f"Expected shape (3,), got {zyz.shape} instead.")

        self._rot_mat = utils.zyz2rot_mat(zyz)

    @SE3.setter
    def SE3(self, T):
        if T.shape != (4, 4):
            raise ValueError(f"Expected shape (4, 4), got {T.shape} instead.")
        self.position = T[:3, 3]
        self.rot_mat = T[:3, :3]

    # Methods to be added
    # inverse()
    #
    # compose(other)
    #
    # relative_to(other)
    #
    # transform_point()
    #
    # interpolate()
    #
    # distance()
    #
    # copy()