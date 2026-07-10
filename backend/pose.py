""" Pose 6D (position + orientation in 3D) """
from __future__ import annotations  # Must be the very first line of code

import copy

from backend import utils
from typing import List
import numpy as np


class Pose:

    def __init__(self, position: np.ndarray, rotation_matrix: np.ndarray):

        self._position = position.copy()
        self._rot_mat = rotation_matrix.copy()

    def __repr__(self):
        return (
            f"Pose("
            f"position={self._position!r}, "
            f"quaternion={self.quaternion!r})"
        )

    def __copy__(self):
        return Pose(self._position, self._rot_mat)

    def __deepcopy__(self, memo):
        return Pose(self._position, self._rot_mat)

    def copy(self):
        return self.__copy__()

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
        norm = np.linalg.norm(quat)
        if norm < 1e-3:
            raise ValueError("Zero length quaternion")
        quat /= norm
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

    def inverse(self):
        inv = Pose.identity()
        inv.rot_mat = self.rot_mat.T
        inv.position = inv.rot_mat @ self.position * -1
        return inv

    def compose(self, other: Pose):
        return Pose.from_SE3(self.SE3 @ other.SE3)

    def relative_to(self, other: Pose):
        return other.inverse().compose(self)

    # transform_point()
    #
    # interpolate()
    #
    def distance(self, other: Pose):
        return np.linalg.norm((self.position - other.position))

    def angle(self, other: Pose):
        similarity = np.clip(np.dot(self.quaternion, other.quaternion), -1, 1)
        similarity = abs(similarity)
        return 2 * np.acos(similarity)

    # def interpolate(self, other: Pose, segment_size_m: float, segment_size_rad: float) -> List[Pose]:
    #     poses = []
    #
    #     start_pos = a.position
    #     start_quat = a.quaternion
    #     end_pos = b.position
    #     end_quat = b.quaternion
    #
    #     # Translational error, meters
    #     transl = end_pos - start_pos
    #     transl_norm = LA.norm(transl)
    #     n_segments_lin = int(np.ceil(transl_norm / segment_size_m))
    #
    #     # Rotational error, radians
    #     similarity = np.clip(np.dot(start_quat, end_quat), -1, 1)
    #     if similarity < 0:
    #         end_quat *= -1
    #         similarity *= -1
    #     rotation_dist = 2 * math.acos(similarity)
    #     n_segments_ang = int(np.ceil(rotation_dist / segment_size_rad))
    #
    #     # Choose whether rotation or translation determines segment count
    #     n_segments = max(n_segments_lin, n_segments_ang)
    #     assert n_segments >= 1, "Pose interpolator: no segments!"
    #
    #     for idx in range(n_segments):
    #         t = (idx + 1) / n_segments  # Interpolation parameter in range ]0, 1]
    #         pos_idx = start_pos + t * transl  # Interpolated position
    #         quat_idx = slerp(start_quat, end_quat, t)  # Interp. orientation
    #         poses.append(Pose(np.hstack((pos_idx, quat_idx))))
    #
    #     return poses

    def is_close(self, other: Pose):
        return np.allclose(self.SE3, other.SE3, rtol=1e-5)