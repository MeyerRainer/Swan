""" A Class for 6D Pose (position + orientation in 3D space).

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from __future__ import annotations

from typing import Tuple

from robot_math.quaternion import Quaternion
from robot_math.zyz_euler import ZYZEuler
import numpy as np

np.set_printoptions(suppress=True)

class Pose:

    def __init__(self, SE3: np.ndarray = None):

        self.pose = np.eye(4, dtype=np.float64)
        if SE3 is not None and SE3.shape == (4,4):
            self.pose = SE3.copy()

    def __repr__(self) -> str:
        return f"Position: {self.position}\tZYZ Euler: {np.rad2deg(self.zyz_euler.array)}"

    def __copy__(self):
        return Pose(self.pose)

    def copy(self):
        return self.__copy__()

    # ======================================== Constructors =======================================
    # TODO: Add orthogonalization, normalization and type checking.
    @classmethod
    def identity(cls) -> Pose:
        return cls(SE3=np.eye(4, dtype=np.float64))

    @classmethod
    def from_position(cls, pos: np.ndarray) -> Pose:
        SE3 = np.eye(4, dtype=np.float64)
        SE3[:3, 3] = pos
        return cls(SE3)

    @classmethod
    def from_quaternion(cls, pos: np.ndarray, quat: Quaternion) -> Pose:
        SE3 = np.eye(4, dtype=np.float64)
        SE3[:3, 3] = pos
        SE3[:3, :3] = quat.to_rotation_matrix()
        return cls(SE3)

    @classmethod
    def from_zyz_euler(cls, pos: np.ndarray, zyz: ZYZEuler) -> Pose:
        SE3 = np.eye(4, dtype=np.float64)
        SE3[:3, 3] = pos
        SE3[:3, :3] = zyz.rot_mat
        return cls(SE3)

    @classmethod
    def from_rot_mat(cls, pos: np.ndarray, R: np.ndarray) -> Pose:
        SE3 = np.eye(4, dtype=np.float64)
        SE3[:3, 3] = pos
        SE3[:3, :3] = R
        return cls(SE3)

    # ======================================== Getters =======================================
    @property
    def position(self) -> np.ndarray:
        return self.pose[:3, 3]

    @property
    def quaternion(self) -> Quaternion:
        return Quaternion.from_rotation_matrix(self.pose[:3, :3])

    @property
    def rot_mat(self) -> np.ndarray:
        return self.pose[:3, :3]

    # Pose, vision_sys, DRO, scene
    @property
    def zyz_euler(self) -> ZYZEuler:
        return ZYZEuler.from_rot_mat(self.pose[:3, :3])

    @property
    def SE3(self) -> np.ndarray:
        return self.pose.copy()

    @property
    def rodrigues(self) -> Tuple[np.ndarray, np.ndarray]:
        # Rodrigues formula
        R = self.rot_mat
        cos_theta: np.float64 = np.clip(0.5 * (np.linalg.trace(R) - 1), -1, 1)
        theta: np.float64 = np.acos(cos_theta)
        k: np.ndarray = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=np.float64).reshape(3, 1)
        k *=  0.5 / np.sin(theta)
        r_vec: np.ndarray = theta * k
        t_vec: np.ndarray = np.asarray(self.position, dtype=np.float64).reshape(3, 1)

        return r_vec, t_vec


    # ======================================== Setters =======================================
    # Set position.
    @position.setter
    def position(self, pos) -> None:
        if pos.shape != (3,):
            raise ValueError(f"Expected shape (3,), got {pos.shape} instead.")
        self.pose[:3, 3] = pos.copy()

    # Set orientation using quaternion.
    @quaternion.setter
    def quaternion(self, quat: Quaternion) -> None:
        self.pose[:3, :3] = quat.to_rotation_matrix()

    # Set orientation using rotation matrix.
    @rot_mat.setter
    def rot_mat(self, R: np.ndarray) -> None:
        if R.shape != (3, 3):
            raise ValueError(f"Expected shape (3, 3), got {R.shape} instead.")

        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt

        if np.linalg.det(R) < 0:
            U[:, 2] *= -1
            R = U @ Vt

        self.pose[:3, :3] = R

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

    # Set Orientation using ZYZ Euler angles.
    @zyz_euler.setter
    def zyz_euler(self, zyz: ZYZEuler) -> None:
        if zyz.shape != (3,):
            raise ValueError(f"Expected shape (3,), got {zyz.shape} instead.")
        self.pose[:3, :3] = zyz.rot_mat

    # Set position and orientation using SE3 transformation matrix.
    @SE3.setter
    def SE3(self, T) -> None:
        if T.shape != (4, 4):
            raise ValueError(f"Expected shape (4, 4), got {T.shape} instead.")
        self.pose = T.copy()

    @rodrigues.setter
    def rodrigues(self, rvec_tvec: Tuple[np.ndarray, np.ndarray]) -> None:
        r = np.asarray(rvec_tvec[0], dtype=np.float64).reshape(3)
        theta = np.linalg.norm(r)

        if theta < 1e-12:
            R = np.eye(3, dtype=np.float64)
        else:
            k = r / theta
            K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
            R = np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

        self.position = rvec_tvec[1].reshape(3)
        self.rot_mat = R

    # ===================================== Spatial operations ====================================
    # Vector transform: R^3 -> R^3
    def vector_mult(self, vec: np.ndarray) -> np.ndarray:
        vec_in: np.ndarray = np.ones((4, 1), dtype=np.float64)
        vec_in[:3, 0] = vec
        vec_out: np.ndarray = self.SE3 @ vec_in
        return vec_out[:3, 0].flatten()

    # Rotate around x.
    def rotate_x(self, angle: float, degrees=False, body_frame=True) -> None:
        R = np.eye(3)
        if degrees:
            angle = np.deg2rad(angle)
        R[1, 1], R[1, 2] = np.cos(angle), -np.sin(angle)
        R[2, 1], R[2, 2] = np.sin(angle), np.cos(angle)
        if body_frame:
            self.pose[:3, :3] = self.pose[:3, :3] @ R
        else:
            self.pose[:3, :3] = R @ self.pose[:3, :3]

    # Rotate around y.
    def rotate_y(self, angle: float, degrees=False, body_frame=True) -> None:
        R = np.eye(3)
        if degrees:
            angle = np.deg2rad(angle)
        R[0, 0], R[0, 2] = np.cos(angle), -np.sin(angle)
        R[2, 0], R[2, 2] = np.sin(angle), np.cos(angle)
        if body_frame:
            self.pose[:3, :3] = self.pose[:3, :3] @ R
        else:
            self.pose[:3, :3] = R @ self.pose[:3, :3]

    # Rotate around z.
    def rotate_z(self, angle: float, degrees=False, body_frame=True) -> None:
        R = np.eye(3)
        if degrees:
            angle = np.deg2rad(angle)
        R[0, 0], R[0, 1] = np.cos(angle), -np.sin(angle)
        R[1, 0], R[1, 1] = np.sin(angle), np.cos(angle)
        if body_frame:
            self.pose[:3, :3] = self.pose[:3, :3] @ R
        else:
            self.pose[:3, :3] = R @ self.pose[:3, :3]

    # Rotate about arbitrary axis an angle.
    def rotate_axis_angle(self, axis: np.ndarray, angle: float, degrees=False) -> None:
        # TODO
        ...


    # Return inverse of self.
    def inverse(self) -> Pose:
        inv = Pose.identity()
        inv.rot_mat = self.rot_mat.T
        inv.position = inv.rot_mat @ self.position * -1
        return inv

    # Compose self with the other. For instance, T_A->C = T_A->B * T_B->C.
    def compose(self, other: Pose) -> Pose:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return Pose(self.SE3 @ other.SE3)

    # Represents self in another frame. For instance, T_B->C = (T_A->B)^-1 * T_A->C.
    def relative_to(self, other: Pose) -> Pose:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return other.inverse().compose(self)

    # Euclidean distance from self to the other.
    def distance(self, other: Pose) -> float:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return np.linalg.norm((self.position - other.position))

    def translation_direction(self, other: Pose) -> np.ndarray:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        dir_vec = other.position - self.position
        norm = np.linalg.norm(dir_vec)
        if norm > 1e-6:
            return dir_vec / norm
        return np.zeros(3)

    def rotation_direction(self, other: Pose) -> np.ndarray:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return self.quaternion.rotation_direction(other.quaternion)

    def angle(self, other: Pose):
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return self.quaternion.angle(other.quaternion)

    def interpolate(self, other: Pose, t: float = 0.5) -> Pose:
        """ Interpolates an intermediate pose between self and other at interpolation parameter t∈[0, 1] """
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        inter_position = self.position + t * (other.position - self.position)
        inter_quaternion = self.quaternion.slerp(other.quaternion, t)
        return Pose.from_quaternion(pos=inter_position, quat=inter_quaternion)

    def arc_interpolate(self, end: Pose, arc_center: np.ndarray, arc_normal: np.ndarray, angle: float, t: float = 0.5) -> Pose:
        """ Interpolates over arc
        :param end: End Pose.
        :param arc_center: Center point of arc.
        :param arc_normal: Normalized rotation axis.
        :param angle: Angle of rotation in radians.
        :param t: Interpolation parameter in range [0, 1].
        """
        v: np.ndarray = self.position - arc_center  # Center -> start point vector.
        theta: float = t * angle
        cos_theta: float = np.cos(theta)
        # Position by Rodrigues' formula:
        inter_position = arc_center + v*cos_theta + np.cross(arc_normal, v)*np.sin(theta) + arc_normal*np.dot(arc_normal, v)*(1-cos_theta)
        inter_quaternion: Quaternion = self.quaternion.slerp(end.quaternion, t=t)
        return Pose.from_quaternion(pos=inter_position, quat=inter_quaternion)

    def is_close(self, other: Pose) -> bool:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return np.allclose(self.SE3, other.SE3, rtol=1e-5)

    # ====================================== Arithmetic ===========================================
    def __eq__(self, other: Pose) -> bool:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        return self.is_close(other)

    def __rmul__(self, other: Pose) -> None:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        self.SE3 = self.SE3 @ other.SE3

    def __mul__(self, other: Pose) -> None:
        if not isinstance(other, Pose):
            raise TypeError(f"Expected type Pose, got {type(other)} instead.")
        self.SE3 = other.SE3 @ self.SE3
