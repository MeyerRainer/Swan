""" Quaternion class. Mostly AI-generated.
"""
from __future__ import annotations

import math
from typing import Tuple, Union
import numpy as np


class Quaternion:
    """An immutable 3D Quaternion class for rotations and spatial math.

    Represented as: q = w + x*i + y*j + z*k
    where 'w' is the scalar (real) component, and (x, y, z) is the vector component.
    """

    __slots__ = ("_w", "_x", "_y", "_z")

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self._w = float(w)
        self._x = float(x)
        self._y = float(y)
        self._z = float(z)

    @classmethod
    def from_iterable(cls, components):
        return cls(components[0], components[1], components[2], components[3])

    # --- Properties (Immutable) ---
    @property
    def w(self) -> float:
        return self._w

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def z(self) -> float:
        return self._z

    @property
    def scalar(self) -> float:
        return self._w

    @property
    def vector(self) -> Tuple[float, float, float]:
        return self._x, self._y, self._z

    @property
    def components(self) -> Tuple[float, float, float, float]:
        return self._w, self._x, self._y, self._z

    # --- Factory Constructors ---
    @classmethod
    def identity(cls) -> Quaternion:
        """Returns the identity quaternion (1, 0, 0, 0)."""
        return cls(1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_axis_angle(cls, axis: Tuple[float, float, float], angle_rad: float) -> Quaternion:
        """ Creates a unit rotation quaternion from a 3D axis vector and angle in radians. """
        ax, ay, az = axis
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            raise ValueError("Axis vector cannot be zero-length.")

        half_angle = angle_rad * 0.5
        sin_half = math.sin(half_angle)
        scale = sin_half / norm

        return cls(
            math.cos(half_angle),
            ax * scale,
            ay * scale,
            az * scale
        )

    @classmethod
    def from_rpy_euler(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """ Creates a quaternion from Euler angles (in radians) using Z-Y-X rotation sequence. """
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy

        return cls(w, x, y, z)

    @classmethod
    def from_zyz_euler(cls, phi: float, nu: float, psi: float) -> Quaternion:
        """ Creates a quaternion from Euler angles (in radians) using Z-Y-Z rotation sequence. """
        c_phi, s_phi = math.cos(phi * 0.5), math.sin(phi * 0.5)
        c_nu, s_nu = math.cos(nu * 0.5), math.sin(nu * 0.5)
        c_psi, s_psi = math.cos(psi * 0.5), math.sin(psi * 0.5)

        # Quaternion component calculations for ZYZ sequence
        w = c_nu*(c_phi*c_psi - s_phi*s_psi)
        x = s_nu*(s_phi*c_psi - c_phi*s_psi)
        y = s_nu*(c_phi*c_psi + s_phi*s_psi)
        z = c_nu*(s_phi*c_psi + c_phi*s_psi)

        return cls(w, x, y, z)

    @classmethod
    def from_rot_mat(cls, R: np.ndarray):
        """ Convert a 3x3 rotation matrix to a unit quaternion
        :param R: (np.ndarray): 3x3 rotation matrix
        :return: Quaternion
        """
        trace = np.trace(R)

        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2  # S = 4*w
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4*x
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4*y
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4*z
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

        return cls(w, x, y, z).normalized()

    def to_rot_mat(self):
        """ Conversion from quaternion to rotation matrix """
        w, x, y, z = self._w, self._x, self._y, self._z
        return np.array([
            [2 * (w * w + x * x) - 1, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 2 * (w * w + y * y) - 1, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 2 * (w * w + z * z) - 1]
        ], dtype=np.float64)

    # --- Fundamental Mathematical Properties ---
    def norm_sq(self) -> float:
        """ Returns the squared norm (magnitude^2). """
        return self._w ** 2 + self._x ** 2 + self._y ** 2 + self._z ** 2

    def norm(self) -> float:
        """ Returns the norm (magnitude) of the quaternion. """
        return math.sqrt(self.norm_sq())

    def conjugate(self) -> Quaternion:
        """ Returns the quaternion conjugate: q* = w - xi - yj - zk. """
        return Quaternion(self._w, -self._x, -self._y, -self._z)

    def normalized(self) -> Quaternion:
        """ Returns a unit quaternion pointing in the same direction. """
        n = self.norm()
        if n == 0:
            raise ZeroDivisionError("Cannot normalize a zero-length quaternion.")
        return self / n

    def relative_to(self, other: Quaternion) -> Quaternion:
        if not isinstance(other, Quaternion):
            raise TypeError(f"Expected type Quaternion, got {type(other)} instead.")
        return other.inverse() * self

    def inverse(self) -> Quaternion:
        """ Returns the multiplicative inverse q^-1 = q* / |q|^2. """
        n_sq = self.norm_sq()
        if n_sq == 0:
            raise ZeroDivisionError("Cannot invert a zero-length quaternion.")
        conj = self.conjugate()
        return Quaternion(conj.w / n_sq, conj.x / n_sq, conj.y / n_sq, conj.z / n_sq)

    # --- Operators & Arithmetic ---
    def __add__(self, other: Quaternion) -> Quaternion:
        if isinstance(other, Quaternion):
            return Quaternion(self._w + other._w, self._x + other._x, self._y + other._y, self._z + other._z)
        return NotImplemented

    def __sub__(self, other: Quaternion) -> Quaternion:
        if isinstance(other, Quaternion):
            return Quaternion(self._w - other._w, self._x - other._x, self._y - other._y, self._z - other._z)
        return NotImplemented

    def __mul__(self, other: Union[Quaternion, float, int]) -> Quaternion:
        if isinstance(other, (int, float)):
            return Quaternion(self._w * other, self._x * other, self._y * other, self._z * other)

        if isinstance(other, Quaternion):
            # Hamilton product
            w1, x1, y1, z1 = self._w, self._x, self._y, self._z
            w2, x2, y2, z2 = other._w, other._x, other._y, other._z
            return Quaternion(
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            )
        return NotImplemented

    def __rmul__(self, other: Union[float, int]) -> Quaternion:
        return self.__mul__(other)

    def __truediv__(self, scalar: float) -> Quaternion:
        if not isinstance(scalar, (int, float)):
            raise TypeError(f"Expected type int or float, got {type(scalar)} instead.")
        return Quaternion(self._w / scalar, self._x / scalar, self._y / scalar, self._z / scalar)

    def __neg__(self) -> Quaternion:
        return Quaternion(-self._w, -self._x, -self._y, -self._z)

    def dot(self, other: Quaternion) -> float:
        """ Computes the inner product between two quaternions. """
        return self._w * other._w + self._x * other._x + self._y * other._y + self._z * other._z

    def angle(self, other: Quaternion) -> float:
        """ Returns the angle (radians) between self and other """
        similarity = abs(np.clip(self.dot(other), -1, 1))
        return 2 * math.acos(similarity)

    def rotation_direction(self, other: Quaternion) -> np.ndarray:
        """Computes the unit rotation axis (direction) from self to other.
            Returns np.zeros(3) if the angle is close to pi or zero.
        """
        # Relative rotation
        q_rel = self.inverse() * other

        # Extract scalar (w) and vector (x, y, z) components
        w = q_rel.w
        v = np.array([q_rel.x, q_rel.y, q_rel.z], dtype=float)

        # Take the shortest rotation path (w >= 0)
        if w < 0:
            w = -w
            v = -v

        w = np.clip(w, -1.0, 1.0)
        angle = 2.0 * np.arccos(w)

        # Return zero array if angle is close to pi (or 0)
        if np.isclose(angle, np.pi, atol=1e-4) or np.isclose(angle, 0.0, atol=1e-4):
            return np.zeros(3)

        # Compute normalized unit axis: u = v / sin(angle / 2)
        sin_half = np.sqrt(1.0 - w * w)
        if sin_half < 1e-8:
            return np.zeros(3)

        return v / sin_half

    # --- Spatial Operations ---
    def rotate_vector(self, vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """ Rotates a 3D vector (x, y, z) using this quaternion: v' = q * v * q^-1.

        Assumes the quaternion is normalized.
        """
        vx, vy, vz = vec
        # Pure vector quaternion q_v = (0, vx, vy, vz)
        # Using optimized active rotation formula: v' = v + 2*r x (r x v + w*v)
        rx, ry, rz = self._x, self._y, self._z
        w = self._w

        # Cross product t = 2 * (r x v)
        tx = 2.0 * (ry * vz - rz * vy)
        ty = 2.0 * (rz * vx - rx * vz)
        tz = 2.0 * (rx * vy - ry * vx)

        # v' = v + w * t + (r x t)
        return (
            vx + w * tx + (ry * tz - rz * ty),
            vy + w * ty + (rz * tx - rx * tz),
            vz + w * tz + (rx * ty - ry * tx),
        )

    def slerp(self, target: Quaternion, t: float, eps: float = 1e-6) -> Quaternion:
        """ Spherical Linear Interpolation between self and target for a parameter t in [0, 1]. """
        cos_theta = self.dot(target)

        # Ensure the shortest path on the 4D sphere
        target_q = target
        if cos_theta < 0.0:
            cos_theta = -cos_theta
            target_q = -target

        # If quaternions are very close, fallback to linear interpolation to avoid division by zero
        if cos_theta > 1.0 - eps:
            result = self * (1.0 - t) + target_q * t
            return result.normalized()

        theta = math.acos(cos_theta)
        sin_theta = math.sin(theta)

        w1 = math.sin((1.0 - t) * theta) / sin_theta
        w2 = math.sin(t * theta) / sin_theta

        return self * w1 + target_q * w2

    # --- Representation & Equality ---
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Quaternion):
            raise TypeError(f"Expected type Quaternion, got {type(other)} instead.")
        return math.isclose(self._w, other._w) and \
            math.isclose(self._x, other._x) and \
            math.isclose(self._y, other._y) and \
            math.isclose(self._z, other._z)

    def __repr__(self) -> str:
        return f"Quaternion(w={self._w:.4f}, x={self._x:.4f}, y={self._y:.4f}, z={self._z:.4f})"