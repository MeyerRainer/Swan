""" Classes for geometric primitives in 3D space.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import numpy as np


class Axis:

    def __init__(self, p1: np.ndarray, p2: np.ndarray):

        self.p1 = p1
        self.p2 = p2

    @classmethod
    def from_point_and_vector(cls, p1: np.ndarray, v1: np.ndarray):
        return cls(p1, p1 + v1)

class Plane:

    def __init__(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):

        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    @classmethod
    def axis_and_point(cls, axis: Axis, point: np.ndarray):
        return cls(axis.p1, axis.p2, point)

    @classmethod
    def from_point_and_normal(cls, point: np.ndarray, normal: np.ndarray):
        p1 = point
        p2 = np.cross(point, normal)  # Perpendicular to normal and point
        p3 = np.cross(p2, normal)  # Perpendicular to normal
        return cls(p1, p2, p3)


def is_collinear_by_deviation(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, max_deviation: float = 0.1) -> bool:

    u: np.ndarray = p2 - p1
    v: np.ndarray = p3 - p1

    v_norm = np.linalg.norm(v)
    # p1 and p3 coincident.
    if np.fabs(v_norm) < 1e-6:
        return np.linalg.norm(u) <= max_deviation

    deviation: float = np.linalg.norm(np.cross(u, v)) / v_norm

    return deviation <= max_deviation

def is_collinear_by_angle(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, max_angle: float = 0.01) -> bool:

    u: np.ndarray = p2 - p1
    v: np.ndarray = p3 - p1

    u_norm: float = np.linalg.norm(u)
    v_norm: float = np.linalg.norm(v)

    if np.fabs(u_norm) < 1e-6 or np.fabs(v_norm) < 1e-6:
        return True

    sin_theta: float = np.clip(np.linalg.norm(np.cross(u, v)) / (u_norm * v_norm), 0, 1)
    theta: float = np.arcsin(sin_theta)

    return theta <= max_angle
