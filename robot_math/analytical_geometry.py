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