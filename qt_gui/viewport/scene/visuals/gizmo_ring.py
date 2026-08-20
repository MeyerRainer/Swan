""" Translation rotation ring visual for interactive gizmo.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import math
import numpy as np
from typing import Tuple
from PyQt6.QtGui import QVector3D
from dataclasses import dataclass

from robot_math.pose import Pose
from qt_gui.viewport import cg_math
from qt_gui.viewport.scene.visuals.visual import Visual


@dataclass
class RingSpecs:
    normal_axis: Tuple[float, float, float] = (1., 0., 0.)
    colors: Tuple[float, float, float, float] = (1., 0., 0., 0.5)
    radius_in: float = 0.04
    radius_out: float = 0.05
    n_segments: int = 32


class RotationRing(Visual):

    def __init__(self, params: RingSpecs):

        super().__init__()

        self._params = params

        self._build(self._params)

    def _build(self, params: RingSpecs):

        vertices = []
        normals = []

        # Determine orthogonal plane basis vectors
        # Axis direction and orthogonal vectors relative to axis.
        # Build orthogonal basis
        normal_dir: QVector3D = QVector3D(*params.normal_axis).normalized()
        other: QVector3D = QVector3D(1, 1, 1).normalized()
        if math.fabs(normal_dir.dotProduct(normal_dir, other)) > 0.9:
            other = QVector3D(-1, 1, 1).normalized()
        u_dir: QVector3D = normal_dir.crossProduct(normal_dir, other).normalized()
        v_dir: QVector3D = normal_dir.crossProduct(normal_dir, u_dir).normalized()

        for i in range(params.n_segments):
            # Increment in range [0, 1]
            a1: float = 2 * math.pi * (i / params.n_segments)
            a2: float = 2 * math.pi * ((i + 1) / params.n_segments)

            c1, s1 = math.cos(a1), math.sin(a1)
            c2, s2 = math.cos(a2), math.sin(a2)

            # Vertices for inner and outer rings on both sides
            p1_in: QVector3D = params.radius_in*(c1*u_dir + s1*v_dir)
            p1_out: QVector3D = params.radius_out*(c1*u_dir + s1*v_dir)
            p2_in: QVector3D = params.radius_in*(c2*u_dir + s2*v_dir)
            p2_out: QVector3D = params.radius_out*(c2*u_dir + s2*v_dir)

            # # Two triangles forming a quad ring segment
            # # Render both sides (clockwise & counter-clockwise) so ring is visible from any angle
            for p in [p1_in, p1_out, p2_out, p1_in, p2_out, p2_in]:
                vertices.extend([p.x(), p.y(), p.z()])
                normals.extend([normal_dir.x(), normal_dir.y(), normal_dir.z()])

            # Reverse side (-normal_dir)
            rev_normal = -normal_dir
            for p in [p1_in, p2_out, p1_out, p1_in, p2_in, p2_out]:
                vertices.extend([p.x(), p.y(), p.z()])
                normals.extend([rev_normal.x(), rev_normal.y(), rev_normal.z()])

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(self._params.colors, (len(self.vertices), 1))

    def hit(self, ray_origin: QVector3D, ray_dir: QVector3D, pose: Pose, body_frame=True):

            origin: QVector3D = QVector3D(*pose.position)
            rot_mat = pose.rot_mat if body_frame else np.eye(3)

            normal_axis: np.ndarray = np.array(self._params.normal_axis)
            normal_axis: QVector3D = QVector3D(*(rot_mat @ normal_axis)).normalized()
            delta_radius = 0.05 * self._params.radius_out

            return cg_math.hit_rotation_ring(ray_origin, ray_dir, origin, normal_axis, self._params.radius_out, self._params.radius_in, delta_radius)