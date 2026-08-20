""" Translation arrow visual for interactive gizmo.

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
class ArrowSpecs:
    axis: Tuple[float, float, float]
    colors: Tuple[float, float, float, float] = (1., 0.2, 0.2, 0.8)
    length_arrow: float  = 0.05
    length_cone: float = 0.015
    radius_shaft: float = 0.002
    radius_cone: float = 0.004
    n_segments: int = 12


class DragArrow(Visual):

    def __init__(self, params: ArrowSpecs):

        super().__init__()

        self._params = params
        self._axis: np.ndarray = np.array(params.axis) * (params.length_arrow / np.linalg.norm(np.array(params.axis)))

        self._build(self._params)

    def _build(self, params: ArrowSpecs):

        length_shaft: float = params.length_arrow - params.length_cone

        vertices = []
        normals = []

        # Build orthogonal basis
        axis_dir: QVector3D = QVector3D(*params.axis).normalized()
        other: QVector3D = QVector3D(1, 1, 1).normalized()
        if math.fabs(axis_dir.dotProduct(axis_dir, other)) > 0.9:
            other = QVector3D(-1, 1, 1).normalized()
        u_dir: QVector3D = axis_dir.crossProduct(axis_dir, other).normalized()
        v_dir: QVector3D = axis_dir.crossProduct(axis_dir, u_dir).normalized()

        # Arrow shaft
        for i in range(params.n_segments):
            # Increment in range [0, 1]
            a1: float = (i / params.n_segments) * 2 * math.pi
            a2: float = ((i + 1) / params.n_segments) * 2 * math.pi

            # Radial unit directions (equivalent to cylinder surface normals)
            n1: QVector3D = (math.cos(a1) * u_dir + math.sin(a1) * v_dir).normalized()
            n2: QVector3D = (math.cos(a2) * u_dir + math.sin(a2) * v_dir).normalized()

            # Radial vectors for shaft vertices
            r1: QVector3D = params.radius_shaft * n1
            r2: QVector3D = params.radius_shaft * n2

            # Circle vertices at the beginning and end of the shaft
            p1: QVector3D = r1
            p2: QVector3D = r1 + axis_dir * length_shaft
            p3: QVector3D = r2 + axis_dir * length_shaft
            p4: QVector3D = r2

            quad_data = [(p1, n1), (p2, n1), (p3, n2), (p1, n1), (p3, n2), (p4, n2)]
            for p, n in quad_data:
                vertices.extend([p.x(), p.y(), p.z()])
                normals.extend([n.x(), n.y(), n.z()])

        # Arrow tip
        tip_base: QVector3D = length_shaft * axis_dir
        tip_apex: QVector3D = (length_shaft + params.length_cone) * axis_dir

        # Cone slant angle normal adjustment factor
        cone_slope: float = params.radius_cone / params.length_cone

        for i in range(params.n_segments):
            a1: float = 2 * math.pi * (i / params.n_segments)
            a2: float = 2 * math.pi * ((i + 1) / params.n_segments)
            a_mid: float = (a1 + a2) * 0.5

            r1_dir = math.cos(a1) * u_dir + math.sin(a1) * v_dir
            r2_dir = math.cos(a2) * u_dir + math.sin(a2) * v_dir
            r_mid_dir: QVector3D = math.cos(a_mid) * u_dir + math.sin(a_mid) * v_dir

            # Slanted normals pointing outward from the cone face
            n1 = (r1_dir + axis_dir * cone_slope).normalized()
            n2 = (r2_dir + axis_dir * cone_slope).normalized()
            n_apex = (r_mid_dir + axis_dir * cone_slope).normalized()

            p1 = tip_base + params.radius_cone * r1_dir
            p2 = tip_base + params.radius_cone * r2_dir
            for p, n in [(p1, n1), (tip_apex, n_apex), (p2, n2)]:
                vertices.extend([p.x(), p.y(), p.z()])
                normals.extend([n.x(), n.y(), n.z()])

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(params.colors, (len(self.vertices), 1))

    def hit(self, ray_origin: QVector3D, ray_dir: QVector3D, pose: Pose) -> bool:
        """ Checks if translation arrow has been hit. If so, which one.
        :param ray_origin: Mouse ray origin (Camera or near point)
        :param ray_dir: Direction of mouse ray. Unit vector.
        :param pose: Pose of parent object
        """
        pose: Pose = pose
        axis_origin: QVector3D = QVector3D(*pose.position)
        axis_dir: QVector3D = QVector3D(*(pose.rot_mat @ self._axis)).normalized()

        return cg_math.hit_arrow(ray_origin, ray_dir, axis_origin, axis_dir, self._params.length_arrow, 1.5 * self._params.radius_shaft)