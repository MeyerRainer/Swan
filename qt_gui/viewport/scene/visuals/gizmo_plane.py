""" Translation plane visual for interactive gizmo.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import numpy as np
from typing import Tuple, Optional
from PyQt6.QtGui import QVector3D
from dataclasses import dataclass

from robot_math.pose import Pose
from qt_gui.viewport import cg_math
from qt_gui.viewport.scene.visuals.visual import Visual


@dataclass
class PlaneSpecs:
    u_dir: Tuple[float, float, float] = (1., 0., 0.)
    v_dir: Tuple[float, float, float] = (0., 1., 0.)
    colors: Tuple[float, float, float, float] = (1., 0., 0., 0.5)
    offset: float = 0.001
    size: float = 0.030


# Translation plane mesh creation normal to axis_dir
class DragPlane(Visual):

    def __init__(self, params: PlaneSpecs):

        super().__init__()

        self._params = params
        # Original directions
        self.n_dir: Optional[np.ndarray] = None
        self.u_dir: Optional[np.ndarray] = None
        self.v_dir: Optional[np.ndarray] = None

        self._build(self._params)

    def _build(self, params: PlaneSpecs):

        vertices = []
        normals = []

        u_dir: QVector3D = QVector3D(*self._params.u_dir).normalized()
        v_dir: QVector3D = QVector3D(*self._params.v_dir).normalized()
        n_dir: QVector3D = QVector3D.crossProduct(u_dir, v_dir)

        self.n_dir = np.array([n_dir.x(), n_dir.y(), n_dir.z()])
        self.u_dir = np.array([u_dir.x(), u_dir.y(), u_dir.z()])
        self.v_dir = np.array([v_dir.x(), v_dir.y(), v_dir.z()])

        p0: QVector3D = u_dir * params.offset + v_dir * params.offset
        p1: QVector3D = p0 + u_dir * params.size
        p2: QVector3D = p0 + u_dir * params.size + v_dir * params.size
        p3: QVector3D = p0 + v_dir * params.size

        # Front side (+normal)
        for p in [p0, p1, p2, p0, p2, p3]:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([n_dir.x(), n_dir.y(), n_dir.z()])

        # Back side (-normal)
        rev_normal: QVector3D = -n_dir
        for p in [p0, p2, p1, p0, p3, p2]:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([rev_normal.x(), rev_normal.y(), rev_normal.z()])

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(self._params.colors, (len(self.vertices), 1))

    def hit(self, ray_origin: QVector3D, ray_dir: QVector3D, pose: Pose, body_frame=True):

        origin: QVector3D = QVector3D(*pose.position)
        rot_mat = pose.rot_mat if body_frame else np.eye(3)

        u: QVector3D = QVector3D(*(rot_mat @ self.u_dir))
        v: QVector3D = QVector3D(*(rot_mat @ self.v_dir))
        n: QVector3D = QVector3D(*(rot_mat @ self.n_dir))

        hit_radius_delta: float = 0.05 * self._params.size

        return cg_math.hit_translation_plane(ray_origin, ray_dir, origin, u, v, n, self._params.size, self._params.offset, hit_radius_delta)