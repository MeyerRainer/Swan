""" Class for renderable ellipsoid.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from dataclasses import dataclass
from PyQt6.QtGui import QVector3D
import numpy as np
import math

from robot_math.pose import Pose
from qt_gui.viewport.visuals.visual import Visual


@dataclass
class EllipsoidSpecs:
  radii: tuple = (1.0, 1.0, 1.0)  # Semi-axes (rx, ry, rz)
  center: tuple = (0.0, 0.0, 0.0)  # Local center offset (x, y, z)
  n_stacks: int = 32  # Latitude divisions (top to bottom)
  n_sectors: int = 32  # Longitude divisions (around z-axis)
  colors: tuple = (0.2, 0.6, 1.0, 0.5)  # RGBA color tuple


class Ellipsoid(Visual):

  def __init__(self, params: EllipsoidSpecs):

    super().__init__()

    self._params = params
    self._radii = np.array(params.radii, dtype=np.float32)
    self._center = QVector3D(*params.center)

    self._build(self._params)

  def _build(self, params: EllipsoidSpecs):
    rx, ry, rz = params.radii
    cx, cy, cz = params.center
    center_vec = QVector3D(cx, cy, cz)

    vertices = []
    normals = []

    for i in range(params.n_stacks):
      # Latitude angles (phi from 0 to pi)
      phi1: float = math.pi * (i / params.n_stacks)
      phi2: float = math.pi * ((i + 1) / params.n_stacks)

      for j in range(params.n_sectors):
        # Longitude angles (theta from 0 to 2*pi)
        theta1: float = 2 * math.pi * (j / params.n_sectors)
        theta2: float = 2 * math.pi * ((j + 1) / params.n_sectors)

        # Helper lambda to compute parametric vertex and normal
        def compute_point_and_normal(
            phi: float, theta: float
        ) -> tuple[QVector3D, QVector3D]:
          # Unit sphere coords
          sin_phi = math.sin(phi)
          cos_phi = math.cos(phi)
          sin_theta = math.sin(theta)
          cos_theta = math.cos(theta)

          # Surface position scaled by radii + offset
          p = center_vec + QVector3D(
              rx * sin_phi * cos_theta,
              ry * sin_phi * sin_theta,
              rz * cos_phi,
          )

          # Surface normal vector derived from ellipsoid gradient: (x/rx^2, y/ry^2, z/rz^2)
          n = QVector3D(
              (sin_phi * cos_theta) / rx,
              (sin_phi * sin_theta) / ry,
              cos_phi / rz,
          ).normalized()

          return p, n

        # Quad corner evaluation
        p1, n1 = compute_point_and_normal(phi1, theta1)
        p2, n2 = compute_point_and_normal(phi2, theta1)
        p3, n3 = compute_point_and_normal(phi2, theta2)
        p4, n4 = compute_point_and_normal(phi1, theta2)

        # Two triangles per quad face: (p1, p2, p3) and (p1, p3, p4)
        quad_data = [(p1, n1), (p2, n2), (p3, n3), (p1, n1), (p3, n3), (p4, n4)]

        for p, n in quad_data:
          vertices.extend([p.x(), p.y(), p.z()])
          normals.extend([n.x(), n.y(), n.z()])

    self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
    self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
    self.indices = np.arange(len(self.vertices), dtype=np.uint32)
    self.colors = np.tile(params.colors, (len(self.vertices), 1))

  def hit(self, ray_origin: QVector3D, ray_dir: QVector3D, pose: Pose) -> bool:
    """Checks if the ellipsoid visual is intersected by a ray.

    :param ray_origin: Mouse ray origin in world coordinates
    :param ray_dir: Mouse ray direction unit vector
    :param pose: Pose of parent object
    """
    # 1. Transform ray origin to local ellipsoid space considering Pose
    world_center = QVector3D(*pose.position) + pose.rot_mat @ self._center
    rel_origin = ray_origin - world_center

    # 2. Transform ray into local orientation if pose includes rotation
    inv_rot = pose.rot_mat.T
    local_origin = inv_rot @ rel_origin
    local_dir = inv_rot @ ray_dir

    # 3. Transform ray into unit-sphere space by dividing by radii
    rx, ry, rz = self._radii
    o_scaled = np.array([
        local_origin.x() / rx,
        local_origin.y() / ry,
        local_origin.z() / rz,
    ])
    d_scaled = np.array(
        [local_dir.x() / rx, local_dir.y() / ry, local_dir.z() / rz]
    )

    # 4. Ray vs Unit-Sphere quadratic equation: ||o + t*d||^2 = 1
    a = np.dot(d_scaled, d_scaled)
    b = 2.0 * np.dot(o_scaled, d_scaled)
    c = np.dot(o_scaled, o_scaled) - 1.0

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
      return False

    # Check if at least one intersection point is in front of the ray origin
    t1 = (-b - math.sqrt(discriminant)) / (2.0 * a)
    t2 = (-b + math.sqrt(discriminant)) / (2.0 * a)

    return t1 >= 0 or t2 >= 0