""" Class for a scene object.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from typing import Protocol, Any

import numpy as np


class Material:

    def __init__(self, name: str=""):

        self.name: str = name
        self.diffuse: np.ndarray = np.array([0.8, 0.8, 0.8], dtype=np.float32)
        self.ambient: np.ndarray = np.array([0.2, 0.2, 0.2], dtype=np.float32)
        self.specular: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float32)


class Visual:
    """Holds CPU-side 3D geometry and material data ready for OpenGL upload."""
    def __init__(self):
        # Flattened NumPy arrays ready to pass to glBufferData
        self.vertices: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.normals: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.textures: np.ndarray = np.empty((0, 2), dtype=np.float32)
        self.colors: np.ndarray = np.empty((0, 4), dtype=np.float32)
        self.indices: np.ndarray = np.empty((0,), dtype=np.uint32)

        self.material: Material | None = None

        # GPU Buffer Handles (Populated later inside your QOpenGLWidget)
        self.vao_id: int | None = None
        self.vbo_id: int | None = None
        self.ebo_id: int | None = None

        self.line_render: bool = False  # Triangle or line rendering.

    def _build(self, params: Any) -> None: ...
