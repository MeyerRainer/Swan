""" Grid visual.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass
from typing import Tuple

from qt_gui.viewport.scene.visuals.visual import Visual
import numpy as np

@dataclass
class GridSpecs:
    size: float = 1.
    step: float = 0.05
    colors: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 0.5)

class Grid(Visual):

    def __init__(self, name, params: GridSpecs):

        super().__init__()

        self.name: str = name

        self._params = params

        self.vao = None
        self.vbo = None
        self.vertex_count = 0

        self.line_render = True

        self._build(params)

    def _build(self, params: GridSpecs):
        vertices = []

        # Generate grid lines on XY plane.
        for i in np.arange(-params.size, params.size + params.step, params.step):
            # Line parallel to Y-axis (or Z)
            vertices.extend([i, -params.size, 0.0])
            vertices.extend([i, params.size, 0.0])
            # Line parallel to X-axis
            vertices.extend([-params.size, i, 0.0])
            vertices.extend([params.size, i, 0.0])

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        # Default upward normals for 2D plane grid.
        self.normals = np.tile([0.0, 0.0, 1.0], (len(self.vertices), 1)).astype(np.float32)
        # Sequential indices for lines (or triangles if rendered as GL_LINES).
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        # Assign an explicit fallback material using the RGB parameters.
        self.colors = np.tile(params.colors, (len(self.vertices), 1))
        # self.material = Material(name="GridColor")
        # self.material.diffuse = np.array([self.color[0], self.color[1], self.color[2]], dtype=np.float32)
