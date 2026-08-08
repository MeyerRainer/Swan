""" Grid visual.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from qt_gui.viewport.visuals.visual import Visual, Material
import numpy as np


class Grid(Visual):

    def __init__(self, name, size=1., step=0.05, color=(0.3, 0.3, 0.3)):

        super().__init__()

        self.name: str = name

        self.size: float = size
        self.step: float = step
        self.color = np.array(color)

        self.vao = None
        self.vbo = None
        self.vertex_count = 0

        self.line_render = True

        self._build()

    def _build(self):
        vertices = []

        # Generate grid lines on XY plane.
        for i in np.arange(-self.size, self.size + self.step, self.step):
            # Line parallel to Y-axis (or Z)
            vertices.extend([i, -self.size, 0.0])
            vertices.extend([i, self.size, 0.0])
            # Line parallel to X-axis
            vertices.extend([-self.size, i, 0.0])
            vertices.extend([self.size, i, 0.0])

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        # Default upward normals for 2D plane grid.
        self.normals = np.tile([0.0, 0.0, 1.0], (len(self.vertices), 1)).astype(np.float32)
        # Sequential indices for lines (or triangles if rendered as GL_LINES).
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        # Assign an explicit fallback material using the RGB parameters.
        self.material = Material(name="GridColor")
        self.material.diffuse = np.array([self.color[0], self.color[1], self.color[2]], dtype=np.float32)
