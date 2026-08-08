from qt_gui.viewport.visuals.visual import Visual, Material
from qt_gui.viewport.shapes import *

import numpy as np


class Arrow(Visual):

    def __init__(self, specs: ArrowSpecs):

        super().__init__()

        self.specs = specs

        self._build()


    def _build(self):

        vertices, normals = build_arrow(self.specs)
        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        # Sequential indices for lines (or triangles if rendered as GL_LINES).
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        # Assign an explicit fallback material using the RGB parameters.
        self.color = np.array(self.specs.color)
        # self.material = Material(name="GridColor")
        # self.material.diffuse = np.array(list(self.specs.color), dtype=np.float32)


class Ring(Visual):

    def __init__(self, params: RingSpecs):

        super().__init__()

        self.params = params

        self._build()

    def _build(self):

        vertices, normals = build_rotation_ring(self.params)
        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)

        self.indices = np.arange(len(self.vertices), dtype=np.uint32)

        self.color = np.array(self.params.color)


class Plane(Visual):

    def __init__(self, params: PlaneSpecs):

        super().__init__()

        self.params = params

        self._build()

    def _build(self):

        vertices, normals = build_plane(self.params)
        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)

        self.indices = np.arange(len(self.vertices), dtype=np.uint32)

        # self.color = np.array(self.params.color)
        self.colors = np.tile(self.params.color, (len(self.vertices), 1))

