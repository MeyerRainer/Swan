import OpenGL.GL as GL
import numpy as np
from PyQt6.QtGui import QMatrix4x4


class WorldFrameOrigin:
    """Renders thin X (Red), Y (Green), and Z (Blue) unit lines at World (0,0,0)."""

    def __init__(self, axis_length=1.0):
        self.axis_length = axis_length
        self.vao = None
        self.vertex_count = 6

        self.init_mesh_()

    def init_mesh_(self):
        # 3 lines starting at (0,0,0) extending along X, Y, and Z
        l = self.axis_length
        verts = [
            # X-axis (Red)
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            l, 0.0, 0.0, 1.0, 0.0, 0.0,
            # Y-axis (Green)
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, l, 0.0, 0.0, 1.0, 0.0,
            # Z-axis (Blue)
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, l, 0.0, 0.0, 1.0,
        ]

        data = np.array(verts, dtype=np.float32)

        self.vao = GL.glGenVertexArrays(1)
        vbo = GL.glGenBuffers(1)

        GL.glBindVertexArray(self.vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)

        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(0))
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(12))
        GL.glEnableVertexAttribArray(1)

    def draw(self, model_loc):
        # Set line thickness slightly wider for the origin tripod
        identity = QMatrix4x4()
        GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, identity.data())

        GL.glBindVertexArray(self.vao)
        GL.glDrawArrays(GL.GL_LINES, 0, self.vertex_count)
