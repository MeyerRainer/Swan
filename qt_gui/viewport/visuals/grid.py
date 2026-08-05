""" Grid visual.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import OpenGL.GL as GL
import numpy as np
from PyQt6.QtGui import QMatrix4x4


class Grid:

  def __init__(self, size=10, step=1, color=(0.3, 0.3, 0.3)):
    """ Initializes and uploads grid geometry to GPU memory.

    Must be instantiated AFTER an active OpenGL current_context exists.
    """
    self.size = size
    self.step = step
    self.color = color

    self.vao = None
    self.vbo = None
    self.vertex_count = 0

    self._build_grid()

  def _build_grid(self):
    vertices = []
    r, g, b = self.color

    # Generate grid lines onto XY-plane
    for i in np.arange(-self.size, self.size + self.step, self.step):
      # Line parallel to Z-axis
      vertices.extend([i, -self.size, 0., r, g, b])
      vertices.extend([i, self.size, 0., r, g, b])
      # Line parallel to X-axis
      vertices.extend([-self.size, i, 0.0, r, g, b])
      vertices.extend([self.size, i, 0.0, r, g, b])

    data = np.array(vertices, dtype=np.float32)
    self.vertex_count = len(data) // 6

    # Create and bind GPU objects
    self.vao = GL.glGenVertexArrays(1)
    self.vbo = GL.glGenBuffers(1)

    GL.glBindVertexArray(self.vao)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
    GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)

    # Position Attribute (Location 0)
    GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(0))
    GL.glEnableVertexAttribArray(0)

    # Color Attribute (Location 1)
    GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(12))
    GL.glEnableVertexAttribArray(1)

    # Unbind VAO to prevent accidental modifications
    GL.glBindVertexArray(0)

  def draw(self, model_loc):
    """ Draws the grid using GL_LINES."""
    identity = QMatrix4x4()
    GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, identity.data())
    GL.glBindVertexArray(self.vao)
    GL.glDrawArrays(GL.GL_LINES, 0, self.vertex_count)
    # GL.glBindVertexArray(0)

  def destroy(self):
    """ Frees GPU memory buffers."""
    if self.vbo:
      GL.glDeleteBuffers(1, [self.vbo])
    if self.vao:
      GL.glDeleteVertexArrays(1, [self.vao])