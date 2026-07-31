from robot_math.pose import Pose

import numpy as np
from OpenGL import GL
from PyQt6.QtGui import QSurfaceFormat, QMatrix4x4, QVector3D, QVector4D, QQuaternion

from robot_math.quaternion import Quaternion


class SceneObject:
    """Represents any renderable 3D entity in the scene with a 4x4 Pose Matrix."""

    def __init__(self):

        self.pose = Pose.identity()
        self.scale = QVector3D(1.0, 1.0, 1.0)

        self.vao = None
        self.index_count = 0
        self.init_mesh()

    def get_position(self) -> QVector3D:
        return QVector3D(*self.pose.position)

    def set_position(self, pos: QVector3D):
        self.pose.position = np.array([pos.x(), pos.y(), pos.z()])

    def set_quaternion(self, quat: QQuaternion):
        quat_vec: QVector4D = quat.toVector4D()
        self.pose.quaternion = Quaternion.from_iterable([quat_vec.w(), quat_vec.x(), quat_vec.y(), quat_vec.z()])

    def model_matrix(self) -> QMatrix4x4:
        m = QMatrix4x4(self.pose.SE3.flatten())
        m.scale(self.scale)
        return m

    def init_mesh(self):
        vertices = [
            -0.5, -0.5, 0.5, 0.5, 0.5, 0.5,
            0.5, -0.5, 0.5, 0.5, 0.5, 0.5,
            0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
            -0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
            -0.5, -0.5, -0.5, 0.5, 0.5, 0.5,
            0.5, -0.5, -0.5, 0.5, 0.5, 0.5,
            0.5, 0.5, -0.5, 0.5, 0.5, 0.5,
            -0.5, 0.5, -0.5, 0.5, 0.5, 0.5
        ]
        indices = [
            0, 1, 2, 2, 3, 0,
            1, 5, 6, 6, 2, 1,
            7, 6, 5, 5, 4, 7,
            4, 0, 3, 3, 7, 4,
            4, 5, 1, 1, 0, 4,
            3, 2, 6, 6, 7, 3
        ]

        v_data = np.array(vertices, dtype=np.float32)
        i_data = np.array(indices, dtype=np.uint32)
        self.index_count = len(indices)

        # Ask the OpenGL driver to reserve unique integer names for three objects
        self.vao: int = GL.glGenVertexArrays(1)
        vbo: int = GL.glGenBuffers(1)
        ebo: int = GL.glGenBuffers(1)


        # Binding a VAO activates it as the active context state. Any subsequent buffer bindings(GL_ELEMENT_ARRAY_BUFFER)
        # and attribute pointers (glVertexAttribPointer) are stored directly inside this VAO context.
        GL.glBindVertexArray(self.vao)
        # Copy Vertices to GPU Memory (glBufferData on VBO)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, v_data.nbytes, v_data, GL.GL_STATIC_DRAW)
        # Copy Indices to GPU Memory (glBufferData on EBO)
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, i_data.nbytes, i_data, GL.GL_STATIC_DRAW)
        # The GPU receives a continuous blob of raw memory bytes. It doesn't know where a position ends and a normal
        # begins. glVertexAttribPointer instructs the vertex shader stage how to parse the interleaved array structure
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(0))
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, GL.GLvoidp(12))
        GL.glEnableVertexAttribArray(1)

    def draw(self, model_loc):
        GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, self.model_matrix().data())
        GL.glBindVertexArray(self.vao)
        GL.glDrawElements(GL.GL_TRIANGLES, self.index_count, GL.GL_UNSIGNED_INT, None)