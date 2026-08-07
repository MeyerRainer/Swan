""" Scene graph renderer.

Author: Rainer Meyer, r.meyer494@gmail.com
"""


from qt_gui.viewport.scene import SceneNode
# from qt_gui.viewport.opengl.shader import *
from qt_gui.viewport.opengl.shaders_advanced import *
from qt_gui.viewport.visuals.visual import Visual

from OpenGL import GL
from PyQt6.QtGui import QMatrix4x4, QVector3D
from typing import Tuple
import numpy as np


class SceneRenderer:

    def __init__(self):

        self._gpu_cache = {}

        self.shader_program = None

    def initialize(self):
        """ Called once when the OpenGL current_context is valid.
        """
        # Compile shaders, set up lighting parameters, depth tests
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_CULL_FACE)
        # GL.glEnable(GL.GL_CULL_FACE)
        # Ambient color
        GL.glClearColor(0.184, 0.204, 0.247, 1.)

        self.init_shaders()

    def init_shaders(self):
        v = GL.glCreateShader(GL.GL_VERTEX_SHADER)
        GL.glShaderSource(v, VERTEX_SHADER_SRC)
        GL.glCompileShader(v)

        f = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
        GL.glShaderSource(f, FRAGMENT_SHADER_SRC)
        GL.glCompileShader(f)

        self.shader_program = GL.glCreateProgram()

        GL.glAttachShader(self.shader_program, v)
        GL.glAttachShader(self.shader_program, f)
        GL.glLinkProgram(self.shader_program)

        GL.glDeleteShader(v)
        GL.glDeleteShader(f)

    @staticmethod
    def resize(width: int, height: int):
        """Called when viewport dimensions change.
        """
        GL.glViewport(0, 0, width, height)
        # Update projection matrices if needed

    def render_scene(self, root_node: SceneNode, projection_matrix: QMatrix4x4, view_matrix: QMatrix4x4, camera_pos: QVector3D):

        # Full clear on every frame
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        if not root_node:
            return

        GL.glUseProgram(self.shader_program)

        # Upload View & Projection once per render pass
        proj_loc: int = GL.glGetUniformLocation(self.shader_program, "projection")
        view_loc: int = GL.glGetUniformLocation(self.shader_program, "view")

        if proj_loc != -1:
            GL.glUniformMatrix4fv(proj_loc, 1, GL.GL_FALSE, projection_matrix.data())
        if view_loc != -1:
            GL.glUniformMatrix4fv(view_loc, 1, GL.GL_FALSE, view_matrix.data())

        # Upload Camera Position for Specular and Fresnel computations
        view_pos_loc = GL.glGetUniformLocation(self.shader_program, "viewPos")
        if view_pos_loc != -1:
            # GL.glUniform3f(view_pos_loc, *cam_pos)
            GL.glUniform3f(view_pos_loc, camera_pos.x(), camera_pos.y(), camera_pos.z())

        # Identity model matrix for root
        model = QMatrix4x4()
        self._draw_node(root_node, parent_transform=model)

    def _draw_node(self, node: SceneNode, parent_transform: QMatrix4x4):
        if not node.visible:
            return

        world_transform = QMatrix4x4(parent_transform)

        if node.visual and len(node.visual.vertices) > 0:
            self._render_visual(node.visual, world_transform)

        for child in node.children:
            self._draw_node(child, world_transform)

    def _render_visual(self, visual: Visual, model_matrix: QMatrix4x4):
        vis_id = id(visual)
        if vis_id not in self._gpu_cache:
            self._upload_visual(vis_id, visual)

        gpu_data = self._gpu_cache[vis_id]

        # Upload model matrix (4x4)
        model_loc = GL.glGetUniformLocation(self.shader_program, "model")
        if model_loc != -1:
            GL.glUniformMatrix4fv(model_loc, 1, GL.GL_TRUE, model_matrix.data())

        # Compute and Upload Normal Matrix (3x3)
        normal_matrix_loc = GL.glGetUniformLocation(self.shader_program, "normalMatrix")
        if normal_matrix_loc != -1:
            normal_matrix = model_matrix.normalMatrix()
            GL.glUniformMatrix3fv(normal_matrix_loc, 1, GL.GL_FALSE, normal_matrix.data())

        loc = GL.glGetUniformLocation(self.shader_program, "diffuseColor")

        # Upload color
        if visual.material is not None:
            GL.glUniform3fv(loc, 1, visual.material.diffuse)
        else:
            GL.glUniform3f(loc, 0.8, 0.8, 0.8)

        # Bind VAO & Draw
        GL.glBindVertexArray(gpu_data["vao"])
        GL.glDrawElements(GL.GL_TRIANGLES, gpu_data["index_count"], GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(0)

    def _upload_visual(self, visual_id: int, visual: Visual):
        """ Uploads NumPy arrays to VBO/VAO lazily on first render.
        """
        # Generate automatic normals if visual.normals is empty
        normals = visual.normals
        if len(normals) == 0:
            normals = np.zeros_like(visual.vertices, dtype=np.float32)
            normals[:, 1] = 1.0  # Default up-vector fallback

        # Interleave vertices and normals: [x, y, z, nx, ny, nz, ...]
        vertex_data = np.hstack([visual.vertices, normals]).astype(np.float32)

        vao: int = GL.glGenVertexArrays(1)
        vbo: int = GL.glGenBuffers(1)
        ebo: int = GL.glGenBuffers(1)

        GL.glBindVertexArray(vao)

        # Upload Combined Vertex + Normal Data
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, vertex_data.nbytes, vertex_data, GL.GL_STATIC_DRAW)

        # Upload Indices
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, visual.indices.nbytes, visual.indices, GL.GL_STATIC_DRAW)

        stride = 6 * 4  # 6 floats total (3 position + 3 normal), 4 bytes per float
        # Attribute 0: Position
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, None)
        GL.glEnableVertexAttribArray(0)

        # Attribute 1: Normal (aNormal)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, GL.GLvoidp(12))  # 12 bytes offset
        GL.glEnableVertexAttribArray(1)

        GL.glBindVertexArray(0)

        print(f"Renderer: Visual uploaded")

        self._gpu_cache[visual_id] = {
            "vao": vao,
            "vbo": vbo,
            "ebo": ebo,
            "index_count": len(visual.indices)
        }

    def cleanup(self):
        """ Free GPU resources when current_context is destroyed.
        """
        for handles in self._gpu_cache.values():
            GL.glDeleteVertexArrays(1, [handles["vao"]])
            GL.glDeleteBuffers(2, [handles["vbo"], handles["ebo"]])
        self._gpu_cache.clear()