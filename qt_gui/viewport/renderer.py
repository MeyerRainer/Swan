from qt_gui.viewport.scene import SceneNode
from qt_gui.viewport.opengl.shader import *
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
        """Called once when the OpenGL context is valid."""
        # Compile shaders, set up lighting parameters, depth tests
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_CULL_FACE)
        # GL.glEnable(GL.GL_CULL_FACE)
        GL.glClearColor(0.15, 0.15, 0.15, 0.5)

        self.init_shaders()

    def init_shaders(self):
        v = GL.glCreateShader(GL.GL_VERTEX_SHADER)
        GL.glShaderSource(v, VERTEX_SHADER_SRC)
        GL.glCompileShader(v)

        f = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
        GL.glShaderSource(f, FRAGMENT_SHADER_SRC)
        GL.glCompileShader(f)

        print(f"Vertex shaders: {GL.glGetShaderInfoLog(v)}")
        print(f"Fragment shaders: {GL.glGetShaderInfoLog(f)}")

        self.shader_program = GL.glCreateProgram()

        GL.glAttachShader(self.shader_program, v)
        GL.glAttachShader(self.shader_program, f)
        GL.glLinkProgram(self.shader_program)

        GL.glDeleteShader(v)
        GL.glDeleteShader(f)

    def resize(self, width: int, height: int):
        """Called when viewport dimensions change."""
        GL.glViewport(0, 0, width, height)
        # Update projection matrices if needed

    def render_scene(self, root_node: SceneNode, matrices: Tuple[QMatrix4x4, QMatrix4x4]):

        # Full clear on every frame
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram(self.shader_program)

        if not root_node:
            return

        # 2. FIX: Create View & Projection matrices so geometry isn't clipped
        aspect_ratio = 1.0
        viewport = GL.glGetIntegerv(GL.GL_VIEWPORT)
        if viewport[3] > 0:
            aspect_ratio = viewport[2] / viewport[3]

        proj, view = matrices

        # Identity model matrix for root
        model = QMatrix4x4()

        GL.glUseProgram(self.shader_program)

        # Upload View & Projection once per render pass
        proj_loc: int = GL.glGetUniformLocation(self.shader_program, "projection")
        view_loc: int = GL.glGetUniformLocation(self.shader_program, "view")

        if proj_loc != -1:
            # QMatrix4x4.data() needs transpose=True for OpenGL column-major expectations
            GL.glUniformMatrix4fv(proj_loc, 1, GL.GL_FALSE, proj.data())
        if view_loc != -1:
            GL.glUniformMatrix4fv(view_loc, 1, GL.GL_FALSE, view.data())

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

        # Upload model
        model_loc = GL.glGetUniformLocation(self.shader_program, "model")
        if model_loc != -1:
            GL.glUniformMatrix4fv(model_loc, 1, GL.GL_TRUE, model_matrix.data())

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
        """Uploads NumPy arrays to VBO/VAO lazily on first render."""
        vao: int = GL.glGenVertexArrays(1)
        vbo: int = GL.glGenBuffers(1)
        ebo: int = GL.glGenBuffers(1)

        GL.glBindVertexArray(vao)

        # Upload Vertices
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, visual.vertices.nbytes, visual.vertices, GL.GL_STATIC_DRAW)

        # Upload Indices
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, visual.indices.nbytes, visual.indices, GL.GL_STATIC_DRAW)

        # Attribute 0: Position
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 0, None)
        GL.glEnableVertexAttribArray(0)

        GL.glBindVertexArray(0)

        print(f"Renderer: Visual uploaded")

        self._gpu_cache[visual_id] = {
            "vao": vao,
            "vbo": vbo,
            "ebo": ebo,
            "index_count": len(visual.indices)
        }

    def cleanup(self):
        """Free GPU resources when context is destroyed."""
        for handles in self._gpu_cache.values():
            GL.glDeleteVertexArrays(1, [handles["vao"]])
            GL.glDeleteBuffers(2, [handles["vbo"], handles["ebo"]])
        self._gpu_cache.clear()