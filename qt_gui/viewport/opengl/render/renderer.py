""" Scene graph renderer.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from __future__ import annotations

from qt_gui.viewport.opengl.shader.shaders_advanced import *
from robot_math.pose import Pose

from OpenGL import GL
from PyQt6.QtGui import QMatrix4x4, QVector3D
from dataclasses import dataclass
from typing import Any
import numpy as np
import ctypes


@dataclass
class RenderContext:
    projection_matrix: QMatrix4x4
    view_matrix: QMatrix4x4
    camera_position: QVector3D
    renderer: "SceneRenderer"


class SceneRenderer:

    def __init__(self):

        self._gpu_cache = {}

        self.shader_program = None

        self.context = RenderContext(
            projection_matrix=QMatrix4x4(),
            view_matrix=QMatrix4x4(),
            camera_position=QVector3D(),
            renderer=self
        )

    def initialize(self) -> None:
        """ Called once when the OpenGL current_context is valid.
        """
        # Compile shaders, set up lighting parameters, depth tests
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_CULL_FACE)
        # GL.glEnable(GL.GL_CULL_FACE)

        # Enable Alpha Blending
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        # Ambient color
        GL.glClearColor(0.184, 0.204, 0.247, 1.)

        self.init_shaders()

    def init_shaders(self) -> None:
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
    def resize(width: int, height: int) -> None:
        """Called when viewport dimensions change.
        """
        GL.glViewport(0, 0, width, height)
        # Update projection matrices if needed

    # def render_scene(self, root_node: SceneNode, projection_matrix: QMatrix4x4, view_matrix: QMatrix4x4, camera_pos: QVector3D):
    def render_scene(self, tree) -> None:

        # Full clear on every frame
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram(self.shader_program)

        # Reserve locations for arrays in GPU.
        proj_loc: int = GL.glGetUniformLocation(self.shader_program, "projection")
        view_loc: int = GL.glGetUniformLocation(self.shader_program, "view")
        cam_loc: int = GL.glGetUniformLocation(self.shader_program, "viewPos")

        # Upload projection matrix to GPU.
        if proj_loc != -1:
            GL.glUniformMatrix4fv(proj_loc, 1, GL.GL_FALSE, self.context.projection_matrix.data())
        # Upload view matrix.
        if view_loc != -1:
            GL.glUniformMatrix4fv(view_loc, 1, GL.GL_FALSE, self.context.view_matrix.data())
        # Upload Camera Position for Specular and Fresnel computations.
        if cam_loc != -1:
            GL.glUniform3f(cam_loc, self.context.camera_position.x(), self.context.camera_position.y(), self.context.camera_position.z())

        # Render root.
        tree.render(render_context=self.context)

    def render_visual(self, visual: Any, pose: Pose):

        vis_id = id(visual)
        if vis_id not in self._gpu_cache:
            self._upload_visual(vis_id, visual)

        gpu_data = self._gpu_cache[vis_id]
        model_matrix = QMatrix4x4(pose.SE3.flatten().tolist())

        # Upload model matrix (4x4).
        model_loc = GL.glGetUniformLocation(self.shader_program, "model")
        if model_loc != -1:
            GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, model_matrix.data())

        # Compute and Upload Normal Matrix (3x3).
        normal_matrix_loc = GL.glGetUniformLocation(self.shader_program, "normalMatrix")
        if normal_matrix_loc != -1:
            normal_matrix = model_matrix.normalMatrix()
            GL.glUniformMatrix3fv(normal_matrix_loc, 1, GL.GL_FALSE, normal_matrix.data())

        # Bind vao and draw.
        GL.glBindVertexArray(gpu_data["vao"])
        # Triangle or line render.
        draw_mode = GL.GL_LINES  if visual.line_render else GL.GL_TRIANGLES
        GL.glDrawElements(draw_mode, gpu_data["index_count"], GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(0)

    def _upload_visual(self, visual_id: int, visual: Any):
        """ Uploads NumPy arrays to VBO/VAO lazily on first render.
        """
        # Generate automatic normals if visual.normals is empty
        normals = visual.normals
        if len(normals) == 0:
            normals = np.zeros_like(visual.vertices, dtype=np.float32)
            normals[:, 1] = 1.0  # Default up-vector fallback

        # Interleave vertices and normals: [x, y, z, nx, ny, nz]
        vertex_data = np.hstack([visual.vertices, normals]).astype(np.float32)

        # Generate IDs for render data arrays.
        vao: int = GL.glGenVertexArrays(1)
        vbo: int = GL.glGenBuffers(1)
        ebo: int = GL.glGenBuffers(1)

        GL.glBindVertexArray(vao)

        # Upload concatenated vertex and normal data.
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, vertex_data.nbytes, vertex_data, GL.GL_STATIC_DRAW)

        # Upload Indices.
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, visual.indices.nbytes, visual.indices, GL.GL_STATIC_DRAW)

        # Color fallback.
        if visual.colors is None:
            colors = np.full((len(visual.vertices), 4), [0.8, 0.8, 0.8, 1.0], dtype=np.float32)
            print(f"No color")
        else:
            colors = visual.colors

        # Combine into single interleaved array: [x, y, z, nx, ny, nz, r, g, b, a].
        vertex_data = np.hstack([visual.vertices, visual.normals, colors]).astype(np.float32)

        vao = GL.glGenVertexArrays(1)
        vbo = GL.glGenBuffers(1)
        ebo = GL.glGenBuffers(1)

        GL.glBindVertexArray(vao)

        # Upload combined vertex data to GPU.
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, vertex_data.nbytes, vertex_data, GL.GL_STATIC_DRAW)

        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, visual.indices.nbytes, visual.indices, GL.GL_STATIC_DRAW)

        # Stride: 10 floats * 4 bytes = 40 bytes.
        stride = 10 * 4

        # Position (3 floats, offset 0).
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(0)

        # Normal (3 floats, offset 12 bytes).
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(12))
        GL.glEnableVertexAttribArray(1)

        # Color (4 floats, offset 24 bytes).
        GL.glVertexAttribPointer(2, 4, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(24))
        GL.glEnableVertexAttribArray(2)

        GL.glBindVertexArray(0)

        # Cache render data.
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
