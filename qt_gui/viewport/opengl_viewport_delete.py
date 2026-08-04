from qt_gui.viewport.visuals.grid import Grid
from qt_gui.viewport.visuals.world_frame_origin import WorldFrameOrigin
from qt_gui.viewport.visuals.scene_object import *
from qt_gui.viewport.visuals.gizmo import *
# from qt_gui.viewport.scene import Scene
from qt_gui.viewport.opengl.shader import *

import qt_gui.viewport.cg_math as cg_math

from typing import override
import math
from OpenGL import GL
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QSurfaceFormat, QMatrix4x4, QVector3D, QVector2D

from PyQt6.QtOpenGLWidgets import QOpenGLWidget

class OpenGLViewport(QOpenGLWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        # Camera State
        self.camera_radius = 12.0
        self.camera_yaw = math.radians(315)
        self.camera_pitch = math.radians(30)
        self.last_mouse_pos = None

        self.body_frame = True

        # Scene Entities
        self.cube = None
        self.gizmo = None
        self.grid = None
        self.world_frame_origin = None

        # Rendering & Timers
        self.shader_program = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)

    @override
    def initializeGL(self):
        # OpenGL context
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.15, 0.15, 0.15, 1.0)

        # Scene Entities
        self.grid = Grid()
        self.world_frame_origin = WorldFrameOrigin()
        self.cube = SceneObject()
        self.gizmo = Gizmo()
        self.gizmo.attach_to(self.cube)

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

    def get_camera_position(self) -> QVector3D:
        """ Converts cameras spherical to cartesian coordinates in world space """
        return QVector3D(
            self.camera_radius * math.cos(self.camera_pitch) * math.cos(self.camera_yaw),
            self.camera_radius * math.cos(self.camera_pitch) * math.sin(self.camera_yaw),
            self.camera_radius * math.sin(self.camera_pitch)
        )

    def get_matrices(self):
        ratio = self.width() / max(1.0, self.height())
        projection = QMatrix4x4()
        projection.perspective(45.0, ratio, 0.1, 100.0)

        cam_coords: QVector3D = self.get_camera_position()

        view = QMatrix4x4()
        view.lookAt(cam_coords, QVector3D(0, 0, 0), QVector3D(0, 0, 1))

        return projection, view

    @override
    def paintGL(self):
        # Full clear on every frame
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram(self.shader_program)

        # Projection and camera matrices
        proj, view = self.get_matrices()  # QMatrix4x4

        proj_loc: int = GL.glGetUniformLocation(self.shader_program, "projection")
        view_loc: int = GL.glGetUniformLocation(self.shader_program, "view")
        model_loc: int = GL.glGetUniformLocation(self.shader_program, "model")

        GL.glUniformMatrix4fv(proj_loc, 1, GL.GL_FALSE, proj.data())
        GL.glUniformMatrix4fv(view_loc, 1, GL.GL_FALSE, view.data())

        # Draw elements
        self.grid.draw(model_loc)
        self.world_frame_origin.draw(model_loc)
        self.cube.draw(model_loc)
        self.gizmo.draw(model_loc, body_frame=self.body_frame)

    @override
    def resizeGL(self, w, h):
        dpr = self.devicePixelRatio()
        GL.glViewport(0, 0, int(w * dpr), int(h * dpr))

    # --- Mouse Handlers ---
    @override
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            ray_origin, ray_dir = cg_math.get_mouse_ray(viewport=self, pos=event.position())

            # Check if some gizmo has been hit
            hit_translation_axis: HandleType = self.gizmo.hit_translation_axis(ray_origin, ray_dir, body_frame=self.body_frame)
            hit_translation_plane: HandleType = self.gizmo.hit_translation_planes(ray_origin, ray_dir, body_frame=self.body_frame)
            hit_rotation_ring: HandleType = self.gizmo.hit_rotation_rings(ray_origin, ray_dir, body_frame=self.body_frame)

            # TODO: Add priority
            if hit_translation_axis != HandleType.NONE:
                axis = hit_translation_axis
            elif hit_translation_plane != HandleType.NONE:
                axis = hit_translation_plane
            elif hit_rotation_ring != HandleType.NONE:
                axis = hit_rotation_ring
            else:
                axis = HandleType.NONE

            # Gizmo motion
            if axis != HandleType.NONE:
                cam_pos = self.get_camera_position()
                self.gizmo.start_drag(ray_origin, ray_dir, cam_pos, axis, body_frame=self.body_frame)
            else:
                self.last_mouse_pos = event.position()

    @override
    def mouseMoveEvent(self, event):
        event_pos = event.position()

        # Gizmo motion
        if self.gizmo.DC.mode != HandleType.NONE:
            ray_origin, ray_dir = cg_math.get_mouse_ray(viewport=self, pos=event_pos)
            self.gizmo.update_drag(ray_origin, ray_dir, body_frame=self.body_frame)
            self.update()

        elif self.last_mouse_pos is not None:
            # Orbit
            delta = event_pos - self.last_mouse_pos
            self.last_mouse_pos = event_pos
            self.camera_yaw = self.constrain_yaw(self.camera_yaw - delta.x() * 0.01)
            self.camera_pitch += delta.y() * 0.01
            pitch_lim = math.radians(89)
            self.camera_pitch = max(-pitch_lim, min(pitch_lim, self.camera_pitch))

            self.update()

    @override
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.gizmo.DC.mode = HandleType.NONE
            self.last_mouse_pos = None

    @override
    def wheelEvent(self, event):
        zoom_delta = event.angleDelta().y() / 120.0
        self.camera_radius -= zoom_delta * 0.5
        self.camera_radius = max(3.0, min(50.0, self.camera_radius))

    @staticmethod
    def constrain_yaw(yaw: float | int) -> float | int:
        if yaw < 0:
            return yaw + 2*math.pi
        if yaw >= 2*math.pi:
            return yaw - 2*math.pi
        return yaw