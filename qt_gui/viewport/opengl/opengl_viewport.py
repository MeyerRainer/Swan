""" OpenGL widget for 3d viewport.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from typing import override
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QMouseEvent, QWheelEvent
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

from qt_gui.viewport.scene.scene_camera_controller import CameraController
from qt_gui.viewport.opengl.render.renderer import SceneRenderer
from qt_gui.viewport.gizmo.gizmo_controller import GizmoController
from qt_gui.viewport.scene.scene_camera import Camera
from qt_gui.viewport import cg_math


class OpenGLViewport(QOpenGLWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.camera = Camera()
        self.camera_controller = CameraController(self.camera, parent=self)
        self.renderer = SceneRenderer()
        self.gizmo_controller = GizmoController()
        self.scene_root = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)

    def mousePressEvent(self, event: QMouseEvent):
        ray_origin, ray_dir = cg_math.get_mouse_ray(viewport=self, pos=event.position())
        self.camera_controller.handle_mouse_press(event)
        self.gizmo_controller.handle_mouse_press(ray_origin, ray_dir, self.camera.get_camera_position())

    def mouseMoveEvent(self, event: QMouseEvent):
        ray_origin, ray_dir = cg_math.get_mouse_ray(viewport=self, pos=event.position())
        # Gizmo move
        if self.gizmo_controller.is_active:
            self.gizmo_controller.handle_mouse_move(ray_origin, ray_dir)
        # Orbit
        else:
            self.camera_controller.handle_mouse_move(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        self.camera_controller.handle_mouse_release(event)
        self.gizmo_controller.handle_mouse_release()

    def wheelEvent(self, event: QWheelEvent):
        self.camera_controller.handle_wheel(event)

    def set_scene_graph_root(self, root):
        self.scene_root = root
        self.gizmo_controller.set_scene_graph_root(root)

    @override
    def initializeGL(self):
        self.renderer.initialize()

    @override
    def resizeGL(self, w: int, h: int) -> None:
        """ Called when widget is resized.
        """
        self.renderer.resize(w, h)

    def get_matrices(self):
        aspect_ratio: float = self.width() / max(self.height(), 1)
        return self.camera.get_projection_matrix(aspect_ratio), self.camera.get_view_matrix()

    @override
    def paintGL(self) -> None:
        """ Render image on screen.
        """
        # Set context.
        aspect_ratio: float = self.width() / max(self.height(), 1)
        self.renderer.context.projection_matrix = self.camera.get_projection_matrix(aspect_ratio)
        self.renderer.context.view_matrix = self.camera.get_view_matrix()
        self.renderer.context.camera_position = self.camera.get_camera_position()
        # Draw.
        self.renderer.render_scene(self.scene_root)

    def cleanupGL(self):
        """On shutdown.
        """
        self.makeCurrent()
        self.renderer.cleanup()
        self.doneCurrent()
