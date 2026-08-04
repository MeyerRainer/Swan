""" OpenGL widget for 3d viewport.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from qt_gui.viewport.scene_camera import Camera
from qt_gui.viewport.scene_camera_controller import CameraController
from qt_gui.viewport.visuals.gizmo import *

from typing import override
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QSurfaceFormat, QMatrix4x4, QVector3D, QVector2D, QMouseEvent, QWheelEvent
from qt_gui.viewport.renderer import SceneRenderer

from PyQt6.QtOpenGLWidgets import QOpenGLWidget

class OpenGLViewport(QOpenGLWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.camera = Camera()
        self.camera_controller = CameraController(self.camera, parent=self)
        self.renderer = SceneRenderer()

        self._scene_tree_provider = None  # Callable or reference to get root node

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)

    def set_scene_provider(self, provider_func):
        # Function for fetching scene tree root.
        self._scene_tree_provider = provider_func

    def mousePressEvent(self, event: QMouseEvent):
        self.camera_controller.handle_mouse_press(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        self.camera_controller.handle_mouse_move(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        self.camera_controller.handle_mouse_release(event)

    def wheelEvent(self, event: QWheelEvent):
        self.camera_controller.handle_wheel(event)

    @override
    def initializeGL(self):
        self.renderer.initialize()

    @override
    def resizeGL(self, w: int, h: int) -> None:
        """ Called when widget is resized.
        """
        self.renderer.resize(w, h)

    @override
    def paintGL(self) -> None:
        """ Render image on screen.
        """
        root_node = self._scene_tree_provider() if self._scene_tree_provider else None
        aspect_ratio: float = self.width() / max(self.height(), 1)
        proj_mtx = self.camera.get_projection_matrix(aspect_ratio)
        view_mtx = self.camera.get_view_matrix()

        # Recursively render scene tree.
        self.renderer.render_scene(root_node, proj_mtx, view_mtx)

    def cleanupGL(self):
        """On shutdown.
        """
        self.makeCurrent()
        self.renderer.cleanup()
        self.doneCurrent()