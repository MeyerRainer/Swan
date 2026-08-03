from qt_gui.viewport.scene_camera import Camera
from qt_gui.viewport.scene_camera_controller import CameraController
from qt_gui.viewport.visuals.gizmo import *

from typing import override
import math
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QSurfaceFormat, QMatrix4x4, QVector3D, QVector2D
from qt_gui.viewport.renderer import SceneRenderer

from PyQt6.QtOpenGLWidgets import QOpenGLWidget

class OpenGLViewport(QOpenGLWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.camera = Camera()
        self.camera = CameraController(self.camera)
        self.renderer = SceneRenderer()

        # Camera State
        self.camera_radius = 0.5
        self.camera_yaw = math.radians(315)
        self.camera_pitch = math.radians(30)

        self._scene_tree_provider = None  # Callable or reference to get root node

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)

    def set_scene_provider(self, provider_func):
        """Injects a callback or reference to fetch the root SceneNode."""
        self._scene_tree_provider = provider_func

    def get_matrices(self) -> Tuple[QMatrix4x4, QMatrix4x4]:
        ratio = self.width() / max(1.0, self.height())
        projection = QMatrix4x4()
        projection.perspective(45.0, ratio, 0.01, 2.0)

        cam_coords: QVector3D = self.get_camera_position()

        view = QMatrix4x4()
        view.lookAt(cam_coords, QVector3D(0, 0, 0), QVector3D(0, 0, 1))

        return projection, view

    def get_camera_position(self) -> QVector3D:
        """ Converts cameras spherical to cartesian coordinates in world space """
        return QVector3D(
            self.camera_radius * math.cos(self.camera_pitch) * math.cos(self.camera_yaw),
            self.camera_radius * math.cos(self.camera_pitch) * math.sin(self.camera_yaw),
            self.camera_radius * math.sin(self.camera_pitch)
        )

    # --- Qt OpenGL Lifecycle Overrides ---

    @override
    def initializeGL(self):
        """Called once by Qt when the context is initialized."""
        self.renderer.initialize()

    @override
    def resizeGL(self, w: int, h: int):
        """Called when widget is resized."""
        self.renderer.resize(w, h)

    @override
    def paintGL(self):
        """THE ENDPOINT: Called by Qt whenever rendering is required."""
        root_node = self._scene_tree_provider() if self._scene_tree_provider else None
        matrices = self.get_matrices()
        self.renderer.render_scene(root_node, matrices)

    def cleanupGL(self):
        """Called when widget context is shutting down."""
        self.makeCurrent()
        self.renderer.cleanup()
        self.doneCurrent()