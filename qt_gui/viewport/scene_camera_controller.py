""" Camera controller for scene 3d scene.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import QObject, Qt
from PyQt6.QtGui import QMouseEvent, QWheelEvent

from qt_gui.viewport.scene_camera import Camera


class CameraController(QObject):

    def __init__(self, camera: Camera, parent: QWidget | None = None):

        super().__init__()

        self.camera = camera
        self.opengl_widget = parent
        self._last_mouse_pos = None

    def handle_mouse_press(self, event: QMouseEvent):
        self._last_mouse_pos = event.position()

    def handle_mouse_move(self, event: QMouseEvent):

        if self._last_mouse_pos is None:
            return

        # Pixel space.
        delta = event.position() - self._last_mouse_pos
        self._last_mouse_pos = event.position()

        # Orbit. (Left Click or Alt + Left Click)
        if event.buttons() & Qt.MouseButton.RightButton:
            # pass
            self.camera.rotate(delta_azimuth=-delta.x() * 0.01, delta_elevation = delta.y() * 0.01)
            self.opengl_widget.update()

        # Pan. (Middle Click or Right Click)
        # elif event.buttons() & (Qt.MouseButton.MiddleButton | Qt.MouseButton.RightButton):
        elif event.buttons() & Qt.MouseButton.LeftButton:
            # pass
            self.camera.pan(dx=delta.x(), dy=delta.y())
            self.opengl_widget.update()

    def handle_mouse_release(self, event: QMouseEvent):
        self._last_mouse_pos = None

    def handle_wheel(self, event: QWheelEvent):
        delta = event.angleDelta().y()
        self.camera.zoom(delta)
        self.opengl_widget.update()
