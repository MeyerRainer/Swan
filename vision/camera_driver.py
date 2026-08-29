""" Higher lever camera driver.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer
from vision.opencv_camera import OpenCVCamera


class CameraDriver(QObject):

    sgn_frame_received = pyqtSignal(object)
    sgn_error = pyqtSignal(str)
    sgn_message = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self.camera = OpenCVCamera()                # Camera backend.

        # Callbacks
        self.camera.on_message = self.sgn_message.emit
        self.camera.on_error = self.sgn_error.emit

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._capture)

    @pyqtSlot()
    def start(self) -> None:
        if not self.camera.connect():
            self.sgn_error.emit("Could not open camera.")
            return

        self._timer.start(int(1000 / self.camera.fps))
        self.sgn_message.emit("Camera started.")

    @pyqtSlot()
    def stop(self) -> None:
        self._timer.stop()
        self.camera.disconnect()
        self.sgn_message.emit(f"Camera stopped.")

    @pyqtSlot()
    def _capture(self) -> None:
        frame = self.camera.capture()
        if frame is None:
            self.sgn_error.emit(f"Could not capture frame.")
            return

        self.sgn_frame_received.emit(frame)
