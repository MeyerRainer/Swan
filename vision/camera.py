import time
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot, QThread, QTimer

from vision.opencv_camera import OpenCVCamera


class CameraWorker(QObject):

    # Signals.
    sgn_frame_received = pyqtSignal(object)
    sgn_error = pyqtSignal(str)
    sgn_message = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self.camera = OpenCVCamera()
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


# class CameraWorker(QObject):
#
#     # Signals sent across thread boundaries to the GUI
#     sgn_frame_received = pyqtSignal(object)
#     sgn_error = pyqtSignal(str)
#     sgn_message = pyqtSignal(str)
#
#     _internal_start = pyqtSignal()
#
#     def __init__(self):
#
#         super().__init__()
#
#         self.camera: OpenCVCamera = OpenCVCamera()
#         self._running = False
#
#         # Bind camera callbacks directly to signal emission methods.
#         self.camera.on_frame = self.sgn_frame_received.emit
#         self.camera.on_error = self.sgn_error.emit
#         self.camera.on_message = self.sgn_message.emit
#
#         # Capture loop thread.
#         self._thread = QThread()
#         self.moveToThread(self._thread)
#
#         # Internal connections.
#         self._internal_start.connect(self._run_loop)
#         self._thread.finished.connect(self._thread.deleteLater)
#         self._thread.start()
#
#     def start_capture(self) -> None:
#         if not self._running:
#             self._running = True
#             self._internal_start.emit()
#
#     def stop_capture(self):
#         self._running = False
#
#     def shutdown(self) -> None:
#         """Cleanly destroys thread on application exit."""
#         self.stop_capture()
#         self._thread.quit()
#         self._thread.wait()
#
#     @pyqtSlot()
#     def _run_loop(self) -> None:
#         """ Runs inside the dedicated QThread.
#         """
#         if not self.camera.connect():
#             self._running = False
#             return
#
#         period = 1.0 / self.camera.fps
#
#         while self._running:
#             start_time = time.perf_counter()
#
#             # Capture frame.
#             self.camera.capture()
#
#             # Sleep until next frame.
#             elapsed = time.perf_counter() - start_time
#             sleep_time = max(0.0, period - elapsed)
#             time.sleep(sleep_time)
#
#         self.camera.disconnect()
