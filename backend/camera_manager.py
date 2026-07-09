from PyQt6.QtCore import QObject, pyqtSignal
import cv2
import threading
import time
import numpy as np


class CameraManager(QObject):

    connected = pyqtSignal()
    disconnected = pyqtSignal()

    frame_received = pyqtSignal(np.ndarray)

    error = pyqtSignal(str)


    def __init__(self):
        super().__init__()

        self._cap = None
        self._thread = None
        self._running = False

        self._camera_index = 0
        self._fps = 30

    @property
    def is_connected(self):
        return self._running


    def connect(self, index=0, fps=30):

        if self._running:
            return True

        self._camera_index = index
        self._fps = fps

        self._cap = cv2.VideoCapture(index)

        if not self._cap.isOpened():
            self.error.emit("Could not open camera.")
            return False

        self._running = True

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)

        self._thread.start()

        self.connected.emit()

        return True


    def disconnect(self):

        self._running = False

        if self._thread:
            self._thread.join()

        if self._cap:
            self._cap.release()

        self._cap = None

        self.disconnected.emit()


    def _capture_loop(self):

        period = 1.0 / self._fps

        while self._running:

            start = time.perf_counter()

            ok, frame = self._cap.read()

            if ok:
                self.frame_received.emit(frame)

            elapsed = time.perf_counter() - start

            time.sleep(max(0, period - elapsed))