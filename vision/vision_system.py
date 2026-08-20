from PyQt6.QtCore import pyqtSlot, pyqtSignal, QObject, QThread, Qt
import threading
import numpy as np
from vision.camera_driver import CameraDriver


class VisionSystem(QObject):

    _camera_start = pyqtSignal()
    _camera_stop = pyqtSignal()

    sgn_raw_frame = pyqtSignal(object)
    sgn_processed_frame = pyqtSignal(object)
    sgn_pose_estimated = pyqtSignal(object)
    sgn_markers_detected = pyqtSignal(list)

    sgn_error = pyqtSignal(str)
    sgn_message = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        # Camera subsystem
        self._camera_driver = CameraDriver()
        self._camera_thread = QThread()
        self._camera_driver.moveToThread(self._camera_thread)
        self._camera_thread.started.connect(self._camera_driver.start)
        self._camera_start.connect(self._camera_driver.start, Qt.ConnectionType.QueuedConnection)
        self._camera_stop.connect(self._camera_driver.stop, Qt.ConnectionType.QueuedConnection)
        # self._camera_thread.finished.connect(self._camera_driver.deleteLater)

        self._camera_driver.sgn_frame_received.connect(self._raw_frame)
        # self._camera_driver.sgn_frame_received.connect(self._process_frame)
        self._camera_driver.sgn_error.connect(self.sgn_error)
        self._camera_driver.sgn_message.connect(self.sgn_message)

        # Vision processing
        self._latest_frame = None
        self._condition = threading.Condition()
        self._processing_running = False
        self._processing_thread = None

    def load_files(self):
        ...

    def start(self) -> None:

        if self._camera_thread.isRunning():
            return

        with self._condition:
            self._processing_running = True

        self._processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._processing_thread.start()
        self._camera_thread.start()

    def stop(self) -> None:
        # Stop accepting/processing new frames.
        with self._condition:
            self._processing_running = False
            self._condition.notify()

        # Stop the camera worker.
        self._camera_driver.stop()
        if self._camera_thread.isRunning():
            self._camera_thread.quit()
            self._camera_thread.wait()

        # Wait for vision processing to finish.
        if self._processing_thread is not None:
            self._processing_thread.join()
            self._processing_thread = None

    @pyqtSlot(object)
    def _raw_frame(self, frame: np.ndarray) -> None:
        # print(f"VisionSys: _raw_frame called. Frame type: {type(frame)}")
        self.sgn_raw_frame.emit(frame)

    @pyqtSlot(object)
    def _process_frame(self, frame: np.ndarray) -> None:
        with self._condition:
            if not self._processing_running:
                return

            self._latest_frame = frame
            self._condition.notify()

    def _processing_loop(self) -> None:
        while True:
            with self._condition:
                while self._latest_frame is None and self._processing_running:
                    self._condition.wait()

                if not self._processing_running:
                    break

                frame = self._latest_frame
                self._latest_frame = None

            try:
                annotated, markers, pose = self._run_pipeline(frame)

                if markers is not None:
                    self.markers_detected.emit(markers)

                if pose is not None:
                    self.pose_estimated.emit(pose)

                self.processed_frame.emit(annotated)

            except Exception as exc:
                self.sgn_error.emit(f"Vision processing error: {exc}")

    def _run_pipeline(self, frame: np.ndarray):
        # frame = self._undistort(frame)
        # markers = self._detect_markers(frame)
        # pose = self._estimate_pose(markers)
        # annotated = self._annotate_frame(frame, markers, pose)

        markers = []
        pose = None
        annotated = frame

        return annotated, markers, pose