from PyQt6.QtCore import QObject, pyqtSignal
import threading
import numpy as np
import time
import cv2
from vision.opencv_camera import OpenCVCamera


class VisionManager(QObject):

    processed_frame = pyqtSignal(np.ndarray)

    pose_estimated = pyqtSignal(object)

    markers_detected = pyqtSignal(list)


    def __init__(self):

        super().__init__()

        self.camera = OpenCVCamera()

        self._latest_frame = None

        self._lock = threading.Lock()

        self._running = True

        self._thread = threading.Thread(target=self._processing_loop, daemon=True)

        self._thread.start()


    def process_frame(self, frame):

        with self._lock:
            self._latest_frame = frame.copy()

    @staticmethod
    def _annotate_frame(frame: np.ndarray, corners: np.ndarray, ids: np.ndarray) -> None:

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.drawFrameAxes(frame, camera_matrix, camera_dist, r_vec, t_vec, 0.050)


    def stop(self):

        self._running = False

        self._thread.join()


    def _processing_loop(self):

        while self._running:

            frame = None

            with self._lock:

                if self._latest_frame is not None:

                    frame = self._latest_frame
                    self._latest_frame = None

            if frame is None:

                time.sleep(0.002)
                continue

            annotated = self._run_pipeline(frame)

            self.processed_frame.emit(annotated)

    def _run_pipeline(self, frame: np.ndarray):

        # frame = self.undistort(frame)        #
        # markers = self.detect_markers(frame)        #
        # pose = self.estimate_pose(markers)        #
        # annotated = self.draw(frame, markers, pose)        #
        # self.markers_detected.emit(markers)
        # self.pose_estimated.emit(pose)




        return frame
