from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal
import threading
import numpy as np
import time
import cv2


class VisionManager(QObject):

    sgn_processed_frame = pyqtSignal(np.ndarray)
    sgn_pose_estimated = pyqtSignal(object)
    sgn_markers_detected = pyqtSignal(list)

    def __init__(self):

        super().__init__()

        self._latest_frame: Optional[np.ndarray] = None
        self._condition = threading.Condition()
        self._running = True

        self._thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._thread.start()

    def process_frame(self, frame):
        with self._condition:
            self._latest_frame = frame.copy()
            self._condition.notify()

    def stop(self):
        with self._condition:
            self._running = False
            self._condition.notify()

        self._thread.join()

    def _processing_loop(self):
        while True:
            with self._condition:
                while self._latest_frame is None and self._running:
                    self._condition.wait()

                if not self._running:
                    break

                frame = self._latest_frame
                self._latest_frame = None

            annotated = self._run_pipeline(frame)
            self.sgn_processed_frame.emit(annotated)

    @staticmethod
    def _run_pipeline(frame: np.ndarray):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



# class VisionManager(QObject):
#
#     processed_frame = pyqtSignal(np.ndarray)
#     pose_estimated = pyqtSignal(object)
#     markers_detected = pyqtSignal(list)
#
#     def __init__(self, camera):
#
#         super().__init__()
#
#         self._running = True
#         self.camera = camera
#         self._latest_frame: Optional[np.ndarray] = None
#         self._lock = threading.Lock()
#         self._frame_event = threading.Event()
#
#         self._thread = threading.Thread(target=self._processing_loop, daemon=True)
#         self._thread.start()
#
#     def process_frame(self, frame):
#         with self._lock:
#             self._latest_frame = frame.copy()
#         self._frame_event.set()
#
#     # @staticmethod
#     # def _annotate_frame(frame: np.ndarray, corners: np.ndarray, ids: np.ndarray) -> None:
#     #
#     #     cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#     #     cv2.drawFrameAxes(frame, camera_matrix, camera_dist, r_vec, t_vec, 0.050)
#
#     def stop(self):
#         self._running = False
#         self._frame_event.set()
#         self._thread.join()
#
#     def _processing_loop(self):
#
#         while self._running:
#             self._frame_event.wait()
#             if not self._running:
#                 break
#
#             with self._lock:
#                 frame = self._latest_frame
#                 self._latest_frame = None
#                 # If another frame arrived while we were processing,
#                 # it remains in _latest_frame.
#             if frame is None:
#                 continue
#
#             annotated = self._run_pipeline(frame)
#             self.processed_frame.emit(annotated)
#
#     def _run_pipeline(self, frame: np.ndarray) -> np.ndarray:
#
#         # frame = self.undistort(frame)        #
#         # markers = self.detect_markers(frame)        #
#         # pose = self.estimate_pose(markers)        #
#         # annotated = self.draw(frame, markers, pose)        #
#         # self.markers_detected.emit(markers)
#         # self.pose_estimated.emit(pose)
#
#         return frame
