""" Vision system.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import pyqtSlot, pyqtSignal, QObject, QThread, Qt
from typing import Optional
import threading
import numpy as np

import config
from robot_math.pose import Pose
from vision.camera_driver import CameraDriver
from vision.pose_estimator import PoseEstimator, EstimatorParams


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

        self._camera_frame_available: bool = False
        self._camera_calibrated_intrinsic: bool = False
        self._camera_calibrated_extrinsic: bool = False
        # self._camera_thread.finished.connect(self._camera_driver.deleteLater)

        self._camera_driver.sgn_frame_received.connect(self._receive_frame)

        self._camera_driver.sgn_error.connect(self.sgn_error.emit)
        self._camera_driver.sgn_message.connect(self.sgn_message.emit)

        # Pose estimators
        self.game_board_pose_estimator: PoseEstimator = PoseEstimator(
            EstimatorParams(camera_calibration=self._camera_driver.camera.calibration,
                            id1=4, id2=42, marker_size=0.040, marker_gap=0.048))
        self.camera_pose_estimator: PoseEstimator = PoseEstimator(
            EstimatorParams(camera_calibration=self._camera_driver.camera.calibration,
                            id1=5, id2=43, marker_size=0.040, marker_gap=0.040))

        # Vision processing
        self._latest_frame = None
        self._condition = threading.Condition()
        self._processing_running: bool = False
        self._processing_thread: Optional[threading.Thread] = None

        self.world2camera: Optional[Pose] = None
        self.CAMERA_CALIBRATION_PATH: str = "camera_calibration"

    def camera_calibration_intrinsic(self):
        self._camera_calibrated_intrinsic = self._camera_driver.camera.calibrate(n_corners=(9, 6), image_path=self.CAMERA_CALIBRATION_PATH)

    def camera_calibration_extrinsic(self):
        if not self._camera_calibrated_intrinsic:
            self.sgn_error.emit("Camera not calibrated.")
            return
        # frame: np.ndarray = self._get_latest_frame()
        frame: np.ndarray = self._latest_frame
        if frame is None:
            self.sgn_error.emit("No camera frame.")
            return

        # Calibration board with respect to camera pose
        camera2aruco: Pose = self.camera_pose_estimator.estimate_pose(frame=frame)
        if camera2aruco is None:
            self.sgn_message.emit("Could not detect calibration markers.")
            return

        # Camera with respect to world pose.
        self.world2camera = Pose(config.WORLD2CAL_BOARD).compose(camera2aruco.inverse())
        self.sgn_message.emit(f"Calibration successful.")
        self.sgn_message.emit(f"Camera position in world frame: {1000*self.world2camera.position} millimeters.")
        self.sgn_message.emit(f"Camera ZYZ-Euler in world frame: {np.rad2deg(self.world2camera.zyz_euler)} degrees.")

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
    def _receive_frame(self, frame: np.ndarray) -> None:
        with self._condition:
            if not self._processing_running:
                return
            # Set latest frame and notify.
            self._camera_frame_available = True
            self._latest_frame = frame
            self._condition.notify_all()

    def _get_latest_frame(self) -> Optional[np.ndarray]:
        with self._condition:
            return self._latest_frame

    def _processing_loop(self) -> None:
        while True:
            with self._condition:
                while not self._camera_frame_available and self._processing_running:
                    self._condition.wait()

                if not self._processing_running:
                    break

                frame = self._latest_frame
                self._camera_frame_available = False
                # frame: np.ndarray = self._get_latest_frame()
                # self._latest_frame = None

            try:
                annotated, markers, pose = self._run_pipeline(frame)

                if markers is not None:
                    self.markers_detected.emit(markers)

                if pose is not None:
                    self.pose_estimated.emit(pose)

                self.processed_frame.emit(annotated)

            except Exception as exc:
                # self.sgn_error.emit(f"Vision processing error: {exc}")
                # TODO: Implement
                self.sgn_processed_frame.emit(frame)

    def _run_pipeline(self, frame: np.ndarray):
        # frame = self._undistort(frame)
        # markers = self._detect_markers(frame)
        # pose = self._estimate_pose(markers)
        # annotated = self._annotate_frame(frame, markers, pose)

        # print(f"Pipeline working.")

        markers = []
        pose = None
        annotated = frame

        return annotated, markers, pose