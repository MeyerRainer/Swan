""" Vision system.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import types

from PyQt6.QtCore import pyqtSlot, pyqtSignal, QObject, QThread, Qt
import time
from typing import Optional
import threading
import numpy as np

from robot_math.pose import Pose
from tic_tac_toe.ttt_board import TTTBoard, MachineMoveRequest
from tic_tac_toe.ttt_detector import DetectorOutput, TTTDetector
from vision.camera_driver import CameraDriver
from vision.pose_estimator import PoseEstimator, PoseEstimatorParams, PoseEstimatorOutput
from vision.frame_overlay import Overlay
import config


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

        self._camera_driver.sgn_frame_received.connect(self._receive_frame)
        self._camera_driver.sgn_error.connect(self.sgn_error.emit)
        self._camera_driver.sgn_message.connect(self.sgn_message.emit)

        # Pose estimations
        self.game_board_pose_estimator: PoseEstimator = PoseEstimator(
            PoseEstimatorParams(camera_calibration=self._camera_driver.camera.calibration,
                                id1=4, id2=42, marker_size=0.040, marker_gap=0.048))
        self.camera_pose_estimator: PoseEstimator = PoseEstimator(
            PoseEstimatorParams(camera_calibration=self._camera_driver.camera.calibration,
                                id1=5, id2=43, marker_size=0.040, marker_gap=0.040))

        self.ttt_detector: TTTDetector = TTTDetector("models/tic_tac_toe/train-7/best.onnx")  # YOLO-based detector.
        self.ttt_board: TTTBoard = TTTBoard()       # Game board.
        self.frame_overlay = Overlay()              # Frame overlay.

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
        pose_estimation: PoseEstimatorOutput = self.camera_pose_estimator.estimate_pose(frame=frame)
        if pose_estimation is None:
            self.sgn_message.emit("Could not detect calibration markers.")
            return

        aruco2camera: Pose = pose_estimation.marker_pose.inverse()

        # Camera with respect to world pose.
        self.world2camera = Pose(config.WORLD2CAL_BOARD).compose(aruco2camera)
        # self._camera_driver.calibration.extrinsic = self.world2camera
        self._camera_driver.camera.calibration.extrinsic = self.world2camera
        self.sgn_message.emit(f"Calibration successful.")
        self.sgn_message.emit(f"Camera position in world frame: {np.round(1000*self.world2camera.position, 1)} millimeters.")
        self.sgn_message.emit(f"Camera ZYZ-Euler in world frame: {np.round(np.rad2deg(self.world2camera.zyz_euler), 1)} degrees.")

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

    def _processing_loop(self) -> None:
        while True:
            with self._condition:
                while not self._camera_frame_available and self._processing_running:
                    self._condition.wait()

                if not self._processing_running:
                    break

                frame = self._latest_frame
                self._camera_frame_available = False

            try:
                # Game board pose detection.
                camera2board: PoseEstimatorOutput = self.game_board_pose_estimator.estimate_pose(frame)
                # Tic-tac-toe detection.
                game_state_detection: DetectorOutput = self.ttt_detector.detect(frame)

                # Frame annotation.
                annotated: np.ndarray = self.frame_overlay.draw(frame=frame,
                                                                camera_calibration=self._camera_driver.camera.calibration,
                                                                ttt_detection=game_state_detection,
                                                                board_pose_estimation=camera2board)

                # Update game
                if camera2board is not None and game_state_detection is not None and self._camera_driver.camera.calibration.extrinsic is not None:
                    self.ttt_board.pose = self._camera_driver.camera.calibration.extrinsic.compose(camera2board.marker_pose)  # World2board.

                    machine_move_request = self.ttt_board.update(
                        camera_calibration=self._camera_driver.camera.calibration,
                        ttt_detection=game_state_detection,
                        board_pose_estimation = camera2board
                    )
                    # if machine_move_request is not None:
                    #     self.sgn_machine_move_request.emit(machine_move_request)

                self.sgn_processed_frame.emit(annotated)

            except Exception as exc:
                tr = exc.__traceback__
                # self.sgn_error.emit(f"Vision processing error: {exc.with_traceback(exc.__traceback__)}")
                raise RuntimeError().with_traceback(tr)
                # TODO: Implement
                self.sgn_processed_frame.emit(frame)

            finally:
                time.sleep(0.5)  # Limit speed to 5Hz

    def command_robot_sys(self):
        ...

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