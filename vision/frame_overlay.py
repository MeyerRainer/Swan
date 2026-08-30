""" Class for drawing overlay onto frame.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from typing import Optional

import cv2
import numpy as np
from tic_tac_toe.ttt_detector import DetectorOutput
from vision.opencv_camera import CameraCalibration
from vision.pose_estimator import PoseEstimatorOutput


class Overlay:

    def __init__(self):

        self.CLS_COLORS = [[63, 63, 255], [31, 255, 31], [255, 63, 63]]
        self.CLS_NAMES = ["O", "Board", "X"]

    def draw(self, frame: np.ndarray,
             camera_calibration: CameraCalibration,
             ttt_detection: Optional[DetectorOutput] = None,
             board_pose_estimation: Optional[PoseEstimatorOutput] = None,) -> np.ndarray:

        # Tic-tac-toe overlay.
        if ttt_detection is not None:
            if len(ttt_detection.indices) > 0:
                for i in ttt_detection.indices:
                    cx, cy = ttt_detection.centers[i]
                    score = ttt_detection.confidences[i]
                    cls_id = ttt_detection.class_ids[i]
                    # Don't annotate game board.
                    if self.CLS_NAMES[cls_id] == "Board":
                        continue
                    color = self.CLS_COLORS[cls_id] if cls_id < len(self.CLS_COLORS) else [255, 255, 255]
                    cv2.circle(frame, (cx, cy), radius=6, color=color, thickness=-1)
                    label = f"{self.CLS_NAMES[cls_id] if cls_id < len(self.CLS_NAMES) else cls_id}: {score:.2f}"
                    cv2.putText(frame, label, (cx + 10, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ArUco overlay.
        if board_pose_estimation is not None:
            cv2.aruco.drawDetectedMarkers(frame, board_pose_estimation.corners, board_pose_estimation.ids)
            r_vec, t_vec = board_pose_estimation.marker_pose.rodrigues
            cv2.drawFrameAxes(frame, camera_calibration.matrix, camera_calibration.distortion, r_vec, t_vec, 0.020)

        return frame
