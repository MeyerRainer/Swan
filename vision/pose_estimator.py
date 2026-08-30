""" Class for ArUco-based Pose-estimation.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass
from typing import Optional, List, Sequence
import cv2
import numpy as np
from robot_math.pose import Pose
from vision.opencv_camera import CameraCalibration


@dataclass
class PoseEstimatorParams:
    camera_calibration: CameraCalibration
    id1: int
    id2: int
    marker_size: float
    marker_gap: float


@dataclass
class PoseEstimatorOutput:
    marker_pose: Optional[Pose] = None
    corners: Optional[Sequence[np.ndarray]] = None
    ids: Optional[np.ndarray] = None
    r_vec: Optional[np.ndarray] = None
    t_vec: Optional[np.ndarray] = None


class PoseEstimator:

    def __init__(self, params: PoseEstimatorParams):

        # Cameras intrinsic parameters.
        self.camera_calibration: CameraCalibration = params.camera_calibration

        # ArUco parameters.
        self.ids: List[int] = [params.id1, params.id2]
        self.marker_size: float = params.marker_size
        self.marker_gap: float = params.marker_gap

        # ArUco fiducial dictionary and board.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.GridBoard((1, 2), self.marker_size, self.marker_gap, self.aruco_dict, ids=np.array(self.ids))

        # ArUco detector and its parameters.
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def estimate_pose(self, frame: np.ndarray) -> Optional[PoseEstimatorOutput]:

        if self.camera_calibration.matrix is None:
            return None

        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:
            # Process only ArUco's of desired ID's.
            mask = np.isin(ids.flatten(), self.ids)
            corners_desired = [corners[i] for i in range(len(corners)) if mask[i]]
            ids_desired = ids[mask]
            if len(ids_desired) < 1:
                return None

            # Match detected markers to our board layout
            obj_points, img_points = self.board.matchImagePoints(corners_desired, ids_desired)

            if len(obj_points) > 0:
                ret, r_vec, t_vec = cv2.solvePnP(obj_points, img_points, self.camera_calibration.matrix,
                                                 self.camera_calibration.distortion, flags=cv2.SOLVEPNP_IPPE)
                if ret and t_vec[2] > 0:  # Reject behind the camera pose
                    # Rotation matrix from rotation vector representation.
                    R, _ = cv2.Rodrigues(r_vec)
                    # Detected ArUco's in camera frame, i.e., pose of ArUco with respect to camera
                    marker_pose: Pose = Pose.from_rot_mat(t_vec.flatten(), R)  # camera2aruco

                    return PoseEstimatorOutput(marker_pose = marker_pose, corners=corners, ids=ids, r_vec = r_vec, t_vec = t_vec)

        return None
