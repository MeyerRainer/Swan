from typing import Optional

import cv2
import numpy as np

from robot_math.pose import Pose

MARKER_LENGTH = 0.040
MARKER_SEPARATION = 0.048

class Detector:

    def __init__(self, camera_matrix: np.ndarray, camera_dist: np.ndarray, id1: int, id2: int):

        # Camera matrix and distortion coefficients.
        self.camera_matrix: np.ndarray = camera_matrix
        self.camera_dist: np.ndarray = camera_dist

        # ArUco fiducial dictionary and board.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.GridBoard((1, 2), MARKER_LENGTH, MARKER_SEPARATION, self.aruco_dict, ids=np.array([[id1], [id2]]))

        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def estimate_pose(self, frame: np.ndarray) -> Optional[Pose]:

        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:
            # Match detected markers to our board layout
            # objPoints = 3D object points, imgPoints = 2D image points
            obj_points, img_points = self.board.matchImagePoints(corners, ids)

            if len(obj_points) > 0:
                print(f"Object points: {obj_points}")
                # print(f"Found {len(obj_points)} object points.")
                ret, r_vec, t_vec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.camera_dist, flags=cv2.SOLVEPNP_IPPE)
                if ret and t_vec[2] > 0:  # Reject behind the camera pose
                    # x, y, z = t_vec.flatten()
                    # rx, ry, rz = r_vec.flatten()
                    R, _ = np.ndarray = cv2.Rodrigues(r_vec)
                    pose: Pose = Pose.from_rot_mat(pos=t_vec, R=R)
                    # print(f"Detector, Pose: X: {x:.3f}\tY: {y:.3f}\tZ: {z:.3f}\tRX: {rx:.3f}\tRY: {ry:.3f}\tRZ: {rz:.3f}")
                    return Pose, corners, ids
                else:
                    print(f"Return value: {ret}")
                    if t_vec[2] < 0:
                        print(f"Pose behind the camera")

        return None
