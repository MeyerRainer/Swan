""" Camera backend for OpenCV Python camera.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, Callable
import glob


@dataclass
class CameraCalibration:
    matrix: Optional[np.ndarray] = None
    distortion: Optional[np.ndarray] = None
    quality: Optional[float] = None


class OpenCVCamera:

    def __init__(self, index: int = 0, fps: int = 30):

        self._index: int = index
        self._fps: int = fps

        self._cap: Optional[cv2.VideoCapture] = None
        self._calibration = CameraCalibration()

        # Callbacks
        self.on_frame: Optional[Callable[[np.ndarray], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None
        self.on_message: Optional[Callable[[str], None]] = None

    @property
    def calibration(self) -> CameraCalibration:
        return self._calibration

    @property
    def fps(self) -> int:
        return self._fps

    @fps.setter
    def fps(self, fps: int) -> None:
        if fps <= 0:
            raise ValueError("FPS must be greater than zero.")
        self._fps = fps

    @calibration.setter
    def calibration(self, value: CameraCalibration) -> None:
        self._calibration = value

    def connect(self) -> bool:
        self._cap = cv2.VideoCapture(self._index)
        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            return False
        return True

    def disconnect(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._cap = None

    def capture(self) -> Optional[np.ndarray]:
        if self._cap is None or not self._cap.isOpened():
            return None

        # Capture frame.
        ok, frame = self._cap.read()

        # Send frame or error.
        if not ok:
            return None

        return frame

    def calibrate(self, n_corners: Tuple[int, int], image_path: str) -> bool:
        # N*3 tall matrix of object points (Real world points)
        grid = np.zeros((n_corners[0] * n_corners[1], 3), dtype=np.float32)

        # Fill two first columns with grid
        grid[:, :2] = np.mgrid[0:n_corners[0], 0:n_corners[1]].T.reshape(-1, 2)

        obj_points = []  # 3D points
        img_points = []  # 2D points
        img_size = None

        images = glob.glob(f"{image_path}/*.png")  # folder with your chessboard images

        num_images: int = len(images)
        num_processed_images: int = 0

        for f_name in images:
            if self.on_message:
                self.on_message(f"Processing image {f_name}...")

            img = cv2.imread(f_name)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Detect corners in image
            ret, corners = cv2.findChessboardCorners(gray, n_corners, None)
            if not ret and self.on_error:
                self.on_error(f"Could not extract corners on image \"{f_name}\"")
                continue

            obj_points.append(grid)
            img_points.append(corners)
            img_size = gray.shape
            num_processed_images += 1

        # Compute:
        # ret, mtx, = calibration accuracy return value, camera matrix
        # dist = distortion coefficients (k1, k2, p1, p2, k3) (k for radial, p for tangential distortion)
        # rot_vecs, trans_vecs = rotation vectors, translation vectors (Orientation and position of chessboard)
        ret, mtx, dist, rot_vecs, trans_vecs = cv2.calibrateCamera(obj_points, img_points, img_size[::-1], None, None)

        if ret < 1 and self.on_error:
            self.on_error("Bad camera calibration.")
            return False

        self._calibration.matrix = mtx
        self._calibration.distortion = dist
        self._calibration.quality = ret
        if self.on_message:
            self.on_message(f"Camera successfully calibrated with return value {ret}. Processed {num_processed_images}/{num_images} images.")

        return True

