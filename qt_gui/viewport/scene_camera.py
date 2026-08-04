""" Camera for 3d viewport scene.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtGui import QMatrix4x4, QVector3D
import math


class Camera:

    def __init__(self):

        self.target: QVector3D = QVector3D(0.0, 0.0, 0.0)  # Orbit pivot point.
        self.azimuth: float = math.radians(45.0)  # Direction in range [0, 2Pi].
        self.zenith: float = math.radians(30.0)  # Degrees up/down [-89, 89].
        self.distance: float = 0.5  # Distance from target, meters.

        self.fov: float = 45.0  # Field of view, degrees.
        self.near_plane: float = 0.01  # Back plane of frustum, meters.
        self.far_plane: float = 2.0  # Front plane of frustum, meters.

    def get_camera_position(self) -> QVector3D:
        """ Converts cameras spherical to cartesian coordinates in world space.
        :return: Camera position in cartesian space.
        """
        offset = QVector3D(
            self.distance * math.cos(self.zenith) * math.cos(self.azimuth),
            self.distance * math.cos(self.zenith) * math.sin(self.azimuth),
            self.distance * math.sin(self.zenith))

        return self.target + offset

    def get_projection_matrix(self, aspect_ratio: float) -> QMatrix4x4:
        proj_mtx = QMatrix4x4()
        proj_mtx.perspective(self.fov, max(aspect_ratio, 0.001), self.near_plane, self.far_plane)

        return proj_mtx

    def get_view_matrix(self) -> QMatrix4x4:
        cam_pos_cartesian: QVector3D = self.get_camera_position()
        view_mtx = QMatrix4x4()
        view_mtx.lookAt(cam_pos_cartesian, self.target, QVector3D(0, 0, 1))

        return view_mtx

    def constrain_azimuth(self) -> None:
        """ Constrains the azimuth angle of camera to range [0, 360] degrees.
        """
        if self.azimuth < 0:
            self.azimuth += 2*math.pi
        elif self.azimuth >= 2*math.pi:
            self.azimuth -= 2*math.pi

    def constrain_zenith(self, max_abs_elevation_deg: float) -> None:
        """ Constrain zenith to +-max_abs_elevation_deg to avoid flip on poles.
        """
        self.zenith = max(-math.radians(max_abs_elevation_deg), min(math.radians(max_abs_elevation_deg), self.zenith))

    def pan(self, dx: float, dy: float):
        """ Pans the camera laterally.
        """
        # Cameras lateral unit vectors in world frame.
        view_mtx: QMatrix4x4 = self.get_view_matrix()
        camera_x: QVector3D = QVector3D(view_mtx.row(0).x(), view_mtx.row(0).y(), view_mtx.row(0).z())
        camera_y: QVector3D = QVector3D(view_mtx.row(1).x(), view_mtx.row(1).y(), view_mtx.row(1).z())

        # Speed proportional to zoom level.
        pan_speed: float = self.distance * 0.001
        target_offset: QVector3D = (-dx * camera_x + dy * camera_y) * pan_speed
        self.target += target_offset

    def rotate(self, delta_azimuth: float, delta_elevation: float) -> None:
        """ Rotate around target point.
        :param delta_azimuth: Radians.
        :param delta_elevation: Radians.
        """
        self.azimuth += delta_azimuth
        self.zenith += delta_elevation
        # Constrain pitch to prevent inversion at poles
        self.constrain_azimuth()
        self.constrain_zenith(max_abs_elevation_deg=89.)

    def zoom(self, delta_distance: float) -> None:
        """ Zoom in/out by changing camera distance.
        :param delta_distance: Meters.
        """
        # Zoom in or out.
        zoom_factor = 0.9 if delta_distance > 0 else 1.1
        # Multiplicative scaling gives natural smooth zooming.
        self.distance = max(0.01, self.distance * zoom_factor)

    def set_pivot(self, new_pivot: QVector3D, keep_eye_position: bool = False):
        """ Change rotation target in world space.
        """
        # TODO: Not tested.
        if keep_eye_position:
            # Recompute distance, azimuth and zenith from current eye to new_pivot
            # current_view = self.get_view_matrix().inverted()[0]
            # current_eye = current_view.column(3).toVector3D()
            # Z-axis of inverted (world to camera transformation) view matrix.
            current_view = self.get_view_matrix()
            current_eye = current_view.row(3).toVector3D()
            offset = current_eye - new_pivot
            self.distance = offset.length()
            # Recompute angles from offset vector if needed
        self.target = new_pivot
