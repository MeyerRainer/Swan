""" Controller for gizmo-objects.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtGui import QVector3D, QQuaternion
from dataclasses import dataclass
from typing import Optional
import math

from qt_gui.viewport.gizmo.gizmo import Gizmo, HandleType
from qt_gui.viewport import cg_math
from robot_math.pose import Pose


@dataclass
class DragContext:
    """ Drag context contains all information of dragging event.
    """

    mode: HandleType = HandleType.NONE

    origin: QVector3D | None = None
    orientation: QQuaternion | None = None

    plane_point: QVector3D | None = None
    plane_normal: QVector3D | None = None

    start_hit: QVector3D | None = None
    start_vector: QVector3D | None = None
    axis: QVector3D | None = None
    rotation_center: QVector3D | None = None


class GizmoController(QObject):

    sgn_mouse_move = pyqtSignal()

    def __init__(self):

        super().__init__()

        self.active_gizmo: Optional[Gizmo] = None
        self.DC = DragContext()
        self.scene_tree = None

    @property
    def is_active(self):
        return self.active_gizmo is not None

    def set_scene_graph_root(self, root):
        self.scene_tree = root

    def handle_mouse_press(self, ray_origin: QVector3D, ray_dir: QVector3D, cam_pos: QVector3D) -> None:
        ray_hit = self.scene_tree.ray_hit(ray_origin, ray_dir)
        if ray_hit is not None:
            handle, gizmo = ray_hit
            if handle is not HandleType.NONE:
                self.active_gizmo = gizmo
                self.start_drag(ray_origin, ray_dir, cam_pos, handle)

    def handle_mouse_move(self, ray_origin: QVector3D, ray_dir: QVector3D):
        if self.active_gizmo is not None:
            self.update_drag(ray_origin, ray_dir)
            self.sgn_mouse_move.emit()

    def handle_mouse_release(self) -> bool:
        if self.active_gizmo is not None:
            # self.on_drag_end(ray_origin, ray_dir)
            self.active_gizmo = None
            return True
        return False

    def get_axis_vector(self, body_frame=True) -> QVector3D | None:
        """ Return axis direction vector basen on current drag current_context.
        """
        mode = self.DC.mode

        match mode:
            case HandleType.NONE:
                return None

            case HandleType.TRANSLATE_X | HandleType.TRANSLATE_YZ | HandleType.ROTATE_X:
                return QVector3D(*self.active_gizmo.target_object.pose.rot_mat[:, 0]) if body_frame else QVector3D(1., 0., 0.) # X-Axis

            case HandleType.TRANSLATE_Y | HandleType.TRANSLATE_ZX | HandleType.ROTATE_Y:
                return QVector3D(*self.active_gizmo.target_object.pose.rot_mat[:, 1]) if body_frame else QVector3D(0., 1., 0.) # Y-Axis

            case HandleType.TRANSLATE_Z | HandleType.TRANSLATE_XY | HandleType.ROTATE_Z:
                return QVector3D(*self.active_gizmo.target_object.pose.rot_mat[:, 2]) if body_frame else QVector3D(0., 0., 1.) # Z-Axis

            case _:
                # Fallback handler for unhandled or invalid cases
                raise ValueError(f"Unhandled handle type: {mode}")

    def build_drag_plane(self, camera_position: QVector3D, body_frame=True) -> bool:
        """ Creates the plain on which dragging happens.
        """

        # Motion axis: Parallel for axis translation, normal for plane translation and rotation
        axis_dir = self.get_axis_vector(body_frame=body_frame)
        if axis_dir is None:
            return False

        view_dir = (self.DC.origin - camera_position).normalized()

        if self.DC.mode in [HandleType.TRANSLATE_X, HandleType.TRANSLATE_Y, HandleType.TRANSLATE_Z]:
            plane_normal: QVector3D = (view_dir - axis_dir * QVector3D.dotProduct(view_dir, axis_dir))
        else:
            plane_normal: QVector3D = axis_dir

        if plane_normal.lengthSquared() < 1e-6:
            return False

        plane_normal.normalize()

        # Drag plain constraints
        self.DC.plane_point = QVector3D(self.DC.origin)
        self.DC.plane_normal = plane_normal

        return True

    def start_drag(self, ray_origin: QVector3D, ray_dir: QVector3D, camera_position: QVector3D, mode: HandleType, body_frame=True):
        """ Start dragging event. Fill drag current_context.
        """

        self.DC.mode = mode
        self.DC.origin = self.active_gizmo.target_object.position

        # Drag plane. For translation arrow, parallel to arrow and as perpendicular to camera as possible.
        # For translation plane and rotation ring, normal to axis.
        if not self.build_drag_plane(camera_position, body_frame=body_frame):
            self.DC.mode = HandleType.NONE
            return

        # Update drag current_context.
        pose: Pose = self.active_gizmo.target_object.pose
        self.DC.orientation = QQuaternion(*pose.quaternion.components)

        # Hit point on drag plane.
        self.DC.start_hit = cg_math.intersect_ray_plane(ray_origin, ray_dir, self.DC.plane_point, self.DC.plane_normal)
        if self.DC.start_hit is None:
            self.DC.mode = HandleType.NONE

        # Reference vector from object center to hit point for rotations.
        self.DC.start_vector = (self.DC.start_hit - self.DC.origin).normalized()



    def update_drag(self, ray_origin, ray_dir, body_frame=True):

        if self.DC.mode == HandleType.NONE:
            return

        hit = cg_math.intersect_ray_plane(ray_origin, ray_dir, self.DC.plane_point, self.DC.plane_normal)
        if hit is None:
            return

        displacement: QVector3D = hit - self.DC.start_hit
        axis: QVector3D = self.get_axis_vector(body_frame=body_frame)

        # Axis translation
        if self.DC.mode in [HandleType.TRANSLATE_X, HandleType.TRANSLATE_Y, HandleType.TRANSLATE_Z]:
            # direction = self.get_axis_vector()
            projected_displacement = QVector3D.dotProduct(displacement, axis)
            self.active_gizmo.target_object.position = self.DC.origin + axis * projected_displacement

        # Plane translation
        elif self.DC.mode in [HandleType.TRANSLATE_XY, HandleType.TRANSLATE_YZ, HandleType.TRANSLATE_ZX]:
            self.active_gizmo.target_object.position = self.DC.origin + displacement

        # Rotation
        elif self.DC.mode in [HandleType.ROTATE_X, HandleType.ROTATE_Y, HandleType.ROTATE_Z]:
            current_vector: QVector3D = hit - self.DC.origin
            # Rotation from start orientation to end orientation.
            rotation_vector: QVector3D = QVector3D.crossProduct(self.DC.start_vector, current_vector)
            # Compute the angle of rotation.
            sin_angle: float = QVector3D.dotProduct(rotation_vector, axis)
            cos_angle: float = QVector3D.dotProduct(self.DC.start_vector, current_vector)
            angle: float = math.atan2(sin_angle, cos_angle)
            # axis = self.get_axis_vector(body_frame=body_frame)
            if self.DC.mode == HandleType.ROTATE_X:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(1., 0., 0.), math.degrees(angle))
            elif self.DC.mode == HandleType.ROTATE_Y:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 1., 0.), math.degrees(angle))
            else:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 0., 1.), math.degrees(angle))

            if body_frame:
                self.active_gizmo.target_object.quaternion = self.DC.orientation * rotation_q  # Body frame rotation
            else:
                self.active_gizmo.target_object.quaternion = rotation_q * self.DC.orientation  # World frame rotation
