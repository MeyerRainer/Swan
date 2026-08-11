""" Interactive gizmo for translating and rotating objects

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtGui import QVector3D
from enum import Enum, auto

from robot_math.pose import Pose
from qt_gui.viewport.visuals.visual import Renderable
from qt_gui.viewport.visuals.frame import ArrowFrame, RingFrame, PlaneFrame


class HandleType(Enum):

    NONE = auto()

    TRANSLATE_X = auto()
    TRANSLATE_Y = auto()
    TRANSLATE_Z = auto()

    TRANSLATE_XY = auto()
    TRANSLATE_YZ = auto()
    TRANSLATE_ZX = auto()

    ROTATE_X = auto()
    ROTATE_Y = auto()
    ROTATE_Z = auto()


class Gizmo(Renderable):

    def __init__(self, target_object):

        self.arrow_frame = ArrowFrame()
        self.ring_frame = RingFrame()
        self.plane_frame = PlaneFrame()

        self.target_object = target_object  # Object to which Gizmo is attached to

    def render(self, context, pose: Pose):
        for frame in [self.arrow_frame, self.ring_frame, self.plane_frame]:
            frame.render(context, pose)

    def ray_hit(self, ray_origin: QVector3D, ray_dir: QVector3D):
        trans_axis_handle: HandleType = self.hit_translation_axis(ray_origin, ray_dir)
        rot_plane_handle: HandleType = self.hit_rotation_rings(ray_origin, ray_dir)
        trans_plane_handle: HandleType = self.hit_translation_planes(ray_origin, ray_dir)

        # TODO: Add priority.
        if trans_axis_handle != HandleType.NONE:
            return trans_axis_handle
        if rot_plane_handle != HandleType.NONE:
            return rot_plane_handle
        if trans_plane_handle != HandleType.NONE:
            return trans_plane_handle
        return HandleType.NONE

    def hit_translation_axis(self, ray_origin: QVector3D, ray_dir: QVector3D) -> HandleType:
        """ Checks if translation arrow has been hit. If so, which one.
        :param ray_origin: Mouse ray origin (Camera or near point)
        :param ray_dir: Direction of mouse ray. Unit vector.
        """

        pose: Pose = self.target_object.pose

        if self.arrow_frame.x.hit(ray_origin, ray_dir, pose):
            return HandleType.TRANSLATE_X
        if self.arrow_frame.y.hit(ray_origin, ray_dir, pose):
            return HandleType.TRANSLATE_Y
        if self.arrow_frame.z.hit(ray_origin, ray_dir, pose):
            return HandleType.TRANSLATE_Z
        return HandleType.NONE

    def hit_rotation_rings(self, ray_origin: QVector3D, ray_dir: QVector3D, body_frame=True):

        pose: Pose = self.target_object.pose

        if self.ring_frame.yz.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.ROTATE_X
        if self.ring_frame.zx.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.ROTATE_Y
        if self.ring_frame.xy.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.ROTATE_Z
        return HandleType.NONE

    def hit_translation_planes(self, ray_origin: QVector3D, ray_dir: QVector3D, body_frame=True):

        pose: Pose = self.target_object.pose

        if self.plane_frame.yz.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.TRANSLATE_YZ
        if self.plane_frame.zx.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.TRANSLATE_ZX
        if self.plane_frame.xy.hit(ray_origin, ray_dir, pose, body_frame):
            return HandleType.TRANSLATE_XY
        return HandleType.NONE
