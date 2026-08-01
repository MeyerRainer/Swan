""" Interactive gizmo for translating and rotating objects

"""
from robot_math.pose import Pose
import qt_gui.viewport.cg_math as cg_math
from qt_gui.viewport.visuals.scene_object import SceneObject
import qt_gui.viewport.shapes as shapes

from typing import List, Tuple
from enum import Enum
from dataclasses import dataclass
import math
import numpy as np
from OpenGL import GL
from PyQt6.QtGui import QMatrix4x4, QVector3D, QQuaternion


class HandleType(Enum):
    NONE = 0

    TRANSLATE_X = 1
    TRANSLATE_Y = 2
    TRANSLATE_Z = 3

    TRANSLATE_XY = 4
    TRANSLATE_YZ = 5
    TRANSLATE_ZX = 6

    ROTATE_X = 7
    ROTATE_Y = 8
    ROTATE_Z = 9

@dataclass
class DragContext:
    mode: HandleType = HandleType(0)

    origin: QVector3D | None = None
    orientation: QQuaternion | None = None

    plane_point: QVector3D | None = None
    plane_normal: QVector3D | None = None

    start_hit: QVector3D | None = None
    start_vector: QVector3D | None = None
    axis: QVector3D | None = None
    rotation_center: QVector3D | None = None


class Gizmo:
    """Manages rendering, hit testing, and dragging along X, Y, Z axes."""

    def __init__(self):

        self.TRANSL_ARROW_LEN = 1.9     # Total length of arrow
        self.TRANSL_ARROW_RAD = 0.04    # Arrow shaft radius
        self.TRANSL_CONE_LEN = 0.35     # Length of cone
        self.TRANSL_CONE_RAD = 0.1      # Radius of cone

        self.ROT_RING_RAD_OUT = 1.25     # Rotation ring outer radius
        self.ROT_RING_T = 0.2           # Rotation ring delta radius
        self.TRANSL_PLANE_SIZE = 0.5    # Translation plane eight and width.
        self.TRANSL_PLANE_OFFS = 0.1    # Planes offset from origin. Negative for plane behind the arrows.

        self.DC = DragContext()  # Drag plane constraints

        self.target_object = None  # Object to which Gizmo is attached to

        self.vao = None  # Vertex array object
        self.vertex_count = 0

        self.init_gizmo_mesh()

    def attach_to(self, obj: SceneObject):
        self.target_object = obj

    def model_matrix(self) -> QMatrix4x4 | None:
        """ Gizmo stays locked to target object's position. """
        if self.target_object:
            return self.target_object.model_matrix()
        return None

    def init_gizmo_mesh(self, segments=12):
        """ Generates 3 Arrow Meshes (X=Red, Y=Green, Z=Blue). """
        vertices = []
        main_color = 1.0
        other_color = 0.1
        # Axis colors
        col_x_axis = [main_color, other_color, other_color, 1.0]
        col_y_axis = [other_color, main_color, other_color, 1.0]
        col_z_axis = [other_color, other_color, main_color, 1.0]
        # Ring colors
        col_xy_ring = [main_color, other_color, other_color, 0.25]
        col_yz_ring = [other_color, main_color, other_color, 0.25]
        col_xz_ring = [other_color, other_color, main_color, 0.25]
        # Plane colors
        col_xy_plane = [other_color, other_color, main_color, 0.25]
        col_xz_plane = [other_color, main_color, other_color, 0.25]
        col_yz_plane = [main_color, other_color, other_color, 0.25]

        # Translation arrows
        vertices.extend(shapes.build_arrow(axis_dir=QVector3D(1, 0, 0), color=col_x_axis, length=self.TRANSL_ARROW_LEN, length_cone=self.TRANSL_CONE_LEN, radius_cone=self.TRANSL_CONE_RAD, radius_shaft=self.TRANSL_ARROW_RAD, segments=segments))
        vertices.extend(shapes.build_arrow(axis_dir=QVector3D(0, 1, 0), color=col_y_axis, length=self.TRANSL_ARROW_LEN, length_cone=self.TRANSL_CONE_LEN, radius_cone=self.TRANSL_CONE_RAD, radius_shaft=self.TRANSL_ARROW_RAD, segments=segments))
        vertices.extend(shapes.build_arrow(axis_dir=QVector3D(0, 0, 1), color=col_z_axis, length=self.TRANSL_ARROW_LEN, length_cone=self.TRANSL_CONE_LEN, radius_cone=self.TRANSL_CONE_RAD, radius_shaft=self.TRANSL_ARROW_RAD, segments=segments))

        # Rotation rings
        vertices.extend(shapes.build_rotation_ring(axis_dir=QVector3D(1, 0, 0), color=col_xy_ring, radius_in=self.ROT_RING_RAD_OUT-self.ROT_RING_T, radius_out=self.ROT_RING_RAD_OUT))
        vertices.extend(shapes.build_rotation_ring(axis_dir=QVector3D(0, 1, 0), color=col_yz_ring, radius_in=self.ROT_RING_RAD_OUT-self.ROT_RING_T, radius_out=self.ROT_RING_RAD_OUT))
        vertices.extend(shapes.build_rotation_ring(axis_dir=QVector3D(0, 0, 1), color=col_xz_ring, radius_in=self.ROT_RING_RAD_OUT-self.ROT_RING_T, radius_out=self.ROT_RING_RAD_OUT))

        # Translation planes
        vertices.extend(shapes.build_plane(u_dir=QVector3D(1, 0, 0), v_dir=QVector3D(0, 1, 0), color=col_xy_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE))
        vertices.extend(shapes.build_plane(u_dir=QVector3D(0, 1, 0), v_dir=QVector3D(0, 0, 1), color=col_yz_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE))
        vertices.extend(shapes.build_plane(u_dir=QVector3D(1, 0, 0), v_dir=QVector3D(0, 0, 1), color=col_xz_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE))

        data = np.array(vertices, dtype=np.float32)
        self.vertex_count = len(data) // 7

        self.vao = GL.glGenVertexArrays(1)
        vbo = GL.glGenBuffers(1)

        GL.glBindVertexArray(self.vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)

        # Position: Vec3 (Offset 0, Stride 28)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 28, GL.GLvoidp(0))
        GL.glEnableVertexAttribArray(0)
        # Color: Vec4 (Offset 12, Stride 28)
        GL.glVertexAttribPointer(1, 4, GL.GL_FLOAT, GL.GL_FALSE, 28, GL.GLvoidp(12))
        GL.glEnableVertexAttribArray(1)

    def draw(self, model_loc: int, body_frame=True):
        if not self.target_object:
            return

        # Depth isolation
        GL.glDisable(GL.GL_DEPTH_TEST)  # Render on top

        # body_frame = True
        if body_frame:
            transform: QMatrix4x4 = self.model_matrix()
        else:
            transform: QMatrix4x4 = QMatrix4x4()
            transform.translate(*self.target_object.pose.position)


        GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, transform.data())
        GL.glBindVertexArray(self.vao)

        # TODO: What is this
        # 36 vertices per arrow segment set
        verts_per_arrow = self.vertex_count // 3

        # Enable alpha blending
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        # Draw gizmo
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, self.vertex_count)

        # Disable alpha blending
        GL.glDisable(GL.GL_BLEND)

        GL.glEnable(GL.GL_DEPTH_TEST)  # ALWAYS Restore Depth Test State!

    def hit_translation_axis(self, ray_origin: QVector3D, ray_dir: QVector3D, body_frame: bool = True) -> HandleType:
        """ Checks if translation arrow has been hit. If so, which one.
        :param ray_origin: Mouse ray origin (Camera or near point)
        :param ray_dir: Direction of mouse ray. Unit vector.
        :param body_frame: body frame / world frame
        """
        if not self.target_object:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        axis_origin: QVector3D = QVector3D(*pose.position)
        rot_mat = pose.rot_mat if body_frame else np.eye(3)
        x_axis = QVector3D(*rot_mat[:, 0])
        y_axis = QVector3D(*rot_mat[:, 1])
        z_axis = QVector3D(*rot_mat[:, 2])

        axis: List[Tuple[HandleType, QVector3D]] = [
            (HandleType.TRANSLATE_X, x_axis),
            (HandleType.TRANSLATE_Y, y_axis),
            (HandleType.TRANSLATE_Z, z_axis),
        ]

        for handle, axis_dir in axis:
            hit: bool = cg_math.hit_arrow(ray_origin, ray_dir, axis_origin, axis_dir, self.TRANSL_ARROW_LEN, 1.5*self.TRANSL_ARROW_RAD)
            if hit:
                return handle

        return HandleType.NONE

    def hit_translation_planes(self, ray_origin: QVector3D, ray_dir: QVector3D, body_frame=True):
        if self.target_object is None:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        rot_mat = pose.rot_mat if body_frame else np.eye(3)
        x_axis: QVector3D = QVector3D(*rot_mat[:, 0])
        y_axis: QVector3D = QVector3D(*rot_mat[:, 1])
        z_axis: QVector3D = QVector3D(*rot_mat[:, 2])
        origin: QVector3D = QVector3D(*pose.position)

        planes = [
            (HandleType.TRANSLATE_XY, x_axis, y_axis, z_axis),
            (HandleType.TRANSLATE_YZ, y_axis, z_axis, x_axis),
            (HandleType.TRANSLATE_ZX, z_axis, x_axis, y_axis)
        ]

        for handle, u, v, normal in planes:
            hit: bool = cg_math.hit_translation_plane(ray_origin, ray_dir, origin, u, v, normal, self.TRANSL_PLANE_SIZE, self.TRANSL_PLANE_OFFS, 0.1*self.TRANSL_PLANE_SIZE)
            if hit:
                return handle

        return HandleType.NONE

    def hit_rotation_rings(self, ray_origin: QVector3D, ray_dir: QVector3D, body_frame=True):
        if self.target_object is None:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        origin: QVector3D = QVector3D(*pose.position)
        rot_mat = pose.rot_mat if body_frame else np.eye(3)
        x_axis: QVector3D = QVector3D(*rot_mat[:, 0])
        y_axis: QVector3D = QVector3D(*rot_mat[:, 1])
        z_axis: QVector3D = QVector3D(*rot_mat[:, 2])

        radius_out: float = self.ROT_RING_RAD_OUT
        radius_in: float = self.ROT_RING_RAD_OUT - self.ROT_RING_T
        delta_radius = 0.1 * self.ROT_RING_T

        rings = [
            (HandleType.ROTATE_X, x_axis),
            (HandleType.ROTATE_Y, y_axis),
            (HandleType.ROTATE_Z, z_axis)
        ]

        for handle, normal in rings:
            hit = cg_math.hit_rotation_ring(ray_origin, ray_dir, origin, normal, radius_out, radius_in, delta_radius)
            if hit:
                return handle

        return HandleType.NONE

    def get_axis_vector(self, body_frame=True) -> QVector3D | None:
        """ Return axis direction vector basen on current drag context """
        mode = self.DC.mode

        match mode:
            case HandleType.NONE:
                return None

            case HandleType.TRANSLATE_X | HandleType.TRANSLATE_YZ | HandleType.ROTATE_X:
                return QVector3D(*self.target_object.pose.rot_mat[:, 0]) if body_frame else QVector3D(1., 0., 0.) # X-Axis

            case HandleType.TRANSLATE_Y | HandleType.TRANSLATE_ZX | HandleType.ROTATE_Y:
                return QVector3D(*self.target_object.pose.rot_mat[:, 1]) if body_frame else QVector3D(0., 1., 0.) # Y-Axis

            case HandleType.TRANSLATE_Z | HandleType.TRANSLATE_XY | HandleType.ROTATE_Z:
                return QVector3D(*self.target_object.pose.rot_mat[:, 2]) if body_frame else QVector3D(0., 0., 1.) # Z-Axis

            case _:
                # Fallback handler for unhandled or invalid cases
                raise ValueError(f"Unhandled handle type: {mode}")

    def build_drag_plane(self, camera_position: QVector3D, body_frame=True) -> bool:
        """ Creates the plain on which dragging happens """

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
        """ Start dragging event. Fill drag context. """

        self.DC.mode = mode
        self.DC.origin = QVector3D(self.target_object.get_position())

        if not self.build_drag_plane(camera_position, body_frame=body_frame):
            self.DC.mode = HandleType.NONE
            return

        # Update drag context
        pose: Pose = self.target_object.pose
        self.DC.orientation = QQuaternion(*pose.quaternion.components)


        self.DC.start_hit = cg_math.intersect_ray_plane(ray_origin, ray_dir, self.DC.plane_point, self.DC.plane_normal)
        self.DC.start_vector = (self.DC.start_hit - self.DC.origin).normalized()

        if self.DC.start_hit is None:
            self.DC.mode = HandleType.NONE

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
            self.target_object.set_position(self.DC.origin + axis * projected_displacement)

        # Plane translation
        elif self.DC.mode in [HandleType.TRANSLATE_XY, HandleType.TRANSLATE_YZ, HandleType.TRANSLATE_ZX]:
            self.target_object.set_position(self.DC.origin + displacement)

        # Rotation
        elif self.DC.mode in [HandleType.ROTATE_X, HandleType.ROTATE_Y, HandleType.ROTATE_Z]:
            current_vector: QVector3D = hit - self.DC.origin
            cross: QVector3D = QVector3D.crossProduct(self.DC.start_vector, current_vector)
            sin_angle: float = QVector3D.dotProduct(cross, axis)
            cos_angle: float = QVector3D.dotProduct(self.DC.start_vector, current_vector)
            angle: float = math.atan2(sin_angle, cos_angle)
            # TODO: Clean this mess
            axis = self.get_axis_vector(body_frame=body_frame)
            if self.DC.mode == HandleType.ROTATE_X:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(1., 0., 0.), math.degrees(angle))
            elif self.DC.mode == HandleType.ROTATE_Y:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 1., 0.), math.degrees(angle))
            else:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 0., 1.), math.degrees(angle))

            current_q: QQuaternion = self.DC.orientation
            if body_frame:
                self.target_object.set_quaternion(current_q * rotation_q)
            else:
                self.target_object.set_quaternion(rotation_q * current_q)
