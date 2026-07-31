""" Interactive gizmo for translating and rotating objects

"""
from robot_math.pose import Pose
from qt_gui.viewport.visuals.scene_object import SceneObject

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

        # Translation arrow mesh creation along axis_dir
        def build_arrow(axis_dir, color):
            length_shaft, radius_shaft = self.TRANSL_ARROW_LEN - self.TRANSL_CONE_LEN, self.TRANSL_ARROW_RAD
            length_cone, radius_cone = self.TRANSL_CONE_LEN, self.TRANSL_CONE_RAD

            # Create orthogonal vectors for circle generation
            if abs(axis_dir.x()) > 0.9:
                u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
            elif abs(axis_dir.y()) > 0.9:
                u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
            else:
                u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

            # Arrow shaft
            for i in range(segments):
                a1 = (i / segments) * 2 * math.pi
                a2 = ((i + 1) / segments) * 2 * math.pi

                r1 = u_dir * math.cos(a1) * radius_shaft + v_dir * math.sin(a1) * radius_shaft
                r2 = u_dir * math.cos(a2) * radius_shaft + v_dir * math.sin(a2) * radius_shaft

                p1, p2 = r1, axis_dir * length_shaft + r1
                p3, p4 = axis_dir * length_shaft + r2, r2

                for p in [p1, p2, p3, p1, p3, p4]:
                    vertices.extend([p.x(), p.y(), p.z()] + color)

            # Arrow tip
            tip_base = axis_dir * length_shaft
            tip_apex = axis_dir * (length_shaft + length_cone)
            for i in range(segments):
                a1 = (i / segments) * 2 * math.pi
                a2 = ((i + 1) / segments) * 2 * math.pi

                r1 = u_dir * math.cos(a1) * radius_cone + v_dir * math.sin(a1) * radius_cone
                r2 = u_dir * math.cos(a2) * radius_cone + v_dir * math.sin(a2) * radius_cone

                for p in [tip_base + r1, tip_apex, tip_base + r2]:
                    vertices.extend([p.x(), p.y(), p.z()] + color)

        # Rotation ring mesh creation normal to axis_dir
        def build_rotation_ring(axis_dir: QVector3D, color: List[float], r_inner: float, r_outer: float, ring_segments: int = 36):
            # Determine orthogonal plane basis vectors
            if abs(axis_dir.x()) > 0.9:
                u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
            elif abs(axis_dir.y()) > 0.9:
                u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
            else:
                u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

            for i in range(ring_segments):
                a1 = (i / ring_segments) * 2 * math.pi
                a2 = ((i + 1) / ring_segments) * 2 * math.pi

                cos1, sin1 = math.cos(a1), math.sin(a1)
                cos2, sin2 = math.cos(a2), math.sin(a2)

                # Inner and outer points for current and next slice
                p1_in = u_dir * cos1 * r_inner + v_dir * sin1 * r_inner
                p1_out = u_dir * cos1 * r_outer + v_dir * sin1 * r_outer
                p2_in = u_dir * cos2 * r_inner + v_dir * sin2 * r_inner
                p2_out = u_dir * cos2 * r_outer + v_dir * sin2 * r_outer

                # Two triangles forming a quad ring segment
                # Render both sides (clockwise & counter-clockwise) so ring is visible from any angle
                quad_vertices = [p1_in, p1_out, p2_out, p1_in, p2_out, p2_in,       # Direct side
                                 p1_in, p2_out, p1_out, p1_in, p2_in, p2_out]       # Reverse side

                for p in quad_vertices:
                    vertices.extend([p.x(), p.y(), p.z()] + color)

        # Translation plane mesh creation normal to axis_dir
        def build_plane(u_dir: QVector3D, v_dir: QVector3D, color: List[float], offset: float, size: float):
            p0 = u_dir * offset + v_dir * offset
            p1 = p0 + u_dir * size
            p2 = p0 + u_dir * size + v_dir * size
            p3 = p0 + v_dir * size

            # Double-sided quad
            quad = [p0, p1, p2, p0, p2, p3,
                    p0, p2, p1, p0, p3, p2]
            for p in quad:
                vertices.extend([p.x(), p.y(), p.z()] + color)

        # Translation arrows
        build_arrow(QVector3D(1, 0, 0), col_x_axis)
        build_arrow(QVector3D(0, 1, 0), col_y_axis)
        build_arrow(QVector3D(0, 0, 1), col_z_axis)

        # Pitch (X-axis ring), Yaw (Y-axis ring), Roll (Z-axis ring)
        build_rotation_ring(QVector3D(1, 0, 0), col_xy_ring, r_inner=self.ROT_RING_RAD_OUT-self.ROT_RING_T, r_outer=self.ROT_RING_RAD_OUT)
        build_rotation_ring(QVector3D(0, 1, 0), col_yz_ring, r_inner=self.ROT_RING_RAD_OUT-self.ROT_RING_T, r_outer=self.ROT_RING_RAD_OUT)
        build_rotation_ring(QVector3D(0, 0, 1), col_xz_ring, r_inner=self.ROT_RING_RAD_OUT-self.ROT_RING_T, r_outer=self.ROT_RING_RAD_OUT)

        # Translation planes
        build_plane(QVector3D(1, 0, 0), QVector3D(0, 1, 0), col_xy_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE)
        build_plane(QVector3D(0, 1, 0), QVector3D(0, 0, 1), col_yz_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE)
        build_plane(QVector3D(1, 0, 0), QVector3D(0, 0, 1), col_xz_plane, offset=self.TRANSL_PLANE_OFFS, size=self.TRANSL_PLANE_SIZE)

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

    def draw(self, model_loc):
        if not self.target_object:
            return

        # Depth isolation
        GL.glDisable(GL.GL_DEPTH_TEST)  # Render on top

        GL.glUniformMatrix4fv(model_loc, 1, GL.GL_FALSE, self.model_matrix().data())
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

    @staticmethod
    def get_world_axis(self, handle_type: HandleType) -> QVector3D:
        """Extracts local axis vectors transformed into World Space using the Model Matrix."""
        m = self.model_matrix()
        if not m:
            return QVector3D(0, 0, 0)

        # Column 0 = World X, Column 1 = World Y, Column 2 = World Z
        if handle_type in (HandleType.TRANSLATE_X, HandleType.TRANSLATE_YZ, HandleType.ROTATE_X):
            return QVector3D(m.column(0).x(), m.column(0).y(), m.column(0).z()).normalized()
        elif handle_type in (HandleType.TRANSLATE_Y, HandleType.TRANSLATE_ZX, HandleType.ROTATE_Y):
            return QVector3D(m.column(1).x(), m.column(1).y(), m.column(1).z()).normalized()
        elif handle_type in (HandleType.TRANSLATE_Z, HandleType.TRANSLATE_XY, HandleType.ROTATE_Z):
            return QVector3D(m.column(2).x(), m.column(2).y(), m.column(2).z()).normalized()
        return QVector3D(0, 0, 0)

    def hit_translation_axis(self, ray_origin: QVector3D, ray_dir: QVector3D) -> HandleType:
        """Tests mouse ray against all 3 arrow capsules."""
        if not self.target_object:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        origin: QVector3D = QVector3D(*pose.position)

        x_axis = QVector3D(*pose.rot_mat[:, 0])
        y_axis = QVector3D(*pose.rot_mat[:, 1])
        z_axis = QVector3D(*pose.rot_mat[:, 2])

        axis: List[Tuple[HandleType, QVector3D]] = [
            (HandleType.TRANSLATE_X, x_axis),
            (HandleType.TRANSLATE_Y, y_axis),
            (HandleType.TRANSLATE_Z, z_axis),
        ]

        for handle, axis_dir in axis:
            p1 = origin
            p2 = origin + axis_dir * self.TRANSL_ARROW_LEN

            # Shortest line segment distance math
            u = p2 - p1
            v = ray_dir
            w = p1 - ray_origin

            a = QVector3D.dotProduct(u, u)
            b = QVector3D.dotProduct(u, v)
            c = QVector3D.dotProduct(v, v)
            d = QVector3D.dotProduct(u, w)
            e = QVector3D.dotProduct(v, w)

            denom = a * c - b * b
            if abs(denom) > 1e-5:
                sc = (b * e - c * d) / denom
                tc = (a * e - b * d) / denom

                if 0.0 <= sc <= 1.0:
                    dist = ((p1 + u * sc) - (ray_origin + v * tc)).length()
                    if dist < 1.5 * self.TRANSL_ARROW_RAD:  # Hit boundary radius
                        return handle
        return HandleType.NONE

    def hit_translation_planes(self, ray_origin: QVector3D, ray_dir: QVector3D):
        if self.target_object is None:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        x_axis: QVector3D = QVector3D(*pose.rot_mat[:, 0])
        y_axis: QVector3D = QVector3D(*pose.rot_mat[:, 1])
        z_axis: QVector3D = QVector3D(*pose.rot_mat[:, 2])
        origin: QVector3D = QVector3D(*pose.position)

        planes = [
            (HandleType.TRANSLATE_XY, x_axis, y_axis, z_axis),
            (HandleType.TRANSLATE_YZ, y_axis, z_axis, x_axis),
            (HandleType.TRANSLATE_ZX, z_axis, x_axis, y_axis)
        ]

        plane_size: float = self.TRANSL_PLANE_SIZE
        offset: float = self.TRANSL_PLANE_OFFS

        for handle, u, v, normal in planes:
            plane_origin = origin + (u + v) * offset
            hit = self.intersect_ray_plane(ray_origin, ray_dir, plane_origin, normal)

            if hit is None:
                continue

            d = hit - plane_origin

            du = QVector3D.dotProduct(d, u)
            dv = QVector3D.dotProduct(d, v)

            if 0.0 <= du <= plane_size and 0.0 <= dv <= plane_size:
                return handle

        return HandleType.NONE

    def hit_rotation_rings(self, ray_origin: QVector3D, ray_dir: QVector3D):
        if self.target_object is None:
            return HandleType.NONE

        pose: Pose = self.target_object.pose
        centre: QVector3D = QVector3D(*pose.position)

        x_axis: QVector3D = QVector3D(*pose.rot_mat[:, 0])
        y_axis: QVector3D = QVector3D(*pose.rot_mat[:, 1])
        z_axis: QVector3D = QVector3D(*pose.rot_mat[:, 2])

        ring_radius: float = self.ROT_RING_RAD_OUT
        thickness: float = self.ROT_RING_T

        rings = [
            (HandleType.ROTATE_X, x_axis),
            (HandleType.ROTATE_Y, y_axis),
            (HandleType.ROTATE_Z, z_axis)
        ]

        for handle, normal in rings:
            hit = self.intersect_ray_plane(ray_origin, ray_dir, centre, normal)

            if hit is None:
                continue

            radius = (hit - centre).length()

            if abs(radius - ring_radius) <= thickness:
                return handle

        return HandleType.NONE

    def get_axis_vector(self) -> QVector3D | None:
        """ Return axis direction vector basen on current drag context """
        mode = self.DC.mode

        match mode:
            case HandleType.NONE:
                return None

            case HandleType.TRANSLATE_X | HandleType.TRANSLATE_YZ | HandleType.ROTATE_X:
                return QVector3D(*self.target_object.pose.rot_mat[:, 0])  # X-Axis

            case HandleType.TRANSLATE_Y | HandleType.TRANSLATE_ZX | HandleType.ROTATE_Y:
                return QVector3D(*self.target_object.pose.rot_mat[:, 1])  # Y-Axis

            case HandleType.TRANSLATE_Z | HandleType.TRANSLATE_XY | HandleType.ROTATE_Z:
                return QVector3D(*self.target_object.pose.rot_mat[:, 2])  # Z-Axis

            case _:
                # Fallback handler for unhandled or invalid cases
                raise ValueError(f"Unhandled handle type: {mode}")

    def build_drag_plane(self, camera_position: QVector3D) -> bool:
        """ Creates the plain on which dragging happens """

        # Motion axis: Parallel for axis translation, normal for plane translation and rotation
        axis_dir = self.get_axis_vector()
        if axis_dir is None:
            return False

        view_dir = (self.DC.origin - camera_position).normalized()

        if self.DC.mode in [HandleType.TRANSLATE_X, HandleType.TRANSLATE_Y, HandleType.TRANSLATE_Z]:
            plane_normal: QVector3D = (view_dir - axis_dir * QVector3D.dotProduct(view_dir, axis_dir))
        else:
            plane_normal: QVector3D = self.get_axis_vector()

        if plane_normal.lengthSquared() < 1e-6:
            return False

        plane_normal.normalize()

        # Drag plain constraints
        self.DC.plane_point = QVector3D(self.DC.origin)
        self.DC.plane_normal = plane_normal

        return True

    @staticmethod
    def intersect_ray_plane(ray_origin: QVector3D, ray_dir: QVector3D, plane_point: QVector3D, plane_normal: QVector3D) -> QVector3D | None:

        denom: float = QVector3D.dotProduct(ray_dir, plane_normal)

        if abs(denom) < 1e-6:
            return None

        t = QVector3D.dotProduct(plane_point - ray_origin, plane_normal) / denom

        if t < 0:
            return None  # plane is behind camera

        return ray_origin + ray_dir*t

    def start_drag(self, ray_origin: QVector3D, ray_dir: QVector3D, camera_position: QVector3D, mode: HandleType):
        """ Start dragging event. Fill drag context. """

        self.DC.mode = mode
        self.DC.origin = QVector3D(self.target_object.get_position())

        if not self.build_drag_plane(camera_position):
            self.DC.mode = HandleType.NONE
            return

        # Update drag context
        pose: Pose = self.target_object.pose
        self.DC.orientation = QQuaternion(*pose.quaternion.components)


        self.DC.start_hit = self.intersect_ray_plane(ray_origin, ray_dir, self.DC.plane_point, self.DC.plane_normal)
        self.DC.start_vector = (self.DC.start_hit - self.DC.origin).normalized()

        if self.DC.start_hit is None:
            self.DC.mode = HandleType.NONE

    def update_drag(self, ray_origin, ray_dir):

        if self.DC.mode == HandleType.NONE:
            return

        hit = self.intersect_ray_plane(ray_origin, ray_dir, self.DC.plane_point, self.DC.plane_normal)
        if hit is None:
            return

        displacement: QVector3D = hit - self.DC.start_hit
        axis: QVector3D = self.get_axis_vector()

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
            if self.DC.mode == HandleType.ROTATE_X:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(1., 0., 0.), math.degrees(angle))
            elif self.DC.mode == HandleType.ROTATE_Y:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 1., 0.), math.degrees(angle))
            else:
                rotation_q: QQuaternion = QQuaternion.fromAxisAndAngle(QVector3D(0., 0., 1.), math.degrees(angle))

            current_q: QQuaternion = self.DC.orientation
            self.target_object.set_quaternion(current_q * rotation_q)  # Rotate in object frame
