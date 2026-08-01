""" Ray maths

"""

from PyQt6.QtCore import QRect
from PyQt6.QtGui import *
from typing import Tuple


def get_mouse_ray(viewport, pos: QVector2D) -> Tuple[QVector3D, QVector3D]:
    """ Get the mouse ray location and direction in 3d space based on
    :param viewport: OpenGLViewport
    :param pos: Click position coordinates in pixels. Origo left up.
    """

    projection_mat, view_mat = viewport.get_matrices()  # Camera projection and view. QMatrix4x4.
    dpr: float = viewport.devicePixelRatio()  # 1 on normal monitor

    # Pixel coordinates of click event. Origo = left down
    win_x_pixel = int(dpr * pos.x())
    win_y_pixel = int(dpr * (viewport.height() - pos.y()))

    screen = QRect(0, 0, int(dpr * viewport.width()), int(dpr * viewport.height()))

    # Far and near point of ray in world coordinates
    near_pt = QVector3D(win_x_pixel, win_y_pixel, 0.0).unproject(view_mat, projection_mat, screen)
    far_pt = QVector3D(win_x_pixel, win_y_pixel, 1.0).unproject(view_mat, projection_mat, screen)
    return near_pt, (far_pt - near_pt).normalized()

def intersect_ray_plane(ray_origin: QVector3D, ray_dir: QVector3D, plane_point: QVector3D, plane_normal: QVector3D) -> QVector3D | None:

    denom: float = QVector3D.dotProduct(ray_dir, plane_normal)

    # Plane parallel to viewing ray. Intersection point location unstable.
    if abs(denom) < 1e-6:
        return None

    t = QVector3D.dotProduct(plane_point - ray_origin, plane_normal) / denom

    # plane is behind camera
    if t < 0:
        return None

    return ray_origin + t*ray_dir

def hit_arrow(ray_origin: QVector3D, ray_dir: QVector3D, axis_origin: QVector3D, axis_dir: QVector3D, arrow_length: float, hit_radius: float) -> bool:
    """Tests mouse ray against all 3 arrow capsules."""

    # Start and end points of arrow
    p1: QVector3D = axis_origin
    p2: QVector3D = axis_origin + axis_dir * arrow_length

    # Shortest line segment distance math
    u: QVector3D = p2 - p1
    v: QVector3D = ray_dir
    w: QVector3D = p1 - ray_origin

    a: float = QVector3D.dotProduct(u, u)
    b: float = QVector3D.dotProduct(u, v)
    c: float = QVector3D.dotProduct(v, v)
    d: float = QVector3D.dotProduct(u, w)
    e: float = QVector3D.dotProduct(v, w)

    denom = a * c - b * b

    if abs(denom) > 1e-5:
        sc = (b * e - c * d) / denom
        tc = (a * e - b * d) / denom

        if 0.0 <= sc <= 1.0:
            dist = ((p1 + u * sc) - (ray_origin + v * tc)).length()
            if dist < hit_radius:
                return True

    return False

def hit_rotation_ring(ray_origin: QVector3D, ray_dir: QVector3D, ring_origin: QVector3D, ring_normal: QVector3D, radius_out: float, radius_in: float, hit_delta_radius: float) -> bool:

    # Viewing ray intersection point on ring plane
    ray_plane_intersect: QVector3D = intersect_ray_plane(ray_origin, ray_dir, ring_origin, ring_normal)

    if ray_plane_intersect is None:
        return False

    # Distance from ring origin the ray has been hit.
    hit_radius: float = (ray_plane_intersect - ring_origin).length()

    # Outer and inner radius for hit boundary
    inner_hit_radius: float = radius_in - hit_delta_radius
    outer_hit_radius: float = radius_out + hit_delta_radius

    if inner_hit_radius <= hit_radius <= outer_hit_radius:
        return True

    return False

def hit_translation_plane(ray_origin: QVector3D, ray_dir: QVector3D, origin: QVector3D, u: QVector3D, v: QVector3D, n: QVector3D, plane_size: float, plane_offset: float, hit_radius_delta: float) -> bool:

    # origin: QVector3D = transform.column(0).toVector3D()
    #
    # # Plane unit direction vectors.
    # u: QVector3D = transform.column(1).toVector3D()
    # v: QVector3D = transform.column(2).toVector3D()
    #
    # # Plane unit normal vector.
    # normal: QVector3D = transform.column(3).toVector3D()

    # Ray intersection point on plane.
    ray_plane_intersect = intersect_ray_plane(ray_origin, ray_dir, origin, n)
    if ray_plane_intersect is None:
        return False

    # Offset plane.
    plane_origin = origin + (u + v) * plane_offset

    # Plane origin to ray intersect vector.
    d = ray_plane_intersect - plane_origin

    # If plane-area is hit, both of these are between [0, plane_size].
    du: float = QVector3D.dotProduct(d, u)
    dv: float = QVector3D.dotProduct(d, v)

    # Add hit offset in all directions.
    lower_hit_limit: float = - hit_radius_delta
    upper_hit_limit: float = plane_size + hit_radius_delta

    # Check if hit.
    if lower_hit_limit <= du <= upper_hit_limit and lower_hit_limit <= dv <= upper_hit_limit:
        return True

    return False
