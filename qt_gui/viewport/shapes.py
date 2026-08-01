import math
from typing import List

from PyQt6.QtGui import QVector3D, QVector4D

# Translation arrow mesh creation along axis_dir
def build_arrow(axis_dir: QVector3D, color: list[float], length: float = 1.9, length_cone: float = 0.4,
                radius_cone: float = 0.2, radius_shaft: float = 0.1, segments: int = 12):

    length_shaft: float = length - length_cone

    vertices = []

    # Orthogonal vectors relative to axis direction
    if abs(axis_dir.x()) > 0.9:
        u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
    elif abs(axis_dir.y()) > 0.9:
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
    else:
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

    # Arrow shaft
    for i in range(segments):
        # Increment in range [0, 1]
        a1: float = (i / segments) * 2 * math.pi
        a2: float = ((i + 1) / segments) * 2 * math.pi

        # Radial vectors for shaft vertices
        r1: QVector3D = radius_shaft * (math.cos(a1)*u_dir + math.sin(a1)*v_dir)
        r2: QVector3D = radius_shaft * (math.cos(a2)*u_dir + math.sin(a2)*v_dir)

        # Circle vertices at the beginning and end of the shaft
        p1: QVector3D = r1
        p2: QVector3D = r1 + axis_dir * length_shaft
        p3: QVector3D = r2 + axis_dir * length_shaft
        p4: QVector3D = r2

        for p in [p1, p2, p3, p1, p3, p4]:
            vertices.extend([p.x(), p.y(), p.z()] + color)

    # Arrow tip
    tip_base: QVector3D = length_shaft * axis_dir
    tip_apex: QVector3D = (length_shaft + length_cone) * axis_dir

    for i in range(segments):
        a1: float = 2 * math.pi * (i / segments)
        a2: float = 2 * math.pi * ((i + 1) / segments)

        r1: QVector3D = radius_cone * (math.cos(a1)*u_dir + math.sin(a1)*v_dir)
        r2: QVector3D = radius_cone * (math.cos(a2)*u_dir + math.sin(a2)*v_dir)

        for p in [tip_base + r1, tip_apex, tip_base + r2]:
            vertices.extend([p.x(), p.y(), p.z()] + color)

    return vertices

# Rotation ring mesh creation normal to axis_dir
def build_rotation_ring(axis_dir: QVector3D, color: List[float], radius_in: float, radius_out: float, segments: int = 36):

    vertices = []

    # Determine orthogonal plane basis vectors
    if abs(axis_dir.x()) > 0.9:
        u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
    elif abs(axis_dir.y()) > 0.9:
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
    else:
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

    for i in range(segments):
        # Increment in range [0, 1]
        a1: float = 2 * math.pi * (i / segments)
        a2: float = 2 * math.pi * ((i + 1) / segments)

        c1, s1 = math.cos(a1), math.sin(a1)
        c2, s2 = math.cos(a2), math.sin(a2)

        # Vertices for inner and outer rings on both sides
        p1_in: QVector3D = radius_in*(c1*u_dir + s1*v_dir)
        p1_out: QVector3D = radius_out*(c1*u_dir + s1*v_dir)
        p2_in: QVector3D = radius_in*(c2*u_dir + s2*v_dir)
        p2_out: QVector3D = radius_out*(c2*u_dir + s2*v_dir)

        # Two triangles forming a quad ring segment
        # Render both sides (clockwise & counter-clockwise) so ring is visible from any angle
        quad_vertices = [p1_in, p1_out, p2_out, p1_in, p2_out, p2_in,       # Direct side
                         p1_in, p2_out, p1_out, p1_in, p2_in, p2_out]       # Reverse side

        for p in quad_vertices:
            vertices.extend([p.x(), p.y(), p.z()] + color)

    return vertices

# Translation plane mesh creation normal to axis_dir
def build_plane(u_dir: QVector3D, v_dir: QVector3D, color: List[float], offset: float, size: float):

    vertices = []

    p0: QVector3D = u_dir * offset + v_dir * offset
    p1: QVector3D = p0 + u_dir * size
    p2: QVector3D = p0 + u_dir * size + v_dir * size
    p3: QVector3D = p0 + v_dir * size

    # Double-sided quad
    quad = [p0, p1, p2, p0, p2, p3,
            p0, p2, p1, p0, p3, p2]
    for p in quad:
        vertices.extend([p.x(), p.y(), p.z()] + color)

    return vertices
