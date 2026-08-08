import math
from typing import List, Tuple

from PyQt6.QtGui import QVector3D, QVector4D
from dataclasses import dataclass, field


@dataclass
class ArrowSpecs:
    axis: str = 'x'
    color: Tuple[float, float, float] = (1., 0., 0.)
    length_arrow: float  = 0.05
    length_cone: float = 0.015
    radius_shaft: float = 0.002
    radius_cone: float = 0.004
    n_segments: int = 12


@dataclass
class RingSpecs:
    normal_axis: str = 'x'
    color: Tuple[float, float, float] = (1., 0., 0.)
    radius_in: float = 0.04
    radius_out: float = 0.05
    n_segments: int = 32


@dataclass
class PlaneSpecs:
    u_dir: Tuple[float, float, float] = (0., 1., 0.)
    v_dir: Tuple[float, float, float] = (0., 0., 1.)
    color: Tuple[float, float, float] = (1., 0., 0.)
    offset: float = 0.001
    size: float = 0.030


# Translation arrow mesh creation along axis_dir.
def build_arrow(params: ArrowSpecs):

    length_shaft: float = params.length_arrow - params.length_cone

    vertices = []
    normals = []

    # Axis direction and orthogonal vectors relative to axis.
    if params.axis == 'x':
        axis_dir = QVector3D(1, 0, 0)
        u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
    elif params.axis == 'y':
        axis_dir = QVector3D(0, 1, 0)
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
    else:  # z
        axis_dir = QVector3D(0, 0, 1)
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

    # Arrow shaft
    for i in range(params.n_segments):
        # Increment in range [0, 1]
        a1: float = (i / params.n_segments) * 2 * math.pi
        a2: float = ((i + 1) / params.n_segments) * 2 * math.pi

        # Radial unit directions (equivalent to cylinder surface normals)
        n1: QVector3D = (math.cos(a1) * u_dir + math.sin(a1) * v_dir).normalized()
        n2: QVector3D = (math.cos(a2) * u_dir + math.sin(a2) * v_dir).normalized()

        # Radial vectors for shaft vertices
        r1: QVector3D = params.radius_shaft * n1
        r2: QVector3D = params.radius_shaft * n2

        # Circle vertices at the beginning and end of the shaft
        p1: QVector3D = r1
        p2: QVector3D = r1 + axis_dir * length_shaft
        p3: QVector3D = r2 + axis_dir * length_shaft
        p4: QVector3D = r2

        quad_data = [(p1, n1), (p2, n1), (p3, n2), (p1, n1), (p3, n2), (p4, n2)]
        for p, n in quad_data:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([n.x(), n.y(), n.z()])

    # Arrow tip
    tip_base: QVector3D = length_shaft * axis_dir
    tip_apex: QVector3D = (length_shaft + params.length_cone) * axis_dir

    # Cone slant angle normal adjustment factor
    cone_slope: float = params.radius_cone / params.length_cone

    for i in range(params.n_segments):
        a1: float = 2 * math.pi * (i / params.n_segments)
        a2: float = 2 * math.pi * ((i + 1) / params.n_segments)
        a_mid: float = (a1 + a2) * 0.5

        r1_dir = math.cos(a1) * u_dir + math.sin(a1) * v_dir
        r2_dir = math.cos(a2) * u_dir + math.sin(a2) * v_dir
        r_mid_dir: QVector3D = math.cos(a_mid) * u_dir + math.sin(a_mid) * v_dir

        # Slanted normals pointing outward from the cone face
        n1 = (r1_dir + axis_dir * cone_slope).normalized()
        n2 = (r2_dir + axis_dir * cone_slope).normalized()
        n_apex = (r_mid_dir + axis_dir * cone_slope).normalized()

        p1 = tip_base + params.radius_cone * r1_dir
        p2 = tip_base + params.radius_cone * r2_dir
        for p, n in [(p1, n1), (tip_apex, n_apex), (p2, n2)]:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([n.x(), n.y(), n.z()])

    return vertices, normals

# Rotation ring mesh creation normal to axis_dir
def build_rotation_ring(params: RingSpecs):

    vertices = []
    normals = []

    # Determine orthogonal plane basis vectors
    # Axis direction and orthogonal vectors relative to axis.
    if params.normal_axis == 'x':
        normal_dir = QVector3D(1, 0, 0)
        u_dir, v_dir = QVector3D(0, 1, 0), QVector3D(0, 0, 1)
    elif params.normal_axis == 'y':
        normal_dir = QVector3D(0, 1, 0)
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 0, 1)
    else:  # z
        normal_dir = QVector3D(0, 0, 1)
        u_dir, v_dir = QVector3D(1, 0, 0), QVector3D(0, 1, 0)

    for i in range(params.n_segments):
        # Increment in range [0, 1]
        a1: float = 2 * math.pi * (i / params.n_segments)
        a2: float = 2 * math.pi * ((i + 1) / params.n_segments)

        c1, s1 = math.cos(a1), math.sin(a1)
        c2, s2 = math.cos(a2), math.sin(a2)

        # Vertices for inner and outer rings on both sides
        p1_in: QVector3D = params.radius_in*(c1*u_dir + s1*v_dir)
        p1_out: QVector3D = params.radius_out*(c1*u_dir + s1*v_dir)
        p2_in: QVector3D = params.radius_in*(c2*u_dir + s2*v_dir)
        p2_out: QVector3D = params.radius_out*(c2*u_dir + s2*v_dir)

        # # Two triangles forming a quad ring segment
        # # Render both sides (clockwise & counter-clockwise) so ring is visible from any angle
        for p in [p1_in, p1_out, p2_out, p1_in, p2_out, p2_in]:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([normal_dir.x(), normal_dir.y(), normal_dir.z()])

        # Reverse side (-normal_dir)
        rev_normal = -normal_dir
        for p in [p1_in, p2_out, p1_out, p1_in, p2_in, p2_out]:
            vertices.extend([p.x(), p.y(), p.z()])
            normals.extend([rev_normal.x(), rev_normal.y(), rev_normal.z()])

    return vertices, normals

# Translation plane mesh creation normal to axis_dir
def build_plane(params):

    vertices = []
    normals = []
    u_dir = QVector3D(*params.u_dir)
    v_dir = QVector3D(*params.v_dir)

    p0: QVector3D = u_dir * params.offset + v_dir * params.offset
    p1: QVector3D = p0 + u_dir * params.size
    p2: QVector3D = p0 + u_dir * params.size + v_dir * params.size
    p3: QVector3D = p0 + v_dir * params.size

    # # Double-sided quad
    # Flat plane face normal
    normal: QVector3D = QVector3D.crossProduct(u_dir, v_dir).normalized()

    # Front side (+normal)
    for p in [p0, p1, p2, p0, p2, p3]:
        vertices.extend([p.x(), p.y(), p.z()])
        normals.extend([normal.x(), normal.y(), normal.z()])

    # Back side (-normal)
    rev_normal = -normal
    for p in [p0, p2, p1, p0, p3, p2]:
        vertices.extend([p.x(), p.y(), p.z()])
        normals.extend([rev_normal.x(), rev_normal.y(), rev_normal.z()])

    return vertices, normals
