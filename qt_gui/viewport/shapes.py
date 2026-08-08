from qt_gui.viewport.visuals.visual import Visual, Material

from pathlib import Path
import tinyobjloader
import math
from typing import Tuple
import numpy as np
from PyQt6.QtGui import QVector3D
from dataclasses import dataclass, field


@dataclass
class ArrowSpecs:
    axis: str = 'x'
    colors: Tuple[float, float, float, float] = (1., 0.2, 0.2, 0.8)
    length_arrow: float  = 0.05
    length_cone: float = 0.015
    radius_shaft: float = 0.002
    radius_cone: float = 0.004
    n_segments: int = 12


@dataclass
class RingSpecs:
    normal_axis: str = 'x'
    colors: Tuple[float, float, float, float] = (1., 0., 0., 0.5)
    radius_in: float = 0.04
    radius_out: float = 0.05
    n_segments: int = 32


@dataclass
class PlaneSpecs:
    u_dir: Tuple[float, float, float] = (0., 1., 0.)
    v_dir: Tuple[float, float, float] = (0., 0., 1.)
    colors: Tuple[float, float, float, float] = (1., 0., 0., 0.5)
    offset: float = 0.001
    size: float = 0.030


class DragArrow(Visual):

    def __init__(self, params: ArrowSpecs):

        super().__init__()

        self._params = params

        self._build(self._params)

    def _build(self, params: ArrowSpecs):

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

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(params.colors, (len(self.vertices), 1))


class RotationRing(Visual):

    def __init__(self, params: RingSpecs):

        super().__init__()

        self._params = params

        self._build(self._params)

    def _build(self, params: RingSpecs):

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

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(self._params.colors, (len(self.vertices), 1))


# Translation plane mesh creation normal to axis_dir
class DragPlane(Visual):

    def __init__(self, params: PlaneSpecs):

        super().__init__()

        self._params = params


        self._build(self._params)

    def _build(self, params: PlaneSpecs):

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

        self.vertices = np.array(vertices, dtype=np.float32).reshape(-1, 3)
        self.normals = np.array(normals, dtype=np.float32).reshape(-1, 3)
        self.indices = np.arange(len(self.vertices), dtype=np.uint32)
        self.colors = np.tile(self._params.colors, (len(self.vertices), 1))


@dataclass
class MeshSpecs:
    file_path: Path
    to_meters: bool = True


class MeshObject(Visual):

    def __init__(self, params: MeshSpecs):

        super().__init__()

        self._params = params

        self._build(self._params)


    def _build(self, params: MeshSpecs) -> None:
        """Loads an .obj file using tinyobjloader and merges all meshes into this Visual instance."""
        reader = tinyobjloader.ObjReader()
        config = tinyobjloader.ObjReaderConfig()
        config.triangulate = True  # Triangles only

        if not reader.ParseFromFile(str(params.file_path), config):
            print(f"Failed to load {params.file_path}: {reader.Error()}")
            return

        attrib = reader.GetAttrib()
        shapes = reader.GetShapes()
        materials = reader.GetMaterials()

        # Raw attribute buffers
        v_buf = np.array(attrib.vertices, dtype=np.float32).reshape(-1, 3) if attrib.vertices else None
        n_buf = np.array(attrib.normals, dtype=np.float32).reshape(-1, 3) if attrib.normals else None
        t_buf = np.array(attrib.texcoords, dtype=np.float32).reshape(-1, 2) if attrib.texcoords else None

        # Lists to accumulate data across all shapes
        all_vertices = []
        all_normals = []
        all_textures = []
        all_colors = []
        all_indices = []

        vertex_offset = 0

        for shape in shapes:
            mesh = shape.mesh
            num_indices = len(mesh.indices)
            if num_indices == 0:
                continue

            # 1. Un-index / assemble positions
            v_indices = [idx.vertex_index for idx in mesh.indices]
            mesh_vertices = v_buf[v_indices] if v_buf is not None else np.empty((0, 3), dtype=np.float32)

            if params.to_meters and mesh_vertices.size > 0:
                mesh_vertices = mesh_vertices * 0.001

            num_vertices = len(mesh_vertices)

            # 2. Un-index / assemble normals
            mesh_normals = np.zeros((num_vertices, 3), dtype=np.float32)
            if n_buf is not None:
                n_indices = [idx.normal_index for idx in mesh.indices if idx.normal_index >= 0]
                if len(n_indices) == num_indices:
                    mesh_normals = n_buf[n_indices]

            # 3. Un-index / assemble texture coordinates
            mesh_textures = np.zeros((num_vertices, 2), dtype=np.float32)
            if t_buf is not None:
                uv_indices = [idx.texcoord_index for idx in mesh.indices if idx.texcoord_index >= 0]
                if len(uv_indices) == num_indices:
                    mesh_textures = t_buf[uv_indices]

            # 4. Generate per-vertex RGBA colors based on materials
            mesh_colors = np.full((num_vertices, 4), [0.8, 0.8, 0.8, 1.0], dtype=np.float32)

            if mesh.material_ids and len(materials) > 0:
                tris = np.arange(num_vertices, dtype=np.uint32).reshape(-1, 3)

                for face_idx, mat_id in enumerate(mesh.material_ids):
                    if 0 <= mat_id < len(materials):
                        mat_data = materials[mat_id]
                        diffuse = mat_data.diffuse
                        alpha = getattr(mat_data, 'dissolve', 1.0)
                        color_rgba = [diffuse[0], diffuse[1], diffuse[2], alpha]

                        v_idx = tris[face_idx]
                        mesh_colors[v_idx] = color_rgba

            # 5. Generate offset indices so merged meshes point to correct global vertices
            mesh_indices = np.arange(num_vertices, dtype=np.uint32) + vertex_offset

            # Append to accumulator lists
            all_vertices.append(mesh_vertices)
            all_normals.append(mesh_normals)
            all_textures.append(mesh_textures)
            all_colors.append(mesh_colors)
            all_indices.append(mesh_indices)

            vertex_offset += num_vertices

        # Combine into single arrays and write directly to self attributes
        if all_vertices:
            self.vertices = np.vstack(all_vertices).astype(np.float32)
            self.normals = np.vstack(all_normals).astype(np.float32)
            self.textures = np.vstack(all_textures).astype(np.float32)
            self.colors = np.vstack(all_colors).astype(np.float32)
            self.indices = np.concatenate(all_indices).astype(np.uint32)
        else:
            self.vertices = np.empty((0, 3), dtype=np.float32)
            self.normals = np.empty((0, 3), dtype=np.float32)
            self.textures = np.empty((0, 2), dtype=np.float32)
            self.colors = np.empty((0, 4), dtype=np.float32)
            self.indices = np.empty((0,), dtype=np.uint32)


    # def _build(self, params: MeshSpecs) -> None:
    #     """ Loads an .obj file using tinyobjloader and returns a list of Visual components.
    #     """
    #
    #     reader = tinyobjloader.ObjReader()
    #     config = tinyobjloader.ObjReaderConfig()
    #     config.triangulate = True  # Triangles only.
    #     # config.search_path = str(file_path.parent)  # Search path for .mtl files
    #
    #     if not reader.ParseFromFile(str(params.file_path), config):
    #         print(f"Failed to load {params.file_path}: {reader.Error()}")
    #         return
    #
    #     attrib = reader.GetAttrib()  # Vertex, normal and texture coordinates
    #     shapes = reader.GetShapes()
    #     materials = reader.GetMaterials()
    #
    #     visuals = []
    #
    #     # Get raw attrib arrays
    #     v_buf = np.array(attrib.vertices, dtype=np.float32).reshape(-1, 3) if attrib.vertices else None  # Vertex coordinates, Nx3
    #     n_buf = np.array(attrib.normals, dtype=np.float32).reshape(-1, 3) if attrib.normals else None  # Normal vectors, Nx3
    #     t_buf = np.array(attrib.texcoords, dtype=np.float32).reshape(-1, 2) if attrib.texcoords else None  # Texture uv-coords, Nx2
    #
    #     for shape in shapes:
    #         mesh = shape.mesh
    #         num_indices = len(mesh.indices)
    #
    #         # De-index / assemble attribute arrays per index
    #         v_indices = [idx.vertex_index for idx in mesh.indices]
    #         vertices = v_buf[v_indices] if v_buf is not None else np.empty((0, 3), dtype=np.float32)
    #
    #         # Scale vertices to meters if CAD model was exported in mm
    #         if params.to_meters and vertices.size > 0:
    #             vertices = vertices * 0.001
    #
    #         normals = np.empty((0, 3), dtype=np.float32)
    #         if n_buf is not None:
    #             n_indices = [idx.normal_index for idx in mesh.indices if idx.normal_index >= 0]
    #             if len(n_indices) == num_indices:
    #                 normals = n_buf[n_indices]
    #
    #         texture_coordinates = np.empty((0, 2), dtype=np.float32)
    #         if t_buf is not None:
    #             uv_indices = [idx.texcoord_index for idx in mesh.indices if idx.texcoord_index >= 0]
    #             if len(uv_indices) == num_indices:
    #                 texture_coordinates = t_buf[uv_indices]
    #
    #         indices = np.arange(len(vertices), dtype=np.uint32)
    #
    #         visual = Visual()
    #         visual.vertices = vertices
    #         visual.normals = normals
    #         visual.textures = texture_coordinates
    #         visual.indices = indices
    #
    #         # Associate Material if available
    #         if mesh.material_ids and mesh.material_ids[0] >= 0:
    #             mat_data = materials[mesh.material_ids[0]]
    #             mat = Material(mat_data.name)
    #             mat.diffuse = np.array(mat_data.diffuse, dtype=np.float32)
    #             mat.ambient = np.array(mat_data.ambient, dtype=np.float32)
    #             mat.specular = np.array(mat_data.specular, dtype=np.float32)
    #             visual.material = mat
    #         # Initialize a default RGBA color array (1 color per vertex, default 80% grey, 100% opaque)
    #         num_vertices = len(vertices)
    #         vertex_colors = np.full((num_vertices, 4), [0.8, 0.8, 0.8, 1.0], dtype=np.float32)
    #
    #         # If materials and material IDs per face exist
    #         if mesh.material_ids and len(materials) > 0:
    #             # Option A: Fast vectorization (if indices are mapped 1-to-1 or processed per triangle)
    #             # Map face material IDs to vertex indices
    #             tris = indices.reshape(-1, 3)
    #
    #             for face_idx, mat_id in enumerate(mesh.material_ids):
    #                 if 0 <= mat_id < len(materials):
    #                     mat_data = materials[mat_id]
    #                     diffuse = mat_data.diffuse  # [r, g, b]
    #
    #                     # Extract alpha if material has it, otherwise default to 1.0
    #                     alpha = getattr(mat_data, 'dissolve', 1.0)  # 'dissolve' or 'alpha' in OBJ spec
    #                     color_rgba = [diffuse[0], diffuse[1], diffuse[2], alpha]
    #
    #                     # Assign color to the 3 vertices belonging to this triangle face
    #                     v_indices = tris[face_idx]
    #                     vertex_colors[v_indices] = color_rgba
    #
    #         # Assign the per-vertex color array to the Visual instance
    #         visual.colors = vertex_colors
    #
    #         visuals.append(visual)
    #
    #     return visuals