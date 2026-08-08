""" Class for a scene object.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import numpy as np
from pathlib import Path
import tinyobjloader


class Material:

    def __init__(self, name: str=""):

        self.name: str = name
        self.diffuse: np.ndarray = np.array([0.8, 0.8, 0.8], dtype=np.float32)
        self.ambient: np.ndarray = np.array([0.2, 0.2, 0.2], dtype=np.float32)
        self.specular: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float32)


class Visual:
    """Holds CPU-side 3D geometry and material data ready for OpenGL upload."""
    def __init__(self):
        # Flattened NumPy arrays ready to pass to glBufferData
        self.vertices: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.normals: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.textures: np.ndarray = np.empty((0, 2), dtype=np.float32)
        self.indices: np.ndarray = np.empty((0,), dtype=np.uint32)

        self.material: Material | None = None
        self.colors: np.ndarray | None = None

        # GPU Buffer Handles (Populated later inside your QOpenGLWidget)
        self.vao_id: int | None = None
        self.vbo_id: int | None = None
        self.ebo_id: int | None = None

        self.line_render: bool = False  # Triangle or line rendering.

# def load_obj_file(file_path: Path) -> list[Visual]:
#     """Loads an .obj file using tinyobjloader and returns a list of Visual components."""
#     reader = tinyobjloader.ObjReader()
#     config = tinyobjloader.ObjReaderConfig()
#     config.triangulate = True
#     # config.search_path = str(file_path.parent)  # Search path for .mtl files
#
#     if not reader.ParseFromFile(str(file_path), config):
#         print(f"Failed to load {file_path}: {reader.Error()}")
#         return []
#
#     attrib = reader.GetAttrib()
#     shapes = reader.GetShapes()
#     materials = reader.GetMaterials()
#
#     visuals = []
#
#     # Get raw attrib arrays
#     v_buf = np.array(attrib.vertices, dtype=np.float32).reshape(-1, 3) if attrib.vertices else None
#     n_buf = np.array(attrib.normals, dtype=np.float32).reshape(-1, 3) if attrib.normals else None
#     t_buf = np.array(attrib.texcoords, dtype=np.float32).reshape(-1, 2) if attrib.texcoords else None
#
#     for shape in shapes:
#         mesh = shape.mesh
#         num_indices = len(mesh.indices)
#
#         # De-index / assemble attribute arrays per index
#         v_indices = [idx.vertex_index for idx in mesh.indices]
#         vertices = v_buf[v_indices] if v_buf is not None else np.empty((0, 3), dtype=np.float32)
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
#         # print(f"load_obj_file: Creating visual: vertices: {vertices.size}\tNormals: {normals.size}\tTextures: {texture_coordinates.size}\tIndices: {indices.size}")
#
#         # Associate Material if available
#         if mesh.material_ids and mesh.material_ids[0] >= 0:
#             mat_data = materials[mesh.material_ids[0]]
#             mat = Material(mat_data.name)
#             mat.diffuse = np.array(mat_data.diffuse, dtype=np.float32)
#             mat.ambient = np.array(mat_data.ambient, dtype=np.float32)
#             mat.specular = np.array(mat_data.specular, dtype=np.float32)
#             visual.material = mat
#
#         visuals.append(visual)
#
#     return visuals

def load_obj_file(file_path: Path, to_meters: bool = True) -> list[Visual]:
    """ Loads an .obj file using tinyobjloader and returns a list of Visual components.
    :param file_path: Path to the .obj file.
    :param to_meters: If True, converts vertex coordinates from mm to meters (scales by 0.001).
    """

    reader = tinyobjloader.ObjReader()
    config = tinyobjloader.ObjReaderConfig()
    config.triangulate = True  # Triangles only.
    # config.search_path = str(file_path.parent)  # Search path for .mtl files

    if not reader.ParseFromFile(str(file_path), config):
        print(f"Failed to load {file_path}: {reader.Error()}")
        return []

    attrib = reader.GetAttrib()  # Vertex, normal and texture coordinates
    shapes = reader.GetShapes()
    materials = reader.GetMaterials()

    visuals = []

    # Get raw attrib arrays
    v_buf = np.array(attrib.vertices, dtype=np.float32).reshape(-1, 3) if attrib.vertices else None     # Vertex coordinates, Nx3
    n_buf = np.array(attrib.normals, dtype=np.float32).reshape(-1, 3) if attrib.normals else None       # Normal vectors, Nx3
    t_buf = np.array(attrib.texcoords, dtype=np.float32).reshape(-1, 2) if attrib.texcoords else None   # Texture uv-coords, Nx2

    for shape in shapes:
        mesh = shape.mesh
        num_indices = len(mesh.indices)

        # De-index / assemble attribute arrays per index
        v_indices = [idx.vertex_index for idx in mesh.indices]
        vertices = v_buf[v_indices] if v_buf is not None else np.empty((0, 3), dtype=np.float32)

        # Scale vertices to meters if CAD model was exported in mm
        if to_meters and vertices.size > 0:
            vertices = vertices * 0.001

        normals = np.empty((0, 3), dtype=np.float32)
        if n_buf is not None:
            n_indices = [idx.normal_index for idx in mesh.indices if idx.normal_index >= 0]
            if len(n_indices) == num_indices:
                normals = n_buf[n_indices]

        texture_coordinates = np.empty((0, 2), dtype=np.float32)
        if t_buf is not None:
            uv_indices = [idx.texcoord_index for idx in mesh.indices if idx.texcoord_index >= 0]
            if len(uv_indices) == num_indices:
                texture_coordinates = t_buf[uv_indices]

        indices = np.arange(len(vertices), dtype=np.uint32)

        visual = Visual()
        visual.vertices = vertices
        visual.normals = normals
        visual.textures = texture_coordinates
        visual.indices = indices

        # Associate Material if available
        if mesh.material_ids and mesh.material_ids[0] >= 0:
            mat_data = materials[mesh.material_ids[0]]
            mat = Material(mat_data.name)
            mat.diffuse = np.array(mat_data.diffuse, dtype=np.float32)
            mat.ambient = np.array(mat_data.ambient, dtype=np.float32)
            mat.specular = np.array(mat_data.specular, dtype=np.float32)
            visual.material = mat
        # Initialize a default RGBA color array (1 color per vertex, default 80% grey, 100% opaque)
        num_vertices = len(vertices)
        vertex_colors = np.full((num_vertices, 4), [0.8, 0.8, 0.8, 1.0], dtype=np.float32)

        # If materials and material IDs per face exist
        if mesh.material_ids and len(materials) > 0:
            # Option A: Fast vectorization (if indices are mapped 1-to-1 or processed per triangle)
            # Map face material IDs to vertex indices
            tris = indices.reshape(-1, 3)

            for face_idx, mat_id in enumerate(mesh.material_ids):
                if 0 <= mat_id < len(materials):
                    mat_data = materials[mat_id]
                    diffuse = mat_data.diffuse  # [r, g, b]

                    # Extract alpha if material has it, otherwise default to 1.0
                    alpha = getattr(mat_data, 'dissolve', 1.0)  # 'dissolve' or 'alpha' in OBJ spec
                    color_rgba = [diffuse[0], diffuse[1], diffuse[2], alpha]

                    # Assign color to the 3 vertices belonging to this triangle face
                    v_indices = tris[face_idx]
                    vertex_colors[v_indices] = color_rgba

        # Assign the per-vertex color array to the Visual instance
        visual.colors = vertex_colors

        visuals.append(visual)

    return visuals