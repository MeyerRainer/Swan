""" Various renderable shapes.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import numpy as np
from pathlib import Path
from dataclasses import dataclass
import tinyobjloader
from qt_gui.viewport.scene.visuals.visual import Visual

@dataclass
class MeshSpecs:
    file_path: Path
    to_meters: bool = False
    to_millimeters: bool = False


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
            elif params.to_millimeters and mesh_vertices.size > 0:
                mesh_vertices = mesh_vertices * 1000

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