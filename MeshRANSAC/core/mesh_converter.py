# SPDX-License-Identifier: LGPL-2.1-or-later
"""Convert a FreeCAD Mesh::Feature to an Open3D PointCloud."""

import numpy as np
from MeshRANSAC.utils.dependency_checker import require_open3d


def freecad_mesh_to_pointcloud(mesh_feature, voxel_size=None):
    """
    Convert a FreeCAD Mesh::Feature to an Open3D PointCloud.

    Args:
        mesh_feature: FreeCAD document object with a .Mesh attribute
                      (must be derived from Mesh::Feature)
        voxel_size:   float or None — voxel down-sample size in mm (None = skip)

    Returns:
        open3d.geometry.PointCloud with positions and normals set

    Raises:
        TypeError:  if mesh_feature does not have a .Mesh attribute
        ValueError: if the mesh contains no points
    """
    o3d = require_open3d()

    if not hasattr(mesh_feature, "Mesh"):
        raise TypeError(
            f"Expected a Mesh::Feature object, got {type(mesh_feature).__name__}. "
            "Select a mesh object in the model tree before running detection."
        )

    mesh = mesh_feature.Mesh

    if len(mesh.Points) == 0:
        raise ValueError(
            f"Mesh '{getattr(mesh_feature, 'Label', '?')}' contains no points. "
            "Import a scan mesh before running detection."
        )

    points = np.array([[p.x, p.y, p.z] for p in mesh.Points], dtype=np.float64)
    normals = _compute_vertex_normals(mesh)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)

    if voxel_size is not None and voxel_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size)

    return pcd


def _compute_vertex_normals(mesh):
    """
    Average face normals onto vertices.

    Returns a (N, 3) float64 array of unit normals, one per vertex.
    Vertices with no adjacent faces get the zero vector (degenerate mesh).
    """
    point_count = len(mesh.Points)
    normals = np.zeros((point_count, 3), dtype=np.float64)
    counts = np.zeros(point_count, dtype=np.float64)

    for facet in mesh.Facets:
        n = facet.Normal
        for idx in facet.PointIndices:
            normals[idx] += (n.x, n.y, n.z)
            counts[idx] += 1.0

    # Avoid division by zero for isolated vertices
    counts = np.maximum(counts, 1.0)
    normals /= counts[:, np.newaxis]

    # Normalize to unit length
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-8)
    return normals / norms
