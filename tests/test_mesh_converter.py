# SPDX-License-Identifier: LGPL-2.1-or-later
"""Tests for core/mesh_converter.py.

Run headlessly:
    freecad --console tests/test_runner.py
Or directly (open3d + numpy must be importable):
    python -m pytest tests/test_mesh_converter.py
"""

import unittest
import numpy as np
from types import SimpleNamespace


# ---------------------------------------------------------------------------
# Minimal FreeCAD mesh object stubs (no FreeCAD runtime needed)
# ---------------------------------------------------------------------------

class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Facet:
    def __init__(self, p0, p1, p2, normal):
        self.PointIndices = (p0, p1, p2)
        self.Normal = _Vec(*normal)


def _make_mesh(points, facets):
    """Build a minimal mesh stub compatible with mesh_converter."""
    mesh = SimpleNamespace()
    mesh.Points = [_Vec(*p) for p in points]
    mesh.Facets = [_Facet(*f) for f in facets]
    return mesh


def _make_feature(points, facets):
    """Wrap a mesh stub in a Mesh::Feature-like object."""
    feat = SimpleNamespace()
    feat.Mesh = _make_mesh(points, facets)
    feat.Label = "TestMesh"
    return feat


# A single triangle in the XY plane (normal = +Z)
_TRIANGLE_PTS = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
_TRIANGLE_FAC = [(0, 1, 2, (0, 0, 1))]

# Simple box-like grid: two triangles forming a quad in XY plane
_QUAD_PTS = [(0,0,0),(1,0,0),(1,1,0),(0,1,0)]
_QUAD_FAC = [(0,1,2,(0,0,1)), (0,2,3,(0,0,1))]


class TestComputeVertexNormals(unittest.TestCase):
    """Test _compute_vertex_normals directly without open3d."""

    def setUp(self):
        from MeshRANSAC.core.mesh_converter import _compute_vertex_normals
        self.fn = _compute_vertex_normals

    def test_shape(self):
        mesh = _make_mesh(_TRIANGLE_PTS, _TRIANGLE_FAC)
        n = self.fn(mesh)
        self.assertEqual(n.shape, (3, 3))

    def test_unit_length(self):
        mesh = _make_mesh(_TRIANGLE_PTS, _TRIANGLE_FAC)
        n = self.fn(mesh)
        norms = np.linalg.norm(n, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6)

    def test_xy_plane_normal_is_z(self):
        mesh = _make_mesh(_TRIANGLE_PTS, _TRIANGLE_FAC)
        n = self.fn(mesh)
        # All vertices of a flat triangle should have normal (0,0,1)
        np.testing.assert_allclose(n, [[0,0,1],[0,0,1],[0,0,1]], atol=1e-6)

    def test_shared_vertex_averages(self):
        """A vertex shared by two coplanar faces keeps the same normal."""
        mesh = _make_mesh(_QUAD_PTS, _QUAD_FAC)
        n = self.fn(mesh)
        np.testing.assert_allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-6)
        # All normals still point in +Z
        np.testing.assert_allclose(n[:, 2], 1.0, atol=1e-6)

    def test_empty_points(self):
        mesh = _make_mesh([], [])
        n = self.fn(mesh)
        self.assertEqual(n.shape, (0, 3))


class TestFreecadMeshToPointcloud(unittest.TestCase):

    def test_returns_pointcloud_with_correct_size(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        feat = _make_feature(_QUAD_PTS, _QUAD_FAC)
        pcd = freecad_mesh_to_pointcloud(feat)
        self.assertEqual(len(pcd.points), 4)

    def test_has_normals(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        feat = _make_feature(_QUAD_PTS, _QUAD_FAC)
        pcd = freecad_mesh_to_pointcloud(feat)
        self.assertTrue(pcd.has_normals())

    def test_normals_unit_length(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        feat = _make_feature(_QUAD_PTS, _QUAD_FAC)
        pcd = freecad_mesh_to_pointcloud(feat)
        n = np.asarray(pcd.normals)
        np.testing.assert_allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-6)

    def test_voxel_downsample_reduces_points(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        # Dense grid of 100 points spread over 10mm — large voxel collapses most
        pts = [(x*0.1, y*0.1, 0) for x in range(10) for y in range(10)]
        fac = []  # normals come from facets; use flat z=1 normal manually
        feat = _make_feature(pts, [])
        # Manually override normals to avoid zero-division on empty facets
        feat.Mesh.Facets = []
        pcd = freecad_mesh_to_pointcloud(feat, voxel_size=5.0)
        self.assertLess(len(pcd.points), 100)

    def test_voxel_none_keeps_all_points(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        feat = _make_feature(_QUAD_PTS, _QUAD_FAC)
        pcd = freecad_mesh_to_pointcloud(feat, voxel_size=None)
        self.assertEqual(len(pcd.points), 4)

    def test_wrong_object_type_raises_type_error(self):
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        with self.assertRaises(TypeError):
            freecad_mesh_to_pointcloud(object())

    def test_empty_mesh_raises_value_error(self):
        try:
            import open3d as o3d
        except ImportError:
            self.skipTest("open3d not available")
        from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
        feat = _make_feature([], [])
        with self.assertRaises(ValueError):
            freecad_mesh_to_pointcloud(feat)


if __name__ == "__main__":
    unittest.main()
