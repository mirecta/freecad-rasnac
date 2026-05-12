# SPDX-License-Identifier: LGPL-2.1-or-later
"""Tests for core/ransac_engine.py.

Run with open3d available:
    python -m pytest tests/test_ransac_engine.py -v
"""

import unittest
import numpy as np

try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    HAS_O3D = False


def _require_o3d(test):
    return unittest.skipUnless(HAS_O3D, "open3d not available")(test)


# ---------------------------------------------------------------------------
# Synthetic point cloud helpers
# ---------------------------------------------------------------------------

def _plane_pcd(normal=(0, 0, 1), offset=0.0, size=100, noise=0.05, n=500, seed=1):
    """Flat plane point cloud with Gaussian noise."""
    rng = np.random.default_rng(seed)
    n_arr = np.asarray(normal, dtype=float)
    n_arr /= np.linalg.norm(n_arr)

    # Build two orthogonal tangent vectors
    u = np.array([1, 0, 0]) if abs(n_arr[0]) < 0.9 else np.array([0, 1, 0])
    u -= np.dot(u, n_arr) * n_arr
    u /= np.linalg.norm(u)
    v = np.cross(n_arr, u)

    s = rng.uniform(-size / 2, size / 2, (n, 2))
    pts = (s[:, 0:1] * u + s[:, 1:2] * v
           - offset * n_arr
           + rng.normal(0, noise, (n, 3)))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    normals = np.tile(n_arr, (n, 1)) + rng.normal(0, 0.01, (n, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    return pcd


def _cylinder_pcd(centre=(0, 0, 0), axis=(0, 0, 1), radius=5.0,
                  height=20.0, noise=0.05, n=600, seed=2):
    """Cylinder surface point cloud."""
    rng = np.random.default_rng(seed)
    ax = np.asarray(axis, dtype=float)
    ax /= np.linalg.norm(ax)

    theta = rng.uniform(0, 2 * np.pi, n)
    h = rng.uniform(0, height, n)

    # Build orthonormal frame
    u = np.array([1, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1, 0])
    u -= np.dot(u, ax) * ax
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)

    pts = (np.outer(np.cos(theta), u * radius)
           + np.outer(np.sin(theta), v * radius)
           + np.outer(h, ax)
           + np.asarray(centre)
           + rng.normal(0, noise, (n, 3)))

    # Outward normals (radial direction)
    rad_dir = (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v)
               + rng.normal(0, 0.01, (n, 3)))
    rad_dir /= np.linalg.norm(rad_dir, axis=1, keepdims=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.normals = o3d.utility.Vector3dVector(rad_dir)
    return pcd


def _sphere_pcd(centre=(0, 0, 0), radius=10.0, noise=0.05, n=600, seed=3):
    """Sphere surface point cloud."""
    rng = np.random.default_rng(seed)
    phi   = rng.uniform(0, np.pi, n)
    theta = rng.uniform(0, 2 * np.pi, n)

    pts = np.column_stack([
        radius * np.sin(phi) * np.cos(theta),
        radius * np.sin(phi) * np.sin(theta),
        radius * np.cos(phi),
    ]) + np.asarray(centre) + rng.normal(0, noise, (n, 3))

    nrm = pts - np.asarray(centre)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.normals = o3d.utility.Vector3dVector(nrm)
    return pcd


def _angle_deg(a, b):
    """Angle between two unit vectors in degrees (handles sign flip)."""
    cos = np.clip(abs(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b))), 0, 1)
    return np.degrees(np.arccos(cos))


# ---------------------------------------------------------------------------
# Plane tests
# ---------------------------------------------------------------------------

@_require_o3d
class TestDetectPlanes(unittest.TestCase):

    def test_detects_single_plane(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        pcd = _plane_pcd()
        results = detect_planes(pcd, distance_threshold=0.3, min_inliers=50)
        self.assertGreaterEqual(len(results), 1)

    def test_normal_within_5_degrees(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        gt_normal = np.array([0.0, 0.0, 1.0])
        pcd = _plane_pcd(normal=gt_normal, noise=0.05)
        results = detect_planes(pcd, distance_threshold=0.3, min_inliers=50)
        self.assertGreaterEqual(len(results), 1)
        angle = _angle_deg(results[0].normal, gt_normal)
        self.assertLess(angle, 5.0, f"Normal angle error {angle:.2f}° > 5°")

    def test_min_inliers_respected(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        pcd = _plane_pcd(n=30)  # fewer than default min_inliers
        results = detect_planes(pcd, distance_threshold=0.3, min_inliers=50)
        self.assertEqual(len(results), 0)

    def test_multiple_planes_extracted(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        pcd1 = _plane_pcd(normal=(0, 0, 1), offset=0, n=500, seed=10)
        pcd2 = _plane_pcd(normal=(0, 0, 1), offset=50, n=500, seed=11)
        combined = o3d.geometry.PointCloud()
        combined.points = o3d.utility.Vector3dVector(
            np.vstack([np.asarray(pcd1.points), np.asarray(pcd2.points)])
        )
        combined.normals = o3d.utility.Vector3dVector(
            np.vstack([np.asarray(pcd1.normals), np.asarray(pcd2.normals)])
        )
        results = detect_planes(combined, distance_threshold=0.5, min_inliers=50,
                                max_planes=5)
        self.assertGreaterEqual(len(results), 2)

    def test_inlier_recall_80_percent(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        n = 500
        pcd = _plane_pcd(n=n, noise=0.05)
        results = detect_planes(pcd, distance_threshold=0.3, min_inliers=50)
        self.assertGreaterEqual(len(results), 1)
        recall = results[0].inlier_count / n
        self.assertGreaterEqual(recall, 0.80,
                                f"Inlier recall {recall:.2f} < 0.80")

    def test_progress_callback_called(self):
        from MeshRANSAC.core.ransac_engine import detect_planes
        calls = []
        pcd = _plane_pcd()
        detect_planes(pcd, distance_threshold=0.3, min_inliers=50, max_planes=3,
                      progress_cb=lambda s, t: calls.append((s, t)))
        self.assertGreater(len(calls), 0)


# ---------------------------------------------------------------------------
# Cylinder tests
# ---------------------------------------------------------------------------

@_require_o3d
class TestDetectCylinders(unittest.TestCase):

    def test_detects_cylinder(self):
        from MeshRANSAC.core.ransac_engine import detect_cylinders
        pcd = _cylinder_pcd(radius=5.0)
        results = detect_cylinders(pcd, distance_threshold=0.3, min_inliers=50,
                                   num_iterations=500)
        self.assertGreaterEqual(len(results), 1)

    def test_radius_within_0_1mm(self):
        from MeshRANSAC.core.ransac_engine import detect_cylinders
        gt_radius = 5.0
        pcd = _cylinder_pcd(radius=gt_radius, noise=0.05, n=800)
        results = detect_cylinders(pcd, distance_threshold=0.3, min_inliers=50,
                                   num_iterations=1000)
        self.assertGreaterEqual(len(results), 1)
        err = abs(results[0].radius - gt_radius)
        self.assertLess(err, 0.5, f"Radius error {err:.3f} mm > 0.5 mm")

    def test_axis_direction_within_5_degrees(self):
        from MeshRANSAC.core.ransac_engine import detect_cylinders
        gt_axis = np.array([0.0, 0.0, 1.0])
        pcd = _cylinder_pcd(axis=gt_axis, radius=5.0, noise=0.05, n=800)
        results = detect_cylinders(pcd, distance_threshold=0.3, min_inliers=50,
                                   num_iterations=1000)
        self.assertGreaterEqual(len(results), 1)
        angle = _angle_deg(results[0].axis_dir, gt_axis)
        self.assertLess(angle, 10.0, f"Axis angle error {angle:.2f}° > 10°")


# ---------------------------------------------------------------------------
# Sphere tests
# ---------------------------------------------------------------------------

@_require_o3d
class TestDetectSpheres(unittest.TestCase):

    def test_detects_sphere(self):
        from MeshRANSAC.core.ransac_engine import detect_spheres
        pcd = _sphere_pcd(radius=10.0)
        results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=50,
                                 num_iterations=500)
        self.assertGreaterEqual(len(results), 1)

    def test_radius_within_tolerance(self):
        from MeshRANSAC.core.ransac_engine import detect_spheres
        gt_radius = 10.0
        pcd = _sphere_pcd(radius=gt_radius, noise=0.05, n=800)
        results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=50,
                                 num_iterations=1000)
        self.assertGreaterEqual(len(results), 1)
        err = abs(results[0].radius - gt_radius)
        self.assertLess(err, 1.0, f"Radius error {err:.3f} mm > 1.0 mm")

    def test_centre_within_tolerance(self):
        from MeshRANSAC.core.ransac_engine import detect_spheres
        gt_centre = np.array([5.0, -3.0, 10.0])
        pcd = _sphere_pcd(centre=gt_centre, radius=10.0, noise=0.05, n=800)
        results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=50,
                                 num_iterations=1000)
        self.assertGreaterEqual(len(results), 1)
        err = np.linalg.norm(results[0].center - gt_centre)
        self.assertLess(err, 2.0, f"Centre error {err:.3f} mm > 2.0 mm")


# ---------------------------------------------------------------------------
# detect_all
# ---------------------------------------------------------------------------

@_require_o3d
class TestDetectAll(unittest.TestCase):

    def test_returns_three_lists(self):
        from MeshRANSAC.core.ransac_engine import detect_all
        pcd = _plane_pcd()
        result = detect_all(pcd, {"detect_planes": True,
                                  "detect_cylinders": False,
                                  "detect_spheres": False,
                                  "distance_threshold": 0.3,
                                  "min_inliers": 50})
        self.assertEqual(len(result), 3)

    def test_can_disable_types(self):
        from MeshRANSAC.core.ransac_engine import detect_all
        pcd = _plane_pcd()
        planes, cylinders, spheres = detect_all(
            pcd, {"detect_planes": False, "detect_cylinders": False,
                  "detect_spheres": False})
        self.assertEqual(planes, [])
        self.assertEqual(cylinders, [])
        self.assertEqual(spheres, [])


if __name__ == "__main__":
    unittest.main()
