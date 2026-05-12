# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Generate synthetic PLY point cloud files for use in unit tests.

Run once before running the test suite:
    python tests/generate_test_data.py

Requires open3d and numpy.  Output goes to tests/sample_data/.
"""

import os
import numpy as np

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "sample_data")


def _save(pcd, filename):
    import open3d as o3d
    path = os.path.join(OUTPUT_DIR, filename)
    o3d.io.write_point_cloud(path, pcd)
    print(f"  Wrote {path}  ({len(pcd.points)} points)")


def make_flat_board(noise_std=0.1):
    """
    Simulated PCB top surface: 100×80 mm flat plane with Gaussian noise.
    Normal = (0, 0, 1).
    """
    import open3d as o3d
    rng = np.random.default_rng(42)
    x = rng.uniform(0, 100, 5000)
    y = rng.uniform(0, 80, 5000)
    z = rng.normal(0, noise_std, 5000)
    pts = np.column_stack([x, y, z])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(10)
    return pcd


def make_cylinder_part(noise_std=0.05):
    """
    Four cylinders of varying radius arranged on a plane base.
    Cylinders are vertical (axis = Z).
    """
    import open3d as o3d

    rng = np.random.default_rng(0)
    all_pts = []

    # Base plane
    x = rng.uniform(-60, 60, 2000)
    y = rng.uniform(-60, 60, 2000)
    z = rng.normal(0, noise_std, 2000)
    all_pts.append(np.column_stack([x, y, z]))

    cylinders = [
        ((-30, -30), 5.0, 20),
        (( 30, -30), 8.0, 25),
        ((-30,  30), 3.0, 15),
        (( 30,  30), 6.0, 30),
    ]
    for (cx, cy), radius, height in cylinders:
        n = 800
        theta = rng.uniform(0, 2 * np.pi, n)
        h = rng.uniform(0, height, n)
        px = cx + radius * np.cos(theta) + rng.normal(0, noise_std, n)
        py = cy + radius * np.sin(theta) + rng.normal(0, noise_std, n)
        pz = h
        all_pts.append(np.column_stack([px, py, pz]))

    pts = np.vstack(all_pts)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=3.0, max_nn=30)
    )
    return pcd


def make_mixed_scene(noise_std=0.1):
    """
    One plane + three cylinders + one sphere.
    Ground truth:
      Plane  : z=0, normal=(0,0,1)
      Cyl 0  : center=(-20,0), r=7, h=30
      Cyl 1  : center=(20,0),  r=5, h=20
      Cyl 2  : center=(0,25),  r=4, h=25
      Sphere : center=(0,-25,15), r=10
    """
    import open3d as o3d

    rng = np.random.default_rng(7)
    all_pts = []

    # Plane
    x = rng.uniform(-50, 50, 3000)
    y = rng.uniform(-50, 50, 3000)
    z = rng.normal(0, noise_std, 3000)
    all_pts.append(np.column_stack([x, y, z]))

    # Cylinders
    for (cx, cy), radius, height in [
        ((-20, 0), 7.0, 30),
        (( 20, 0), 5.0, 20),
        ((  0,25), 4.0, 25),
    ]:
        n = 600
        theta = rng.uniform(0, 2 * np.pi, n)
        h = rng.uniform(0, height, n)
        px = cx + radius * np.cos(theta) + rng.normal(0, noise_std, n)
        py = cy + radius * np.sin(theta) + rng.normal(0, noise_std, n)
        all_pts.append(np.column_stack([px, py, h]))

    # Sphere centre=(0,-25,15), r=10
    n = 800
    phi   = rng.uniform(0, np.pi, n)
    theta = rng.uniform(0, 2 * np.pi, n)
    r = 10.0
    px = r * np.sin(phi) * np.cos(theta) + rng.normal(0, noise_std, n)
    py = r * np.sin(phi) * np.sin(theta) + rng.normal(0, noise_std, n)
    pz = r * np.cos(phi) + 15 + rng.normal(0, noise_std, n)
    all_pts.append(np.column_stack([px, py - 25, pz]))

    pts = np.vstack(all_pts)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=3.0, max_nn=30)
    )
    return pcd


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("Generating test data …")
    _save(make_flat_board(),    "flat_board.ply")
    _save(make_cylinder_part(), "cylinder_part.ply")
    _save(make_mixed_scene(),   "mixed_scene.ply")
    print("Done.")
