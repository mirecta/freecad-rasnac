#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Visualize RANSAC detection results as reconstructed primitive meshes.

Shows the original point cloud in grey alongside solid mesh reconstructions:
  blue   plane slab
  green  cylinder solid
  orange sphere solid

Run:
    .venv/bin/python tests/visualize_reconstructed.py          # all scenes
    .venv/bin/python tests/visualize_reconstructed.py 5        # scene 5 only
"""

import sys
import os
import numpy as np

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo not in sys.path:
    sys.path.insert(0, _repo)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not installed.  Run:  uv pip install open3d")

from MeshRANSAC.core.ransac_engine import (
    detect_planes, detect_cylinders, detect_spheres,
    PlaneResult, CylinderResult, SphereResult,
)

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
GREY      = [0.45, 0.45, 0.45]
BLUE      = [0.25, 0.55, 1.00]
BLUE2     = [0.45, 0.70, 1.00]
GREEN     = [0.10, 0.80, 0.25]
GREEN2    = [0.30, 1.00, 0.50]
ORANGE    = [1.00, 0.55, 0.05]
ORANGE2   = [1.00, 0.75, 0.20]
YELLOW    = [0.95, 0.85, 0.05]

_P_PAL = [BLUE,   BLUE2]
_C_PAL = [GREEN,  GREEN2]
_S_PAL = [ORANGE, ORANGE2]

# ---------------------------------------------------------------------------
# Synthetic scene helpers (same as visualize_detection.py)
# ---------------------------------------------------------------------------

def _make_plane(normal=(0,0,1), offset=0, size=80, n=2000, noise=0.08, seed=0):
    rng = np.random.default_rng(seed)
    n_arr = np.asarray(normal, float); n_arr /= np.linalg.norm(n_arr)
    u = np.array([1.,0.,0.]) if abs(n_arr[0]) < 0.9 else np.array([0.,1.,0.])
    u -= np.dot(u, n_arr)*n_arr; u /= np.linalg.norm(u)
    v = np.cross(n_arr, u)
    s = rng.uniform(-size/2, size/2, (n, 2))
    pts = s[:,0:1]*u + s[:,1:2]*v - offset*n_arr + rng.normal(0, noise, (n,3))
    nrm = np.tile(n_arr, (n,1)) + rng.normal(0, 0.01, (n,3))
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return pts, nrm

def _make_cylinder(centre=(0,0,0), axis=(0,0,1), radius=5.0,
                   height=30.0, n=1000, noise=0.06, seed=1):
    rng = np.random.default_rng(seed)
    ax = np.asarray(axis, float); ax /= np.linalg.norm(ax)
    u = np.array([1.,0.,0.]) if abs(ax[0]) < 0.9 else np.array([0.,1.,0.])
    u -= np.dot(u, ax)*ax; u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    theta = rng.uniform(0, 2*np.pi, n)
    h     = rng.uniform(0, height, n)
    pts = (np.outer(np.cos(theta), u*radius)
           + np.outer(np.sin(theta), v*radius)
           + np.outer(h, ax)
           + np.asarray(centre)
           + rng.normal(0, noise, (n,3)))
    rad = np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v)
    rad += rng.normal(0, 0.01, (n,3)); rad /= np.linalg.norm(rad, axis=1, keepdims=True)
    return pts, rad

def _make_sphere(centre=(0,0,0), radius=12.0, n=900, noise=0.07, seed=2):
    rng = np.random.default_rng(seed)
    phi = rng.uniform(0, np.pi, n); theta = rng.uniform(0, 2*np.pi, n)
    pts = np.column_stack([
        radius*np.sin(phi)*np.cos(theta),
        radius*np.sin(phi)*np.sin(theta),
        radius*np.cos(phi),
    ]) + np.asarray(centre) + rng.normal(0, noise, (n,3))
    nrm = pts - np.asarray(centre)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return pts, nrm

def _pcd(*parts):
    pts = np.vstack([p for p,_ in parts])
    nrm = np.vstack([n for _,n in parts])
    pc = o3d.geometry.PointCloud()
    pc.points  = o3d.utility.Vector3dVector(pts)
    pc.normals = o3d.utility.Vector3dVector(nrm)
    return pc, len(pts)

# ---------------------------------------------------------------------------
# Rotation helper: align mesh Z-axis to target direction
# ---------------------------------------------------------------------------

def _rot_z_to(target):
    """3×3 rotation matrix that rotates [0,0,1] to unit vector `target`."""
    z = np.array([0., 0., 1.])
    t = np.asarray(target, float); t /= np.linalg.norm(t)
    v = np.cross(z, t)
    c = float(np.dot(z, t))
    if abs(c + 1.0) < 1e-6:                        # 180° flip
        return np.diag([1., -1., -1.])
    s = np.linalg.norm(v)
    if s < 1e-6:                                    # already aligned
        return np.eye(3)
    K = np.array([[    0, -v[2],  v[1]],
                  [ v[2],     0, -v[0]],
                  [-v[1],  v[0],     0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (s * s))

# ---------------------------------------------------------------------------
# Mesh builders from RANSAC results
# ---------------------------------------------------------------------------

def _plane_mesh(result: PlaneResult, color):
    pts = result.bounding_box
    edges = sorted([np.linalg.norm(pts[1]-pts[0]),
                    np.linalg.norm(pts[3]-pts[0]),
                    np.linalg.norm(pts[4]-pts[0])], reverse=True)
    w, l = max(edges[0], 1.0), max(edges[1], 1.0)
    thickness = 1.5

    mesh = o3d.geometry.TriangleMesh.create_box(w, l, thickness)
    mesh.compute_vertex_normals()
    # Centre box at origin before rotating
    mesh.translate([-w/2, -l/2, -thickness/2])

    R = _rot_z_to(result.normal)
    mesh.rotate(R, center=(0, 0, 0))
    mesh.translate(result.center)
    mesh.paint_uniform_color(color)
    return mesh


def _cylinder_mesh(result: CylinderResult, color):
    mesh = o3d.geometry.TriangleMesh.create_cylinder(
        radius=max(result.radius, 0.5),
        height=max(result.height, 1.0),
        resolution=40,
    )
    mesh.compute_vertex_normals()
    R = _rot_z_to(result.axis_dir)
    mesh.rotate(R, center=(0, 0, 0))
    mesh.translate(result.center)
    mesh.paint_uniform_color(color)
    return mesh


def _sphere_mesh(result: SphereResult, color):
    mesh = o3d.geometry.TriangleMesh.create_sphere(
        radius=max(result.radius, 0.5), resolution=30
    )
    mesh.compute_vertex_normals()
    mesh.translate(result.center)
    mesh.paint_uniform_color(color)
    return mesh

# ---------------------------------------------------------------------------
# Render: grey point cloud + coloured meshes
# ---------------------------------------------------------------------------

def _render(pcd, meshes, filename):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, filename)

    pcd_grey = o3d.geometry.PointCloud()
    pcd_grey.points = pcd.points
    n = len(pcd.points)
    pcd_grey.colors = o3d.utility.Vector3dVector(np.tile(GREY, (n, 1)))

    render = o3d.visualization.rendering.OffscreenRenderer(1280, 800)

    pt_mat = o3d.visualization.rendering.MaterialRecord()
    pt_mat.shader = "defaultUnlit"
    pt_mat.point_size = 2.0
    render.scene.add_geometry("pcd", pcd_grey, pt_mat)

    mesh_mat = o3d.visualization.rendering.MaterialRecord()
    mesh_mat.shader = "defaultLit"
    mesh_mat.base_roughness = 0.5
    mesh_mat.base_reflectance = 0.1

    for i, m in enumerate(meshes):
        render.scene.add_geometry(f"mesh_{i}", m, mesh_mat)

    render.scene.set_background([0.10, 0.10, 0.10, 1.0])
    render.scene.scene.set_sun_light(
        [0.6, -0.8, -0.5], [1.0, 1.0, 1.0], 80000
    )
    render.scene.scene.enable_sun_light(True)

    # Camera: isometric view covering all geometry
    bbox = pcd_grey.get_axis_aligned_bounding_box()
    center = np.asarray(bbox.get_center())
    extent = np.asarray(bbox.get_extent())
    dist = float(np.linalg.norm(extent)) * 1.5
    eye = center + np.array([dist*0.55, -dist*0.75, dist*0.6])
    render.setup_camera(55.0, center.tolist(), eye.tolist(), [0, 0, 1])

    img = render.render_to_image()
    o3d.io.write_image(out_path, img)
    print(f"  → saved {out_path}")

# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------

def scene_plane():
    print("\n─── Scene 1: plane reconstruction ───")
    pcd, n = _pcd(_make_plane(normal=(0,0,1), size=100, n=3000, seed=10))
    results = detect_planes(pcd, distance_threshold=0.3, min_inliers=100,
                            num_iterations=500, max_planes=2)
    print(f"  {len(results)} plane(s) detected")
    meshes = [_plane_mesh(r, _P_PAL[i % 2]) for i, r in enumerate(results)]
    _render(pcd, meshes, "recon1_plane.png")


def scene_two_planes():
    print("\n─── Scene 2: two planes reconstruction ───")
    pcd, n = _pcd(
        _make_plane(normal=(0,0,1), offset=0,  size=80, n=2000, seed=20),
        _make_plane(normal=(0,1,0), offset=50, size=80, n=2000, seed=21),
    )
    results = detect_planes(pcd, distance_threshold=0.4, min_inliers=100,
                            num_iterations=1000, max_planes=3)
    print(f"  {len(results)} plane(s) detected")
    meshes = [_plane_mesh(r, _P_PAL[i % 2]) for i, r in enumerate(results)]
    _render(pcd, meshes, "recon2_two_planes.png")


def scene_cylinders():
    print("\n─── Scene 3: cylinder reconstruction ───")
    pcd, n = _pcd(
        _make_cylinder((-20,0,0), (0,0,1), 6.0, 30, n=1200, seed=30),
        _make_cylinder(( 20,0,0), (0,0,1), 4.0, 20, n=900,  seed=31),
    )
    results = detect_cylinders(pcd, distance_threshold=0.4, min_inliers=100,
                               num_iterations=1000, max_cylinders=3)
    print(f"  {len(results)} cylinder(s) detected")
    for r in results:
        print(f"    r={r.radius:.2f} mm  h={r.height:.2f} mm  inliers={r.inlier_count}")
    meshes = [_cylinder_mesh(r, _C_PAL[i % 2]) for i, r in enumerate(results)]
    _render(pcd, meshes, "recon3_cylinders.png")


def scene_sphere():
    print("\n─── Scene 4: sphere reconstruction ───")
    pcd, n = _pcd(_make_sphere((0,0,0), radius=15.0, n=1500, seed=40))
    results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=100,
                             num_iterations=800, max_spheres=2)
    print(f"  {len(results)} sphere(s) detected")
    for r in results:
        print(f"    centre={r.center.round(2)}  r={r.radius:.2f} mm  inliers={r.inlier_count}")
    meshes = [_sphere_mesh(r, _S_PAL[i % 2]) for i, r in enumerate(results)]
    _render(pcd, meshes, "recon4_sphere.png")


def scene_mixed():
    print("\n─── Scene 5: mixed reconstruction ───")
    rng = np.random.default_rng(50)
    parts = [
        _make_plane(   (0,0,1),        size=100, n=3000, seed=50),
        _make_cylinder((-30,0,0),  (0,0,1), 7, 25, n=800, seed=51),
        _make_cylinder(( 10,20,0), (0,0,1), 4, 18, n=600, seed=52),
        _make_cylinder(( 30,-15,0),(0,1,0), 5, 22, n=700, seed=53),
        _make_sphere(  (0,-30,15), radius=12,     n=800, seed=54),
    ]
    noise_pts = rng.uniform(-60, 60, (400, 3))
    noise_nrm = rng.normal(0, 1, (400, 3))
    noise_nrm /= np.linalg.norm(noise_nrm, axis=1, keepdims=True)

    all_pts = np.vstack([p for p,_ in parts] + [noise_pts])
    all_nrm = np.vstack([n for _,n in parts] + [noise_nrm])
    pcd = o3d.geometry.PointCloud()
    pcd.points  = o3d.utility.Vector3dVector(all_pts)
    pcd.normals = o3d.utility.Vector3dVector(all_nrm)

    # Detect planes first, then remove their inliers before cylinder/sphere search
    planes = detect_planes(pcd, distance_threshold=0.4, min_inliers=100,
                           num_iterations=1000, max_planes=1)

    plane_inliers = set()
    for r in planes:
        plane_inliers.update(r.inlier_indices)
    keep = [i for i in range(len(pcd.points)) if i not in plane_inliers]
    pcd_no_plane = pcd.select_by_index(keep)

    cylinders = detect_cylinders(pcd_no_plane, distance_threshold=0.5,
                                 min_inliers=80, num_iterations=1500,
                                 max_cylinders=4, max_radius=50.0)
    spheres   = detect_spheres(  pcd_no_plane, distance_threshold=0.6,
                                 min_inliers=80, num_iterations=1000,
                                 max_spheres=2, max_radius=50.0)

    print(f"  {len(planes)} plane(s), {len(cylinders)} cylinder(s), {len(spheres)} sphere(s)")
    for r in cylinders:
        print(f"    Cyl: r={r.radius:.1f} mm  h={r.height:.1f} mm  inliers={r.inlier_count}")
    for r in spheres:
        print(f"    Sph: centre={r.center.round(1)}  r={r.radius:.1f} mm  inliers={r.inlier_count}")

    meshes = (
        [_plane_mesh(r,    _P_PAL[i%2]) for i, r in enumerate(planes)]
      + [_cylinder_mesh(r, _C_PAL[i%2]) for i, r in enumerate(cylinders)]
      + [_sphere_mesh(r,   _S_PAL[i%2]) for i, r in enumerate(spheres)]
    )
    _render(pcd, meshes, "recon5_mixed.png")


# ---------------------------------------------------------------------------
SCENES = [
    ("Plane reconstruction",        scene_plane),
    ("Two planes reconstruction",   scene_two_planes),
    ("Cylinders reconstruction",    scene_cylinders),
    ("Sphere reconstruction",       scene_sphere),
    ("Mixed reconstruction",        scene_mixed),
]

def main():
    print("MeshRANSAC — reconstructed primitive visualization")
    print(f"Output: {OUTPUT_DIR}/\n")

    if len(sys.argv) > 1:
        try:
            idx = int(sys.argv[1]) - 1
            name, fn = SCENES[idx]
            print(f"Scene {idx+1}: {name}")
            fn()
            return
        except (ValueError, IndexError):
            print(f"Usage: {sys.argv[0]} [1-{len(SCENES)}]")
            sys.exit(1)

    for i, (name, fn) in enumerate(SCENES, 1):
        print(f"\n[{i}/{len(SCENES)}] {name}")
        fn()

    print(f"\nDone. Images in {OUTPUT_DIR}/")

if __name__ == "__main__":
    main()
