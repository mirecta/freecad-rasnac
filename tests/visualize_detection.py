#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Standalone visualization test for the RANSAC engine.

Saves one PNG per scene into tests/output/.
No display or GUI required — uses Open3D offscreen rendering.

Run with uv:
    .venv/bin/python tests/visualize_detection.py

Or a single scene (1-5):
    .venv/bin/python tests/visualize_detection.py 5

Color legend:
  grey   — unclassified points
  blue   — detected plane inliers
  green  — detected cylinder inliers
  orange — detected sphere inliers
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

from MeshRANSAC.core.ransac_engine import detect_planes, detect_cylinders, detect_spheres

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

GREY   = [0.55, 0.55, 0.55]
BLUE   = [0.20, 0.50, 1.00]
BLUE2  = [0.40, 0.70, 1.00]
BLUE3  = [0.10, 0.30, 0.80]
GREEN  = [0.10, 0.80, 0.20]
GREEN2 = [0.30, 1.00, 0.50]
GREEN3 = [0.05, 0.55, 0.20]
ORANGE = [1.00, 0.55, 0.05]
ORANGE2= [1.00, 0.75, 0.20]
RED    = [0.90, 0.15, 0.15]
YELLOW = [0.95, 0.85, 0.05]
WHITE  = [1.00, 1.00, 1.00]

_PLANE_PALETTE    = [BLUE,   BLUE2,  BLUE3]
_CYLINDER_PALETTE = [GREEN,  GREEN2, GREEN3]
_SPHERE_PALETTE   = [ORANGE, ORANGE2, RED]

# ---------------------------------------------------------------------------
# Synthetic scene generators
# ---------------------------------------------------------------------------

def _make_plane(normal=(0,0,1), offset=0, size=80, n=2000, noise=0.08, seed=0):
    rng = np.random.default_rng(seed)
    n_arr = np.asarray(normal, float)
    n_arr /= np.linalg.norm(n_arr)
    u = np.array([1.,0.,0.]) if abs(n_arr[0]) < 0.9 else np.array([0.,1.,0.])
    u -= np.dot(u, n_arr) * n_arr;  u /= np.linalg.norm(u)
    v = np.cross(n_arr, u)
    s = rng.uniform(-size/2, size/2, (n, 2))
    pts = s[:,0:1]*u + s[:,1:2]*v - offset*n_arr + rng.normal(0, noise, (n,3))
    nrm = np.tile(n_arr, (n,1)) + rng.normal(0, 0.01, (n,3))
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return pts, nrm


def _make_cylinder(centre=(0,0,0), axis=(0,0,1), radius=5.0,
                   height=30.0, n=1000, noise=0.06, seed=1):
    rng = np.random.default_rng(seed)
    ax = np.asarray(axis, float);  ax /= np.linalg.norm(ax)
    u = np.array([1.,0.,0.]) if abs(ax[0]) < 0.9 else np.array([0.,1.,0.])
    u -= np.dot(u, ax)*ax;  u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    theta = rng.uniform(0, 2*np.pi, n)
    h     = rng.uniform(0, height, n)
    pts = (np.outer(np.cos(theta), u*radius)
           + np.outer(np.sin(theta), v*radius)
           + np.outer(h, ax)
           + np.asarray(centre)
           + rng.normal(0, noise, (n,3)))
    rad_dir = np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v)
    rad_dir += rng.normal(0, 0.01, (n,3))
    rad_dir /= np.linalg.norm(rad_dir, axis=1, keepdims=True)
    return pts, rad_dir


def _make_sphere(centre=(0,0,0), radius=12.0, n=900, noise=0.07, seed=2):
    rng = np.random.default_rng(seed)
    phi   = rng.uniform(0, np.pi, n)
    theta = rng.uniform(0, 2*np.pi, n)
    pts = np.column_stack([
        radius*np.sin(phi)*np.cos(theta),
        radius*np.sin(phi)*np.sin(theta),
        radius*np.cos(phi),
    ]) + np.asarray(centre) + rng.normal(0, noise, (n,3))
    nrm = pts - np.asarray(centre)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return pts, nrm


def _combine(*parts):
    pts = np.vstack([p for p,_ in parts])
    nrm = np.vstack([n for _,n in parts])
    pcd = o3d.geometry.PointCloud()
    pcd.points  = o3d.utility.Vector3dVector(pts)
    pcd.normals = o3d.utility.Vector3dVector(nrm)
    return pcd, len(pts)

# ---------------------------------------------------------------------------
# Colouring
# ---------------------------------------------------------------------------

def _colorize(total_pts, results_by_type):
    colors = np.tile(GREY, (total_pts, 1))
    for results, palette in results_by_type:
        for i, r in enumerate(results):
            c = palette[i % len(palette)]
            colors[r.inlier_indices] = c
    return colors

# ---------------------------------------------------------------------------
# Axis line helper
# ---------------------------------------------------------------------------

def _axis_line(origin, direction, length=10.0, color=YELLOW):
    tip = np.asarray(origin) + np.asarray(direction) * length
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector([origin, tip.tolist()])
    ls.lines  = o3d.utility.Vector2iVector([[0, 1]])
    ls.colors = o3d.utility.Vector3dVector([color])
    return ls

# ---------------------------------------------------------------------------
# Offscreen render → PNG
# ---------------------------------------------------------------------------

def _render(pcd, colors, filename, extra_geometries=()):
    """Render the point cloud offscreen and save to tests/output/<filename>."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, filename)

    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points = pcd.points
    pcd_vis.colors = o3d.utility.Vector3dVector(colors)

    geoms = [pcd_vis] + list(extra_geometries)

    # Fit all geometry into view
    bbox   = pcd_vis.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    extent = np.asarray(bbox.get_extent())
    cam_dist = float(np.linalg.norm(extent)) * 1.4

    render = o3d.visualization.rendering.OffscreenRenderer(1280, 800)
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.point_size = 3.0

    for i, g in enumerate(geoms):
        if isinstance(g, o3d.geometry.PointCloud):
            render.scene.add_geometry(f"pcd_{i}", g, mat)
        else:
            lmat = o3d.visualization.rendering.MaterialRecord()
            lmat.shader = "unlitLine"
            lmat.line_width = 3.0
            render.scene.add_geometry(f"line_{i}", g, lmat)

    render.scene.set_background([0.12, 0.12, 0.12, 1.0])

    # Isometric-ish camera
    eye    = center + np.array([cam_dist*0.6, -cam_dist*0.8, cam_dist*0.6])
    render.setup_camera(60.0, center, eye.tolist(), [0, 0, 1])

    img = render.render_to_image()
    o3d.io.write_image(out_path, img)
    print(f"  → saved {out_path}")
    return out_path

# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------

def scene_plane():
    print("\n─── Scene 1: single plane ───")
    pcd, n = _combine(_make_plane(normal=(0,0,1), size=100, n=3000, seed=10))
    results = detect_planes(pcd, distance_threshold=0.3, min_inliers=100,
                            num_iterations=500, max_planes=3)
    print(f"  Detected {len(results)} plane(s)")
    for i, r in enumerate(results):
        print(f"    Plane {i+1}: normal={r.normal.round(3)}, inliers={r.inlier_count}")
    colors = _colorize(n, [(results, _PLANE_PALETTE)])
    _render(pcd, colors, "scene1_plane.png")


def scene_two_planes():
    print("\n─── Scene 2: two tilted planes ───")
    pcd, n = _combine(
        _make_plane(normal=(0,0,1), offset=0,  size=80, n=2000, seed=20),
        _make_plane(normal=(0,1,0), offset=50, size=80, n=2000, seed=21),
    )
    results = detect_planes(pcd, distance_threshold=0.4, min_inliers=100,
                            num_iterations=1000, max_planes=5)
    print(f"  Detected {len(results)} plane(s)")
    for i, r in enumerate(results):
        print(f"    Plane {i+1}: normal={r.normal.round(3)}, inliers={r.inlier_count}")
    colors = _colorize(n, [(results, _PLANE_PALETTE)])
    _render(pcd, colors, "scene2_two_planes.png")


def scene_cylinders():
    print("\n─── Scene 3: two cylinders ───")
    pcd, n = _combine(
        _make_cylinder(centre=(-20,0,0), axis=(0,0,1), radius=6.0, height=30, n=1200, seed=30),
        _make_cylinder(centre=( 20,0,0), axis=(0,0,1), radius=4.0, height=20, n=900,  seed=31),
    )
    results = detect_cylinders(pcd, distance_threshold=0.4, min_inliers=100,
                               num_iterations=1000, max_cylinders=5)
    print(f"  Detected {len(results)} cylinder(s)")
    extras = []
    for i, r in enumerate(results):
        print(f"    Cylinder {i+1}: radius={r.radius:.2f} mm, "
              f"height={r.height:.2f} mm, inliers={r.inlier_count}")
        extras.append(_axis_line(r.center.tolist(), r.axis_dir.tolist(),
                                 length=r.height*0.6))
    colors = _colorize(n, [(results, _CYLINDER_PALETTE)])
    _render(pcd, colors, "scene3_cylinders.png", extra_geometries=extras)


def scene_sphere():
    print("\n─── Scene 4: sphere ───")
    pcd, n = _combine(_make_sphere(centre=(0,0,0), radius=15.0, n=1500, seed=40))
    results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=100,
                             num_iterations=800, max_spheres=3)
    print(f"  Detected {len(results)} sphere(s)")
    for i, r in enumerate(results):
        print(f"    Sphere {i+1}: centre={r.center.round(2)}, "
              f"radius={r.radius:.2f} mm, inliers={r.inlier_count}")
    colors = _colorize(n, [(results, _SPHERE_PALETTE)])
    _render(pcd, colors, "scene4_sphere.png")


def scene_mixed():
    print("\n─── Scene 5: mixed scene ───")
    rng = np.random.default_rng(50)
    parts = [
        _make_plane(   normal=(0,0,1),      size=100, n=3000, seed=50),
        _make_cylinder((-30, 0,  0), (0,0,1), 7, 25, n=800,  seed=51),
        _make_cylinder(( 10, 20, 0), (0,0,1), 4, 18, n=600,  seed=52),
        _make_cylinder(( 30,-15, 0), (0,1,0), 5, 22, n=700,  seed=53),
        _make_sphere(  (0, -30, 15), radius=12,       n=800,  seed=54),
    ]
    noise_pts = rng.uniform(-60, 60, (500, 3))
    noise_nrm = rng.normal(0, 1, (500, 3))
    noise_nrm /= np.linalg.norm(noise_nrm, axis=1, keepdims=True)

    all_pts = np.vstack([p for p,_ in parts] + [noise_pts])
    all_nrm = np.vstack([n for _,n in parts] + [noise_nrm])
    n_total = len(all_pts)

    pcd = o3d.geometry.PointCloud()
    pcd.points  = o3d.utility.Vector3dVector(all_pts)
    pcd.normals = o3d.utility.Vector3dVector(all_nrm)

    print("  Running plane detection …")
    planes = detect_planes(pcd, distance_threshold=0.4, min_inliers=100,
                           num_iterations=1000, max_planes=5)
    print(f"    → {len(planes)} plane(s)")

    print("  Running cylinder detection …")
    cylinders = detect_cylinders(pcd, distance_threshold=0.5, min_inliers=80,
                                 num_iterations=1500, max_cylinders=5)
    print(f"    → {len(cylinders)} cylinder(s)")

    print("  Running sphere detection …")
    spheres = detect_spheres(pcd, distance_threshold=0.6, min_inliers=80,
                             num_iterations=1000, max_spheres=3)
    print(f"    → {len(spheres)} sphere(s)")

    for i, r in enumerate(planes):
        print(f"    Plane {i+1}:    normal={r.normal.round(3)}, inliers={r.inlier_count}")
    for i, r in enumerate(cylinders):
        print(f"    Cylinder {i+1}: r={r.radius:.2f} mm, h={r.height:.2f} mm, "
              f"inliers={r.inlier_count}")
    for i, r in enumerate(spheres):
        print(f"    Sphere {i+1}:   centre={r.center.round(2)}, "
              f"r={r.radius:.2f} mm, inliers={r.inlier_count}")

    colors = _colorize(n_total, [
        (planes,    _PLANE_PALETTE),
        (cylinders, _CYLINDER_PALETTE),
        (spheres,   _SPHERE_PALETTE),
    ])
    extras = [_axis_line(r.center.tolist(), r.axis_dir.tolist(), length=r.height*0.5)
              for r in cylinders]
    _render(pcd, colors, "scene5_mixed.png", extra_geometries=extras)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

SCENES = [
    ("Single plane",      scene_plane),
    ("Two tilted planes", scene_two_planes),
    ("Two cylinders",     scene_cylinders),
    ("Sphere",            scene_sphere),
    ("Mixed scene",       scene_mixed),
]


def main():
    print("MeshRANSAC — visualization test  (offscreen render → tests/output/)")
    print(f"Scenes: {', '.join(n for n,_ in SCENES)}\n")

    if len(sys.argv) > 1:
        try:
            idx = int(sys.argv[1]) - 1
            name, fn = SCENES[idx]
            print(f"Running scene {idx+1}: {name}")
            fn()
            return
        except (ValueError, IndexError):
            print(f"Usage: {sys.argv[0]} [1-{len(SCENES)}]")
            sys.exit(1)

    for i, (name, fn) in enumerate(SCENES, 1):
        print(f"\n[{i}/{len(SCENES)}] {name}")
        fn()

    print(f"\nAll done. Images saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
