#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Standalone visualization test for the RANSAC engine.

Run directly — no FreeCAD required, only open3d + numpy:
    python tests/visualize_detection.py

Each scene opens an Open3D viewer window.  Close it to advance to the next.

Color legend:
  grey   — unclassified points
  blue   — detected plane inliers
  green  — detected cylinder inliers
  orange — detected sphere inliers
"""

import sys
import os
import numpy as np

# Make sure the repo root is on sys.path when run from any directory
_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo not in sys.path:
    sys.path.insert(0, _repo)

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d is not installed.  Run:  pip install open3d")

from MeshRANSAC.core.ransac_engine import (
    detect_planes, detect_cylinders, detect_spheres,
    PlaneResult, CylinderResult, SphereResult,
)

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

GREY   = [0.55, 0.55, 0.55]
BLUE   = [0.20, 0.50, 1.00]
GREEN  = [0.10, 0.80, 0.20]
ORANGE = [1.00, 0.55, 0.05]
RED    = [0.90, 0.15, 0.15]
YELLOW = [0.95, 0.85, 0.05]

_PLANE_PALETTE    = [BLUE,   [0.40, 0.70, 1.00], [0.10, 0.30, 0.80]]
_CYLINDER_PALETTE = [GREEN,  [0.30, 1.00, 0.50], [0.05, 0.55, 0.20]]
_SPHERE_PALETTE   = [ORANGE, [1.00, 0.75, 0.20], RED]


# ---------------------------------------------------------------------------
# Synthetic scene generators
# ---------------------------------------------------------------------------

def _make_plane(normal=(0,0,1), offset=0, size=80, n=2000, noise=0.08, seed=0):
    rng = np.random.default_rng(seed)
    n_arr = np.asarray(normal, float)
    n_arr /= np.linalg.norm(n_arr)
    u = np.array([1,0,0]) if abs(n_arr[0]) < 0.9 else np.array([0,1,0])
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
    u = np.array([1,0,0]) if abs(ax[0]) < 0.9 else np.array([0,1,0])
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
# Colouring helpers
# ---------------------------------------------------------------------------

def _colorize(total_pts, results_by_type):
    """
    Build an (N,3) colour array.

    results_by_type: list of (list_of_results, palette)
    """
    colors = np.tile(GREY, (total_pts, 1))
    for results, palette in results_by_type:
        for i, r in enumerate(results):
            c = palette[i % len(palette)]
            colors[r.inlier_indices] = c
    return colors


def _show(pcd, colors, title, extra_geometries=()):
    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points  = pcd.points
    pcd_vis.colors  = o3d.utility.Vector3dVector(colors)
    geoms = [pcd_vis] + list(extra_geometries)
    o3d.visualization.draw_geometries(geoms, window_name=title,
                                      width=1100, height=700,
                                      point_show_normal=False)


# ---------------------------------------------------------------------------
# Axis arrow helper
# ---------------------------------------------------------------------------

def _arrow(origin, direction, length=10.0, color=(0.9, 0.9, 0.1)):
    tip = np.asarray(origin) + np.asarray(direction) * length
    pts = [origin, tip.tolist()]
    lines = [[0, 1]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color])
    return ls


# ---------------------------------------------------------------------------
# Scene 1 — single large plane
# ---------------------------------------------------------------------------

def scene_plane():
    print("\n─── Scene 1: single plane ───")
    pcd, n = _combine(
        _make_plane(normal=(0,0,1), size=100, n=3000, seed=10),
    )
    results = detect_planes(pcd, distance_threshold=0.3, min_inliers=100,
                            num_iterations=500, max_planes=3)
    print(f"  Detected {len(results)} plane(s)")
    for i, r in enumerate(results):
        print(f"    Plane {i+1}: normal={r.normal.round(3)}, "
              f"inliers={r.inlier_count}")

    colors = _colorize(n, [(results, _PLANE_PALETTE)])
    _show(pcd, colors, "Scene 1 — Plane detection  (blue = inliers, grey = noise)")


# ---------------------------------------------------------------------------
# Scene 2 — two tilted planes
# ---------------------------------------------------------------------------

def scene_two_planes():
    print("\n─── Scene 2: two tilted planes ───")
    p1 = _make_plane(normal=(0, 0, 1), offset=0,  size=80, n=2000, seed=20)
    p2 = _make_plane(normal=(0, 1, 0), offset=50, size=80, n=2000, seed=21)
    pcd, n = _combine(p1, p2)
    results = detect_planes(pcd, distance_threshold=0.4, min_inliers=100,
                            num_iterations=1000, max_planes=5)
    print(f"  Detected {len(results)} plane(s)")
    for i, r in enumerate(results):
        print(f"    Plane {i+1}: normal={r.normal.round(3)}, "
              f"inliers={r.inlier_count}")

    colors = _colorize(n, [(results, _PLANE_PALETTE)])
    _show(pcd, colors, "Scene 2 — Two planes  (blue shades, grey = noise)")


# ---------------------------------------------------------------------------
# Scene 3 — two cylinders
# ---------------------------------------------------------------------------

def scene_cylinders():
    print("\n─── Scene 3: two cylinders ───")
    c1 = _make_cylinder(centre=(-20, 0, 0), axis=(0,0,1), radius=6.0,
                        height=30, n=1200, seed=30)
    c2 = _make_cylinder(centre=( 20, 0, 0), axis=(0,0,1), radius=4.0,
                        height=20, n=900,  seed=31)
    pcd, n = _combine(c1, c2)
    results = detect_cylinders(pcd, distance_threshold=0.4, min_inliers=100,
                               num_iterations=1000, max_cylinders=5)
    print(f"  Detected {len(results)} cylinder(s)")
    extras = []
    for i, r in enumerate(results):
        print(f"    Cylinder {i+1}: radius={r.radius:.2f} mm, "
              f"height={r.height:.2f} mm, inliers={r.inlier_count}")
        extras.append(_arrow(r.center.tolist(), r.axis_dir.tolist(),
                             length=r.height * 0.6, color=YELLOW))

    colors = _colorize(n, [(results, _CYLINDER_PALETTE)])
    _show(pcd, colors, "Scene 3 — Cylinders  (green shades + yellow axis arrows)",
          extra_geometries=extras)


# ---------------------------------------------------------------------------
# Scene 4 — sphere
# ---------------------------------------------------------------------------

def scene_sphere():
    print("\n─── Scene 4: sphere ───")
    pcd, n = _combine(
        _make_sphere(centre=(0, 0, 0), radius=15.0, n=1500, seed=40),
    )
    results = detect_spheres(pcd, distance_threshold=0.5, min_inliers=100,
                             num_iterations=800, max_spheres=3)
    print(f"  Detected {len(results)} sphere(s)")
    for i, r in enumerate(results):
        print(f"    Sphere {i+1}: centre={r.center.round(2)}, "
              f"radius={r.radius:.2f} mm, inliers={r.inlier_count}")

    colors = _colorize(n, [(results, _SPHERE_PALETTE)])
    _show(pcd, colors, "Scene 4 — Sphere  (orange = inliers, grey = noise)")


# ---------------------------------------------------------------------------
# Scene 5 — mixed: plane + 3 cylinders + sphere
# ---------------------------------------------------------------------------

def scene_mixed():
    print("\n─── Scene 5: mixed scene (plane + 3 cylinders + sphere) ───")
    rng = np.random.default_rng(50)

    plane_pts,  plane_nrm  = _make_plane(normal=(0,0,1), size=100, n=3000, seed=50)
    cyl1_pts,   cyl1_nrm   = _make_cylinder((-30, 0, 0), (0,0,1), 7, 25, n=800, seed=51)
    cyl2_pts,   cyl2_nrm   = _make_cylinder(( 10, 20, 0), (0,0,1), 4, 18, n=600, seed=52)
    cyl3_pts,   cyl3_nrm   = _make_cylinder(( 30,-15, 0), (0,1,0), 5, 22, n=700, seed=53)
    sph_pts,    sph_nrm    = _make_sphere((0, -30, 15), radius=12, n=800, seed=54)

    # Small amount of random noise
    noise_pts = rng.uniform(-60, 60, (500, 3))
    noise_nrm = rng.normal(0, 1, (500, 3))
    noise_nrm /= np.linalg.norm(noise_nrm, axis=1, keepdims=True)

    all_pts = np.vstack([plane_pts, cyl1_pts, cyl2_pts, cyl3_pts, sph_pts, noise_pts])
    all_nrm = np.vstack([plane_nrm, cyl1_nrm, cyl2_nrm, cyl3_nrm, sph_nrm, noise_nrm])
    n = len(all_pts)

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
        print(f"    Plane {i+1}: normal={r.normal.round(3)}, inliers={r.inlier_count}")
    for i, r in enumerate(cylinders):
        print(f"    Cylinder {i+1}: r={r.radius:.2f} mm, h={r.height:.2f} mm, "
              f"inliers={r.inlier_count}")
    for i, r in enumerate(spheres):
        print(f"    Sphere {i+1}: centre={r.center.round(2)}, "
              f"r={r.radius:.2f} mm, inliers={r.inlier_count}")

    colors = _colorize(n, [
        (planes,    _PLANE_PALETTE),
        (cylinders, _CYLINDER_PALETTE),
        (spheres,   _SPHERE_PALETTE),
    ])

    extras = [_arrow(r.center.tolist(), r.axis_dir.tolist(),
                     length=r.height * 0.5, color=YELLOW)
              for r in cylinders]

    _show(pcd, colors,
          "Scene 5 — Mixed  (blue=planes  green=cylinders  orange=spheres  grey=noise)",
          extra_geometries=extras)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

SCENES = [
    ("Single plane",          scene_plane),
    ("Two tilted planes",     scene_two_planes),
    ("Two cylinders",         scene_cylinders),
    ("Sphere",                scene_sphere),
    ("Mixed scene",           scene_mixed),
]


def main():
    print("MeshRANSAC — visualization test")
    print("Close each viewer window to advance to the next scene.")
    print(f"Scenes: {', '.join(n for n,_ in SCENES)}\n")

    if len(sys.argv) > 1:
        # Allow running a single scene by number: python visualize_detection.py 5
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

    print("\nAll scenes done.")


if __name__ == "__main__":
    main()
