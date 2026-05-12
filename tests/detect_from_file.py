#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Run RANSAC detection on any point cloud or mesh file and visualize results.

Supported input formats (via Open3D):
  Point clouds : .ply  .pcd  .xyz  .xyzn  .xyzrgb  .pts
  Meshes       : .ply  .stl  .obj  .off    (sampled to point cloud automatically)

Usage:
    .venv/bin/python tests/detect_from_file.py <file> [options]

Options:
    --points   N      Points to sample from mesh surface  (default 50000)
    --voxel    F      Voxel down-sample size in mm        (default 0 = off)
    --dist     F      RANSAC distance threshold mm         (default 0.5)
    --inliers  N      Minimum inliers to accept primitive  (default 50)
    --iters    N      RANSAC iterations per shape          (default 1000)
    --maxr     N      Max shapes per type                  (default 10)
    --maxrad   F      Max primitive radius mm              (default 500)
    --no-planes       Skip plane detection
    --no-cylinders    Skip cylinder detection
    --no-spheres      Skip sphere detection
    --seq             Sequential mode: remove plane inliers before cyl/sph
    --save     PATH   Save rendered PNG to this path

Examples:
    # Point cloud
    .venv/bin/python tests/detect_from_file.py scan.ply --dist 0.3 --seq

    # Mesh (STL)
    .venv/bin/python tests/detect_from_file.py part.stl --points 80000 --seq

    # Mesh, save image
    .venv/bin/python tests/detect_from_file.py part.obj --save out.png
"""

import sys
import os
import argparse
import numpy as np

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo not in sys.path:
    sys.path.insert(0, _repo)

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
GREY   = [0.45, 0.45, 0.45]
BLUE   = [0.25, 0.55, 1.00]
BLUE2  = [0.45, 0.70, 1.00]
GREEN  = [0.10, 0.80, 0.25]
GREEN2 = [0.30, 1.00, 0.50]
ORANGE = [1.00, 0.55, 0.05]
ORANGE2= [1.00, 0.75, 0.20]

_P_PAL = [BLUE,   BLUE2]
_C_PAL = [GREEN,  GREEN2]
_S_PAL = [ORANGE, ORANGE2]

# ---------------------------------------------------------------------------
# Load: auto-detect point cloud vs mesh
# ---------------------------------------------------------------------------

_MESH_EXTS = {".stl", ".obj", ".off"}
_PCD_EXTS  = {".pcd", ".xyz", ".xyzn", ".xyzrgb", ".pts"}

def load_as_pointcloud(path, n_sample=50000, voxel_size=0.0):
    """
    Load a point cloud or mesh file and return an Open3D PointCloud with normals.

    For meshes: uniformly samples n_sample surface points and estimates normals.
    For point clouds: loads directly; estimates normals if missing.
    """
    ext = os.path.splitext(path)[1].lower()

    if ext in _MESH_EXTS or (ext == ".ply" and _is_mesh_ply(path)):
        print(f"  Loading as mesh → sampling {n_sample} surface points …")
        mesh = o3d.io.read_triangle_mesh(path)
        if len(mesh.triangles) == 0:
            raise ValueError(f"No triangles found in {path}. "
                             "Is it really a mesh file?")
        mesh.compute_vertex_normals()
        pcd = mesh.sample_points_poisson_disk(n_sample)
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=_estimate_radius(pcd), max_nn=30)
            )
    else:
        print(f"  Loading as point cloud …")
        pcd = o3d.io.read_point_cloud(path)
        if len(pcd.points) == 0:
            raise ValueError(f"No points loaded from {path}.")
        if not pcd.has_normals():
            print("  No normals found — estimating …")
            r = _estimate_radius(pcd)
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=r, max_nn=30)
            )
            pcd.orient_normals_consistent_tangent_plane(15)

    print(f"  {len(pcd.points)} points loaded")

    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"  → {len(pcd.points)} points after voxel down-sample ({voxel_size} mm)")

    return pcd


def _is_mesh_ply(path):
    """Peek at the PLY header to check if it has face elements."""
    try:
        with open(path, "rb") as f:
            header = f.read(2048).decode("latin-1")
        return "element face" in header
    except Exception:
        return False


def _estimate_radius(pcd):
    """Heuristic normal estimation radius: ~1% of bounding box diagonal."""
    bb = pcd.get_axis_aligned_bounding_box()
    diag = float(np.linalg.norm(np.asarray(bb.get_extent())))
    return max(diag * 0.01, 0.1)

# ---------------------------------------------------------------------------
# Mesh builders for reconstructed primitives
# ---------------------------------------------------------------------------

def _rot_z_to(target):
    z = np.array([0., 0., 1.])
    t = np.asarray(target, float); t /= np.linalg.norm(t)
    v = np.cross(z, t); c = float(np.dot(z, t))
    if abs(c + 1.0) < 1e-6:
        return np.diag([1., -1., -1.])
    s = np.linalg.norm(v)
    if s < 1e-6:
        return np.eye(3)
    K = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (s * s))


def _plane_mesh(r: PlaneResult, color):
    pts = r.bounding_box
    edges = sorted([np.linalg.norm(pts[1]-pts[0]),
                    np.linalg.norm(pts[3]-pts[0]),
                    np.linalg.norm(pts[4]-pts[0])], reverse=True)
    w, l = max(edges[0], 1.0), max(edges[1], 1.0)
    th = max(np.linalg.norm(np.asarray(
        o3d.geometry.AxisAlignedBoundingBox(
            pts.min(0), pts.max(0)).get_extent())) * 0.01, 0.5)
    m = o3d.geometry.TriangleMesh.create_box(w, l, th)
    m.compute_vertex_normals()
    m.translate([-w/2, -l/2, -th/2])
    m.rotate(_rot_z_to(r.normal), center=(0,0,0))
    m.translate(r.center); m.paint_uniform_color(color)
    return m


def _cylinder_mesh(r: CylinderResult, color):
    m = o3d.geometry.TriangleMesh.create_cylinder(
        radius=max(r.radius, 0.5), height=max(r.height, 1.0), resolution=40)
    m.compute_vertex_normals()
    m.rotate(_rot_z_to(r.axis_dir), center=(0,0,0))
    m.translate(r.center); m.paint_uniform_color(color)
    return m


def _sphere_mesh(r: SphereResult, color):
    m = o3d.geometry.TriangleMesh.create_sphere(
        radius=max(r.radius, 0.5), resolution=30)
    m.compute_vertex_normals()
    m.translate(r.center); m.paint_uniform_color(color)
    return m

# ---------------------------------------------------------------------------
# Render
# ---------------------------------------------------------------------------

def render(pcd, meshes, save_path=None):
    pcd_grey = o3d.geometry.PointCloud()
    pcd_grey.points = pcd.points
    n = len(pcd.points)
    pcd_grey.colors = o3d.utility.Vector3dVector(np.tile(GREY, (n, 1)))

    render = o3d.visualization.rendering.OffscreenRenderer(1600, 1000)

    pt_mat = o3d.visualization.rendering.MaterialRecord()
    pt_mat.shader = "defaultUnlit"; pt_mat.point_size = 4.0
    render.scene.add_geometry("pcd", pcd_grey, pt_mat)

    mesh_mat = o3d.visualization.rendering.MaterialRecord()
    mesh_mat.shader = "defaultLit"
    mesh_mat.base_roughness = 0.4; mesh_mat.base_reflectance = 0.1
    for i, m in enumerate(meshes):
        render.scene.add_geometry(f"m{i}", m, mesh_mat)

    render.scene.set_background([0.10, 0.10, 0.10, 1.0])
    render.scene.scene.set_sun_light([0.6, -0.8, -0.5], [1,1,1], 80000)
    render.scene.scene.enable_sun_light(True)

    bb = pcd_grey.get_axis_aligned_bounding_box()
    ctr = np.asarray(bb.get_center())
    dist = float(np.linalg.norm(np.asarray(bb.get_extent()))) * 0.9
    eye = ctr + np.array([dist*0.55, -dist*0.75, dist*0.6])
    render.setup_camera(60.0, ctr.tolist(), eye.tolist(), [0, 0, 1])

    img = render.render_to_image()

    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        o3d.io.write_image(save_path, img)
        print(f"\n  Image saved → {save_path}")
    else:
        default = os.path.join(os.path.dirname(__file__), "output", "detected.png")
        os.makedirs(os.path.dirname(default), exist_ok=True)
        o3d.io.write_image(default, img)
        print(f"\n  Image saved → {default}")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _remove_inliers(pcd, results):
    """Return pcd with all inlier indices from results removed."""
    inlier_set = set()
    for r in results:
        inlier_set.update(r.inlier_indices)
    keep = [i for i in range(len(pcd.points)) if i not in inlier_set]
    return pcd.select_by_index(keep)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="RANSAC detection on a point cloud or mesh file")
    parser.add_argument("file", help="Input file (.ply .pcd .stl .obj …)")
    parser.add_argument("--points",        type=int,   default=50000)
    parser.add_argument("--voxel",         type=float, default=0.0)
    parser.add_argument("--dist",          type=float, default=0.5)
    parser.add_argument("--inliers",       type=int,   default=50)
    parser.add_argument("--iters",         type=int,   default=1000)
    parser.add_argument("--maxr",          type=int,   default=10)
    parser.add_argument("--maxrad",        type=float, default=500.0)
    parser.add_argument("--no-planes",     action="store_true")
    parser.add_argument("--no-cylinders",  action="store_true")
    parser.add_argument("--no-spheres",    action="store_true")
    parser.add_argument("--seq",           action="store_true",
                        help="Sequential: remove plane inliers before cyl/sph")
    parser.add_argument("--save",          default=None)
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        sys.exit(f"File not found: {args.file}")

    print(f"\nMeshRANSAC — detecting in: {args.file}")
    pcd = load_as_pointcloud(args.file, n_sample=args.points, voxel_size=args.voxel)

    kw = dict(distance_threshold=args.dist, min_inliers=args.inliers,
              num_iterations=args.iters)

    planes, cylinders, spheres = [], [], []
    working_pcd = pcd

    if not args.no_planes:
        print("\n  Detecting planes …")
        planes = detect_planes(working_pcd, max_planes=args.maxr, **kw)
        print(f"    → {len(planes)} plane(s)")
        for i, r in enumerate(planes):
            print(f"       Plane {i+1}: normal={r.normal.round(3)}, "
                  f"inliers={r.inlier_count}")
        if args.seq and planes:
            working_pcd = _remove_inliers(working_pcd, planes)
            print(f"    (→ {len(working_pcd.points)} pts remaining)")

    if not args.no_cylinders:
        print("\n  Detecting cylinders …")
        cylinders = detect_cylinders(working_pcd, max_cylinders=args.maxr,
                                     max_radius=args.maxrad, **kw)
        print(f"    → {len(cylinders)} cylinder(s)")
        for i, r in enumerate(cylinders):
            print(f"       Cyl {i+1}: r={r.radius:.2f} mm, "
                  f"h={r.height:.2f} mm, inliers={r.inlier_count}")
        if args.seq and cylinders:
            working_pcd = _remove_inliers(working_pcd, cylinders)
            print(f"    (→ {len(working_pcd.points)} pts remaining)")

    if not args.no_spheres:
        print("\n  Detecting spheres …")
        spheres = detect_spheres(working_pcd, max_spheres=args.maxr,
                                 max_radius=args.maxrad, **kw)
        print(f"    → {len(spheres)} sphere(s)")
        for i, r in enumerate(spheres):
            print(f"       Sph {i+1}: centre={r.center.round(2)}, "
                  f"r={r.radius:.2f} mm, inliers={r.inlier_count}")

    meshes = (
        [_plane_mesh(r,    _P_PAL[i%2]) for i, r in enumerate(planes)]
      + [_cylinder_mesh(r, _C_PAL[i%2]) for i, r in enumerate(cylinders)]
      + [_sphere_mesh(r,   _S_PAL[i%2]) for i, r in enumerate(spheres)]
    )

    total = len(planes) + len(cylinders) + len(spheres)
    print(f"\n  Total primitives detected: {total}")
    print("  Rendering …")
    render(pcd, meshes, save_path=args.save)


if __name__ == "__main__":
    main()
