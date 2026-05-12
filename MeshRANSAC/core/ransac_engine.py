# SPDX-License-Identifier: LGPL-2.1-or-later
"""RANSAC primitive detection on Open3D point clouds."""

from dataclasses import dataclass, field
from typing import List, Optional, Callable
import numpy as np

from MeshRANSAC.utils.dependency_checker import require_open3d


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------

@dataclass
class PlaneResult:
    normal: np.ndarray          # unit normal [a, b, c]
    offset: float               # d in ax+by+cz+d=0
    center: np.ndarray          # centroid of inlier points
    inlier_count: int
    inlier_indices: List[int]
    bounding_box: np.ndarray    # (8,3) corners of oriented bounding box


@dataclass
class CylinderResult:
    axis_point: np.ndarray      # a point on the cylinder axis
    axis_dir: np.ndarray        # unit direction vector of axis
    radius: float               # mm
    height: float               # estimated from inlier extent along axis
    center: np.ndarray          # midpoint of axis segment
    inlier_count: int
    inlier_indices: List[int]


@dataclass
class SphereResult:
    center: np.ndarray
    radius: float
    inlier_count: int
    inlier_indices: List[int]


# ---------------------------------------------------------------------------
# Plane detection
# ---------------------------------------------------------------------------

def detect_planes(pcd, distance_threshold=0.5, ransac_n=3,
                  num_iterations=1000, min_inliers=50, max_planes=10,
                  progress_cb: Optional[Callable] = None):
    """
    Iteratively extract planes from a point cloud using Open3D's segment_plane.

    After each detection the inlier points are removed so subsequent
    iterations find the next-largest plane.

    Args:
        pcd:                open3d.geometry.PointCloud
        distance_threshold: max point-to-plane distance to count as inlier (mm)
        ransac_n:           points sampled per RANSAC hypothesis
        num_iterations:     RANSAC iterations per plane
        min_inliers:        minimum inlier count to accept a detection
        max_planes:         maximum number of planes to extract
        progress_cb:        optional callable(step, total) for progress reporting

    Returns:
        list of PlaneResult
    """
    o3d = require_open3d()
    results = []
    remaining = pcd

    for step in range(max_planes):
        if len(remaining.points) < min_inliers:
            break

        model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )

        if len(inliers) < min_inliers:
            break

        inlier_cloud = remaining.select_by_index(inliers)
        pts = np.asarray(inlier_cloud.points)
        center = pts.mean(axis=0)
        obb = inlier_cloud.get_oriented_bounding_box()

        results.append(PlaneResult(
            normal=np.array(model[:3]),
            offset=float(model[3]),
            center=center,
            inlier_count=len(inliers),
            inlier_indices=list(inliers),
            bounding_box=np.asarray(obb.get_box_points()),
        ))

        remaining = remaining.select_by_index(inliers, invert=True)

        if progress_cb is not None:
            progress_cb(step + 1, max_planes)

    return results


# ---------------------------------------------------------------------------
# Cylinder detection (custom RANSAC — Open3D stable API has no built-in)
# ---------------------------------------------------------------------------

def detect_cylinders(pcd, distance_threshold=0.5, min_inliers=50,
                     max_cylinders=10, num_iterations=1000,
                     max_radius=500.0,
                     progress_cb: Optional[Callable] = None):
    """
    Detect cylinders using a custom RANSAC.

    Hypothesis generation: sample 2 points + their normals.
    The cross product of the two normals gives the axis direction.
    Project both points onto the plane perpendicular to that axis
    and fit a circle to estimate the radius and axis position.

    Args:
        pcd:                open3d.geometry.PointCloud (must have normals)
        distance_threshold: max radial distance from cylinder surface (mm)
        min_inliers:        minimum inlier count to accept a detection
        max_cylinders:      maximum cylinders to extract
        num_iterations:     RANSAC iterations per cylinder
        progress_cb:        optional callable(step, total)

    Returns:
        list of CylinderResult
    """
    require_open3d()
    results = []
    remaining = pcd

    for step in range(max_cylinders):
        pts = np.asarray(remaining.points)
        nrm = np.asarray(remaining.normals)
        n_pts = len(pts)

        if n_pts < min_inliers:
            break

        best = None
        best_inliers = []

        rng = np.random.default_rng()

        for _ in range(num_iterations):
            # Sample 2 distinct points
            i, j = rng.choice(n_pts, size=2, replace=False)
            ni, nj = nrm[i], nrm[j]

            axis = np.cross(ni, nj)
            axis_len = np.linalg.norm(axis)
            if axis_len < 1e-6:
                continue  # normals nearly parallel — degenerate
            axis /= axis_len

            # Project points onto the plane perpendicular to axis
            # Circle fit on the 2D projections of the two sample points
            pi = pts[i] - np.dot(pts[i], axis) * axis
            pj = pts[j] - np.dot(pts[j], axis) * axis
            ni2 = ni - np.dot(ni, axis) * axis
            nj2 = nj - np.dot(nj, axis) * axis
            n2_len = np.linalg.norm(ni2)
            if n2_len < 1e-6:
                continue
            ni2 /= n2_len

            # The centre lies along the normal from each sample point in 2D.
            # Solve: pi + t*ni2 == pj + s*nj2  → least-squares intersection
            A = np.column_stack([ni2, -nj2])
            b = pj - pi
            if np.linalg.matrix_rank(A) < 2:
                continue
            ts, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            t = ts[0]
            centre3d = pi + t * ni2

            radius = np.linalg.norm(pts[i] - centre3d -
                                    np.dot(pts[i] - centre3d, axis) * axis)
            if radius < 1e-3 or radius > max_radius:
                continue

            # Score: count inliers
            # Radial distance = ||(p - centre3d) - ((p-centre3d)·axis)*axis|| - r
            diff = pts - centre3d
            proj_axis = diff @ axis
            radial = diff - np.outer(proj_axis, axis)
            radial_dist = np.abs(np.linalg.norm(radial, axis=1) - radius)
            inlier_mask = radial_dist < distance_threshold
            n_inliers = inlier_mask.sum()

            if n_inliers > len(best_inliers):
                best_inliers = np.where(inlier_mask)[0].tolist()
                best = (centre3d, axis, radius)

        if best is None or len(best_inliers) < min_inliers:
            break

        centre3d, axis, radius = best
        inlier_pts = pts[best_inliers]

        # Estimate height from extent along axis
        proj = inlier_pts @ axis
        height = float(proj.max() - proj.min())
        axis_mid = centre3d + ((proj.max() + proj.min()) / 2) * axis

        results.append(CylinderResult(
            axis_point=centre3d,
            axis_dir=axis,
            radius=float(radius),
            height=height,
            center=axis_mid,
            inlier_count=len(best_inliers),
            inlier_indices=best_inliers,
        ))

        # Remove inliers and continue
        keep = [i for i in range(n_pts) if i not in set(best_inliers)]
        remaining = remaining.select_by_index(keep)

        if progress_cb is not None:
            progress_cb(step + 1, max_cylinders)

    return results


# ---------------------------------------------------------------------------
# Sphere detection (custom RANSAC)
# ---------------------------------------------------------------------------

def detect_spheres(pcd, distance_threshold=0.5, min_inliers=50,
                   max_spheres=10, num_iterations=1000,
                   progress_cb: Optional[Callable] = None):
    """
    Detect spheres using a custom RANSAC.

    Hypothesis: sample 1 point + its outward normal. The sphere centre lies
    along that normal. Sample a second point and use the constraint that both
    points are equidistant from the centre to solve for the radius.

    Args:
        pcd:                open3d.geometry.PointCloud (must have normals)
        distance_threshold: max distance from sphere surface (mm)
        min_inliers:        minimum inlier count to accept a detection
        max_spheres:        maximum spheres to extract
        num_iterations:     RANSAC iterations per sphere
        progress_cb:        optional callable(step, total)

    Returns:
        list of SphereResult
    """
    require_open3d()
    results = []
    remaining = pcd

    for step in range(max_spheres):
        pts = np.asarray(remaining.points)
        nrm = np.asarray(remaining.normals)
        n_pts = len(pts)

        if n_pts < min_inliers:
            break

        best_inliers = []
        best = None
        rng = np.random.default_rng()

        for _ in range(num_iterations):
            i, j = rng.choice(n_pts, size=2, replace=False)
            # Outward normals point AWAY from centre, so:
            #   c = pts[i] - t * nrm[i],  t = radius > 0
            # Constraint: pts[j] also on sphere → |pts[j] - c|² = t²
            #   let e = pts[j] - pts[i]
            #   |e + t*nrm[i]|² = t²
            #   e·e + 2t(e·nrm[i]) + t²‖nrm‖² = t²
            #   since ‖nrm‖ = 1 → e·e + 2t(e·nrm[i]) = 0
            #   t = -(e·e) / (2 * e·nrm[i])
            # For distinct sphere points e·nrm[i] < 0, so t > 0 always.
            e  = pts[j] - pts[i]
            ni = nrm[i]
            denom = 2.0 * np.dot(e, ni)
            if abs(denom) < 1e-6:
                continue  # degenerate
            t = -np.dot(e, e) / denom
            if t <= 0:
                continue

            centre = pts[i] - t * ni
            radius = t

            dist = np.abs(np.linalg.norm(pts - centre, axis=1) - radius)
            inlier_mask = dist < distance_threshold
            n_inliers = inlier_mask.sum()

            if n_inliers > len(best_inliers):
                best_inliers = np.where(inlier_mask)[0].tolist()
                best = (centre, radius)

        if best is None or len(best_inliers) < min_inliers:
            break

        centre, radius = best
        results.append(SphereResult(
            center=centre,
            radius=float(radius),
            inlier_count=len(best_inliers),
            inlier_indices=best_inliers,
        ))

        keep = [i for i in range(n_pts) if i not in set(best_inliers)]
        remaining = remaining.select_by_index(keep)

        if progress_cb is not None:
            progress_cb(step + 1, max_spheres)

    return results


# ---------------------------------------------------------------------------
# Combined detection
# ---------------------------------------------------------------------------

def detect_all(pcd, params, progress_cb: Optional[Callable] = None):
    """
    Run all detectors and return (planes, cylinders, spheres).

    Args:
        pcd:    open3d.geometry.PointCloud
        params: dict with keys matching the detect_* keyword arguments.
                Unrecognised keys are ignored.
        progress_cb: optional callable(step, total_steps)

    Returns:
        tuple (List[PlaneResult], List[CylinderResult], List[SphereResult])
    """
    def _cb(step, total, offset, grand_total):
        if progress_cb:
            progress_cb(offset + step, grand_total)

    dt   = params.get("distance_threshold", 0.5)
    mi   = params.get("min_inliers", 50)
    ni   = params.get("num_iterations", 1000)
    mp   = params.get("max_planes", 10)
    mc   = params.get("max_cylinders", 10)
    ms   = params.get("max_spheres", 10)
    grand = mp + mc + ms

    planes = detect_planes(
        pcd,
        distance_threshold=dt, min_inliers=mi,
        num_iterations=ni, max_planes=mp,
        progress_cb=lambda s, t: _cb(s, t, 0, grand),
    ) if params.get("detect_planes", True) else []

    cylinders = detect_cylinders(
        pcd,
        distance_threshold=dt, min_inliers=mi,
        num_iterations=ni, max_cylinders=mc,
        progress_cb=lambda s, t: _cb(s, t, mp, grand),
    ) if params.get("detect_cylinders", True) else []

    spheres = detect_spheres(
        pcd,
        distance_threshold=dt, min_inliers=mi,
        num_iterations=ni, max_spheres=ms,
        progress_cb=lambda s, t: _cb(s, t, mp + mc, grand),
    ) if params.get("detect_spheres", True) else []

    return planes, cylinders, spheres
