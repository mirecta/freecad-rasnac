# SPDX-License-Identifier: LGPL-2.1-or-later
"""Build FreeCAD Part shapes from RANSAC primitive results and assemble a compound."""

import json
import numpy as np

from MeshRANSAC.core.ransac_engine import PlaneResult, CylinderResult, SphereResult


# ---------------------------------------------------------------------------
# Individual shape builders
# ---------------------------------------------------------------------------

def build_plane_shape(result: PlaneResult):
    """
    Return a Part.Shape (flat box 1 mm thick) oriented to the detected plane.

    The box is sized to the oriented bounding box of the inlier points and
    centred at the inlier centroid.
    """
    import FreeCAD
    import Part

    pts = result.bounding_box   # (8, 3) numpy array — OBB corners
    obb_center = pts.mean(axis=0)
    thickness = 1.0  # mm

    # Longest two edge lengths of the OBB → width and length
    edges = sorted([
        np.linalg.norm(pts[1] - pts[0]),
        np.linalg.norm(pts[3] - pts[0]),
        np.linalg.norm(pts[4] - pts[0]),
    ], reverse=True)
    width, length = edges[0], edges[1]

    shape = Part.makeBox(max(width, 0.1), max(length, 0.1), thickness)

    z_axis = FreeCAD.Vector(0, 0, 1)
    normal = FreeCAD.Vector(*result.normal.tolist())
    rotation = FreeCAD.Rotation(z_axis, normal)
    center_vec = FreeCAD.Vector(*obb_center.tolist())
    offset = normal * (thickness / 2)
    shape.Placement = FreeCAD.Placement(center_vec - offset, rotation)

    return shape


def build_cylinder_shape(result: CylinderResult):
    """
    Return a Part.Shape (cylinder solid) oriented along the detected axis.
    """
    import FreeCAD
    import Part

    shape = Part.makeCylinder(
        max(result.radius, 0.1),
        max(result.height, 0.1),
    )

    z_axis = FreeCAD.Vector(0, 0, 1)
    axis_dir = FreeCAD.Vector(*result.axis_dir.tolist())
    rotation = FreeCAD.Rotation(z_axis, axis_dir)

    half_h = axis_dir * (result.height / 2)
    origin = FreeCAD.Vector(*result.center.tolist()) - half_h
    shape.Placement = FreeCAD.Placement(origin, rotation)

    return shape


def build_sphere_shape(result: SphereResult):
    """
    Return a Part.Shape (sphere solid) at the detected centre.
    """
    import FreeCAD
    import Part

    shape = Part.makeSphere(max(result.radius, 0.1))
    shape.Placement = FreeCAD.Placement(
        FreeCAD.Vector(*result.center.tolist()),
        FreeCAD.Rotation(),
    )
    return shape


# ---------------------------------------------------------------------------
# Compound assembler
# ---------------------------------------------------------------------------

def build_compound(doc, planes, cylinders, spheres, source_name="Scan"):
    """
    Assemble all detected primitive shapes into a single Part::Feature compound
    and insert it into the FreeCAD document.

    - Reuses an existing object named RANSAC_<source_name> if present (re-run).
    - Stores a JSON label map as a custom App::PropertyString on the object
      so sub-shape names survive save/reload.
    - Sets 50 % transparency and per-type face colours when running in GUI mode.

    Returns the compound document object.
    """
    import FreeCAD
    import Part

    shapes = []
    label_map = {}  # str(index) → human label
    index = 0

    for i, r in enumerate(planes):
        shapes.append(build_plane_shape(r))
        label_map[str(index)] = f"Plane_{i+1:03d}"
        index += 1

    for i, r in enumerate(cylinders):
        shapes.append(build_cylinder_shape(r))
        label_map[str(index)] = f"Cylinder_{i+1:03d}"
        index += 1

    for i, r in enumerate(spheres):
        shapes.append(build_sphere_shape(r))
        label_map[str(index)] = f"Sphere_{i+1:03d}"
        index += 1

    if not shapes:
        raise ValueError(
            "No primitives detected — compound not created. "
            "Try lowering 'Min inliers' or 'Distance threshold'."
        )

    compound = Part.makeCompound(shapes)
    obj_name = f"RANSAC_{source_name}"

    existing = doc.getObject(obj_name)
    if existing is not None:
        compound_obj = existing
    else:
        compound_obj = doc.addObject("Part::Feature", obj_name)

    compound_obj.Shape = compound

    # Store label map so sub-shapes stay named after save/load
    if not hasattr(compound_obj, "SubShapeLabels"):
        compound_obj.addProperty(
            "App::PropertyString",
            "SubShapeLabels",
            "RANSAC",
            "JSON map of sub-shape index to label",
        )
    compound_obj.SubShapeLabels = json.dumps(label_map)

    if FreeCAD.GuiUp:
        vobj = compound_obj.ViewObject
        vobj.Transparency = 50
        vobj.DisplayMode = "Shaded"
        _apply_face_colors(compound_obj, label_map)

    doc.recompute()
    return compound_obj


# ---------------------------------------------------------------------------
# Colour helpers
# ---------------------------------------------------------------------------

_COLORS = {
    "Plane":    (0.3, 0.5, 1.0),   # blue
    "Cylinder": (0.2, 0.8, 0.2),   # green
    "Sphere":   (1.0, 0.6, 0.1),   # orange
    "Unknown":  (0.7, 0.7, 0.7),
}


def _apply_face_colors(compound_obj, label_map):
    """Colour each sub-shape's faces by primitive type."""
    colors = []
    for i, sub in enumerate(compound_obj.Shape.SubShapes):
        label = label_map.get(str(i), "")
        color_key = next(
            (k for k in _COLORS if label.startswith(k)),
            "Unknown",
        )
        c = _COLORS[color_key]
        colors.extend([c] * len(sub.Faces))

    compound_obj.ViewObject.DiffuseColor = colors
