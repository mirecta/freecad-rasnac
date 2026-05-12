# SPDX-License-Identifier: LGPL-2.1-or-later
"""Tests for core/shape_builder.py.

These tests require FreeCAD's Part module.  Run headlessly:
    freecad --console tests/test_runner.py

The tests mock the document object so FreeCAD GUI is not needed.
"""

import json
import unittest
import numpy as np
from types import SimpleNamespace

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

_skip = unittest.skipUnless(HAS_FREECAD, "FreeCAD not available")


# ---------------------------------------------------------------------------
# Minimal result stubs
# ---------------------------------------------------------------------------

def _plane_result(normal=(0, 0, 1), center=(0, 0, 0), n_inliers=100):
    from MeshRANSAC.core.ransac_engine import PlaneResult
    bbox = np.array([
        [-5, -5, 0], [5, -5, 0], [5, 5, 0], [-5, 5, 0],
        [-5, -5, 1], [5, -5, 1], [5, 5, 1], [-5, 5, 1],
    ], dtype=float)
    return PlaneResult(
        normal=np.asarray(normal, dtype=float),
        offset=0.0,
        center=np.asarray(center, dtype=float),
        inlier_count=n_inliers,
        inlier_indices=list(range(n_inliers)),
        bounding_box=bbox,
    )


def _cylinder_result(radius=5.0, height=20.0, center=(0, 0, 10),
                     axis=(0, 0, 1), n_inliers=100):
    from MeshRANSAC.core.ransac_engine import CylinderResult
    ax = np.asarray(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    return CylinderResult(
        axis_point=np.asarray(center, dtype=float),
        axis_dir=ax,
        radius=radius,
        height=height,
        center=np.asarray(center, dtype=float),
        inlier_count=n_inliers,
        inlier_indices=list(range(n_inliers)),
    )


def _sphere_result(center=(0, 0, 0), radius=10.0, n_inliers=100):
    from MeshRANSAC.core.ransac_engine import SphereResult
    return SphereResult(
        center=np.asarray(center, dtype=float),
        radius=radius,
        inlier_count=n_inliers,
        inlier_indices=list(range(n_inliers)),
    )


# ---------------------------------------------------------------------------
# Minimal FreeCAD document stub (no GUI needed)
# ---------------------------------------------------------------------------

class _FakeViewObject:
    Transparency = 0
    DisplayMode = ""
    DiffuseColor = []


class _FakeObj:
    def __init__(self, name):
        self.Name = name
        self.Label = name
        self.Shape = None
        self._props = {}
        self.ViewObject = _FakeViewObject()

    def addProperty(self, ptype, pname, group="", doc=""):
        self._props[pname] = ""

    def __getattr__(self, name):
        if name in self.__dict__.get("_props", {}):
            return self._props[name]
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name != "_props" and name in self.__dict__.get("_props", {}):
            self._props[name] = value
        else:
            super().__setattr__(name, value)


class _FakeDoc:
    def __init__(self):
        self._objects = {}
        self._recomputed = False

    def getObject(self, name):
        return self._objects.get(name)

    def addObject(self, obj_type, name):
        obj = _FakeObj(name)
        self._objects[name] = obj
        return obj

    def recompute(self):
        self._recomputed = True


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@_skip
class TestBuildPlaneShape(unittest.TestCase):

    def test_returns_shape(self):
        from MeshRANSAC.core.shape_builder import build_plane_shape
        shape = build_plane_shape(_plane_result())
        self.assertIsInstance(shape, Part.Shape)

    def test_is_solid(self):
        from MeshRANSAC.core.shape_builder import build_plane_shape
        shape = build_plane_shape(_plane_result())
        self.assertTrue(shape.isValid())
        self.assertGreater(shape.Volume, 0)

    def test_thickness_is_1mm(self):
        from MeshRANSAC.core.shape_builder import build_plane_shape
        # For a Z-normal plane the bounding box Z-extent should be ~1mm
        shape = build_plane_shape(_plane_result(normal=(0, 0, 1)))
        bb = shape.BoundBox
        self.assertAlmostEqual(bb.ZLength, 1.0, delta=0.1)


@_skip
class TestBuildCylinderShape(unittest.TestCase):

    def test_returns_solid(self):
        from MeshRANSAC.core.shape_builder import build_cylinder_shape
        shape = build_cylinder_shape(_cylinder_result(radius=5.0, height=20.0))
        self.assertIsInstance(shape, Part.Shape)
        self.assertTrue(shape.isValid())
        self.assertGreater(shape.Volume, 0)

    def test_radius_in_bounding_box(self):
        from MeshRANSAC.core.shape_builder import build_cylinder_shape
        r = 5.0
        shape = build_cylinder_shape(_cylinder_result(radius=r, height=20.0,
                                                      axis=(0, 0, 1)))
        bb = shape.BoundBox
        # Diameter should match XY extents within tolerance
        self.assertAlmostEqual(bb.XLength, 2 * r, delta=0.5)
        self.assertAlmostEqual(bb.YLength, 2 * r, delta=0.5)

    def test_height_in_bounding_box(self):
        from MeshRANSAC.core.shape_builder import build_cylinder_shape
        h = 20.0
        shape = build_cylinder_shape(_cylinder_result(radius=5.0, height=h,
                                                      axis=(0, 0, 1)))
        self.assertAlmostEqual(shape.BoundBox.ZLength, h, delta=0.5)


@_skip
class TestBuildSphereShape(unittest.TestCase):

    def test_returns_solid(self):
        from MeshRANSAC.core.shape_builder import build_sphere_shape
        shape = build_sphere_shape(_sphere_result(radius=10.0))
        self.assertIsInstance(shape, Part.Shape)
        self.assertTrue(shape.isValid())
        self.assertGreater(shape.Volume, 0)

    def test_bounding_box_matches_radius(self):
        from MeshRANSAC.core.shape_builder import build_sphere_shape
        r = 10.0
        shape = build_sphere_shape(_sphere_result(radius=r, center=(0, 0, 0)))
        bb = shape.BoundBox
        self.assertAlmostEqual(bb.XLength, 2 * r, delta=0.5)
        self.assertAlmostEqual(bb.YLength, 2 * r, delta=0.5)
        self.assertAlmostEqual(bb.ZLength, 2 * r, delta=0.5)


@_skip
class TestBuildCompound(unittest.TestCase):

    def _doc(self):
        return _FakeDoc()

    def test_correct_subshape_count(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        obj = build_compound(
            doc,
            planes=[_plane_result()],
            cylinders=[_cylinder_result(), _cylinder_result(radius=3.0)],
            spheres=[_sphere_result()],
            source_name="Test",
        )
        self.assertEqual(len(obj.Shape.SubShapes), 4)

    def test_sublabels_json_property(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        obj = build_compound(
            doc,
            planes=[_plane_result()],
            cylinders=[_cylinder_result()],
            spheres=[],
            source_name="Test",
        )
        labels = json.loads(obj.SubShapeLabels)
        self.assertEqual(labels["0"], "Plane_001")
        self.assertEqual(labels["1"], "Cylinder_001")

    def test_compound_name_uses_source(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        obj = build_compound(doc, [_plane_result()], [], [], source_name="MyScan")
        self.assertEqual(obj.Name, "RANSAC_MyScan")

    def test_rerun_replaces_existing(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        obj1 = build_compound(doc, [_plane_result()], [], [], source_name="S")
        obj2 = build_compound(doc, [_plane_result(), _plane_result()],
                              [], [], source_name="S")
        # Same Python object — shape replaced in place
        self.assertIs(obj1, obj2)
        self.assertEqual(len(obj2.Shape.SubShapes), 2)

    def test_empty_raises_value_error(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        with self.assertRaises(ValueError):
            build_compound(doc, [], [], [], source_name="Test")

    def test_document_recomputed(self):
        from MeshRANSAC.core.shape_builder import build_compound
        doc = self._doc()
        build_compound(doc, [_plane_result()], [], [], source_name="Test")
        self.assertTrue(doc._recomputed)


if __name__ == "__main__":
    unittest.main()
