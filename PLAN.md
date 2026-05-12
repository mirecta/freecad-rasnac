# FreeCAD RANSAC Shape Detection Workbench — Implementation Plan

## Project Goal

Implement an external FreeCAD workbench called **MeshRANSAC** that detects geometric
primitives (planes, cylinders, spheres) from imported 3D scan meshes using the RANSAC
algorithm. Output is a single **`Part::Compound`** object in the document tree containing
all detected primitives as named sub-shapes, ready to use in Part/PartDesign workbenches.
Intended for eventual merge request into FreeCAD core Mesh workbench.

---

## Repository Structure

```
MeshRANSAC/
├── PLAN.md                        # This file
├── README.md
├── LICENSE                        # LGPL-2.1 (matches FreeCAD)
├── package.xml                    # FreeCAD Addon Manager metadata
├── InitGui.py                     # Workbench registration
├── Init.py                        # Non-GUI init (console mode)
├── MeshRANSAC/
│   ├── __init__.py
│   ├── workbench.py               # Workbench class definition
│   ├── commands/
│   │   ├── __init__.py
│   │   ├── cmd_detect_planes.py   # Detect plane primitives
│   │   ├── cmd_detect_cylinders.py# Detect cylinder primitives
│   │   ├── cmd_detect_spheres.py  # Detect sphere primitives
│   │   └── cmd_detect_all.py      # Run all detections at once
│   ├── core/
│   │   ├── __init__.py
│   │   ├── mesh_converter.py      # FreeCAD mesh → Open3D point cloud
│   │   ├── ransac_engine.py       # RANSAC detection logic (Open3D)
│   │   ├── shape_builder.py       # Detected params → FreeCAD Part shapes
│   │   └── result_manager.py      # Insert results into document tree
│   ├── gui/
│   │   ├── __init__.py
│   │   ├── panel_detect.py        # Task panel UI (Qt)
│   │   └── icons/
│   │       ├── workbench.svg
│   │       ├── detect_planes.svg
│   │       ├── detect_cylinders.svg
│   │       ├── detect_spheres.svg
│   │       └── detect_all.svg
│   └── utils/
│       ├── __init__.py
│       ├── dependency_checker.py  # Check Open3D installed
│       └── logger.py              # FreeCAD console logging helpers
├── tests/
│   ├── __init__.py
│   ├── test_mesh_converter.py
│   ├── test_ransac_engine.py
│   ├── test_shape_builder.py
│   └── sample_data/
│       ├── flat_board.ply         # Simulated PCB scan (flat plane)
│       ├── cylinder_part.ply      # Cylindrical component
│       └── mixed_scene.ply        # Multiple primitives
└── docs/
    ├── installation.md
    ├── usage.md
    └── contributing.md
```

---

## Dependencies

| Library   | Version   | Purpose                              | How to install              |
|-----------|-----------|--------------------------------------|-----------------------------|
| FreeCAD   | >= 0.21   | Host application & Part API          | System / AppImage           |
| Open3D    | >= 0.18   | RANSAC point cloud processing        | `pip install open3d`        |
| numpy     | >= 1.24   | Array math for coordinate transforms | included with Open3D        |
| PySide2   | any       | Qt UI panels                         | bundled with FreeCAD        |

> **Note for Claude Code:** Check if Open3D is available at runtime via
> `dependency_checker.py` and show a helpful install message if missing.
> FreeCAD bundles its own Python, so install path matters:
> `<FreeCAD_dir>/bin/python -m pip install open3d`

---

## Phase 1 — Project Scaffold

### Tasks
- [ ] Create repository folder structure as shown above
- [ ] Write `package.xml` with addon metadata for FreeCAD Addon Manager
- [ ] Write `InitGui.py` to register the workbench class
- [ ] Write `Init.py` for headless/console mode compatibility
- [ ] Write `workbench.py` defining menus, toolbars, and command list
- [ ] Create placeholder SVG icons (simple shapes, LGPL-compatible)
- [ ] Write `dependency_checker.py` to verify Open3D at startup
- [ ] Write `logger.py` wrapping `FreeCAD.Console.PrintMessage/Warning/Error`

### Key code — `InitGui.py`
```python
import FreeCADGui
from MeshRANSAC.workbench import MeshRANSACWorkbench
FreeCADGui.addWorkbench(MeshRANSACWorkbench)
```

### Key code — `workbench.py`
```python
import FreeCADGui, FreeCAD
from PySide2 import QtGui

class MeshRANSACWorkbench(FreeCADGui.Workbench):
    MenuText = "MeshRANSAC"
    ToolTip = "RANSAC primitive detection from 3D scan meshes"
    Icon = ":/icons/workbench.svg"

    def Initialize(self):
        from MeshRANSAC.commands import (
            cmd_detect_planes, cmd_detect_cylinders,
            cmd_detect_spheres, cmd_detect_all
        )
        self.appendToolbar("MeshRANSAC", [
            "MeshRANSAC_DetectAll",
            "MeshRANSAC_DetectPlanes",
            "MeshRANSAC_DetectCylinders",
            "MeshRANSAC_DetectSpheres",
        ])
        self.appendMenu("MeshRANSAC", [
            "MeshRANSAC_DetectAll",
            "MeshRANSAC_DetectPlanes",
            "MeshRANSAC_DetectCylinders",
            "MeshRANSAC_DetectSpheres",
        ])

    def GetClassName(self):
        return "Gui::PythonWorkbench"
```

---

## Phase 2 — Core: Mesh Converter

**File:** `core/mesh_converter.py`

### Goal
Convert a FreeCAD `Mesh::Feature` object into an `open3d.geometry.PointCloud`
by sampling vertices and computing normals.

### Tasks
- [ ] Accept a FreeCAD mesh object as input
- [ ] Extract vertex coordinates as numpy array via `mesh.Points`
- [ ] Extract face normals via `mesh.Facets`
- [ ] Build Open3D PointCloud with positions and normals
- [ ] Optionally subsample with `voxel_down_sample()` for large scans
- [ ] Handle empty mesh or wrong object type with clear error messages

### Key code
```python
import numpy as np
import open3d as o3d
import FreeCAD

def freecad_mesh_to_pointcloud(mesh_feature, voxel_size=None):
    """
    Convert a FreeCAD Mesh::Feature to an Open3D PointCloud.

    Args:
        mesh_feature: FreeCAD document object with mesh data
        voxel_size: float or None — downsample voxel size in mm

    Returns:
        open3d.geometry.PointCloud
    """
    mesh = mesh_feature.Mesh

    # Extract vertices
    points = np.array([[p.x, p.y, p.z] for p in mesh.Points])

    # Extract face normals and assign to vertices (averaged per vertex)
    normals = _compute_vertex_normals(mesh)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)

    if voxel_size is not None:
        pcd = pcd.voxel_down_sample(voxel_size)

    return pcd


def _compute_vertex_normals(mesh):
    """Average face normals onto vertices."""
    points_count = len(mesh.Points)
    normals = np.zeros((points_count, 3))
    counts = np.zeros(points_count)

    for facet in mesh.Facets:
        n = facet.Normal
        for idx in facet.PointIndices:
            normals[idx] += [n.x, n.y, n.z]
            counts[idx] += 1

    counts = np.maximum(counts, 1)
    normals = normals / counts[:, np.newaxis]

    # Normalize
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-8)
    return normals / norms
```

---

## Phase 3 — Core: RANSAC Engine

**File:** `core/ransac_engine.py`

### Goal
Run RANSAC detection on a point cloud and return structured results for each
detected primitive.

### Tasks
- [ ] Implement `detect_planes(pcd, params)` — iterative plane extraction
- [ ] Implement `detect_cylinders(pcd, params)` — using Open3D or manual RANSAC
- [ ] Implement `detect_spheres(pcd, params)` — using Open3D
- [ ] Implement `detect_all(pcd, params)` — runs all, returns combined list
- [ ] Each function returns a list of `PrimitiveResult` dataclass objects
- [ ] Remove inliers between iterations so multiple shapes are found
- [ ] Respect `min_inliers` threshold to skip noise detections
- [ ] Add progress reporting via callback for GUI progress bar

### Data structures
```python
from dataclasses import dataclass, field
from typing import List
import numpy as np

@dataclass
class PlaneResult:
    normal: np.ndarray      # [a, b, c] unit normal
    offset: float           # d in ax+by+cz+d=0
    center: np.ndarray      # centroid of inlier points
    inlier_count: int
    inlier_indices: List[int]
    bounding_box: np.ndarray  # 8 corners of oriented bounding box

@dataclass
class CylinderResult:
    axis_point: np.ndarray  # point on cylinder axis
    axis_dir: np.ndarray    # unit direction of axis
    radius: float           # in mm
    height: float           # estimated from inlier extent
    center: np.ndarray      # midpoint of axis segment
    inlier_count: int
    inlier_indices: List[int]

@dataclass
class SphereResult:
    center: np.ndarray
    radius: float
    inlier_count: int
    inlier_indices: List[int]
```

### Key code — plane detection loop
```python
def detect_planes(pcd, distance_threshold=0.5, ransac_n=3,
                  num_iterations=1000, min_inliers=50, max_planes=10):
    results = []
    remaining = pcd

    for _ in range(max_planes):
        if len(remaining.points) < min_inliers:
            break

        model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )

        if len(inliers) < min_inliers:
            break

        inlier_cloud = remaining.select_by_index(inliers)
        pts = np.asarray(inlier_cloud.points)
        center = pts.mean(axis=0)
        obb = inlier_cloud.get_oriented_bounding_box()

        results.append(PlaneResult(
            normal=np.array(model[:3]),
            offset=model[3],
            center=center,
            inlier_count=len(inliers),
            inlier_indices=inliers,
            bounding_box=np.asarray(obb.get_box_points())
        ))

        # Remove inliers and continue
        remaining = remaining.select_by_index(inliers, invert=True)

    return results
```

> **Note:** Open3D has built-in `segment_plane()` but cylinder/sphere detection
> requires either Open3D's `SACSegmentation` (via `open3d.t` tensor API) or
> a custom implementation. Implement custom RANSAC for cylinders — sample 2
> points + normal to define axis, fit radius. See reference algorithm in
> `docs/ransac_cylinder_math.md` (to be written).

---

## Phase 4 — Core: Shape Builder

**File:** `core/shape_builder.py`

### Goal
Convert all `PrimitiveResult` objects into a single **`Part::Compound`** document object
containing all detected primitives as named sub-shapes. One clean entry in the model tree,
fully usable in Part/PartDesign (boolean ops, measurements, references).

### Document tree result
```
📄 Document
 ├── 📦 PCB_Scan              (original Mesh::Feature — untouched)
 └── 🔷 RANSAC_Compound       (Part::Feature with compound shape)
      ├── Plane_001            (sub-shape, accessible via .Shape.SubShapes[0])
      ├── Plane_002
      ├── Cylinder_001
      ├── Cylinder_002
      ├── Cylinder_003
      └── Sphere_001
```

### Tasks
- [ ] `build_plane_shape(result)` → returns a `Part.Shape` (flat box 1mm thick, oriented)
- [ ] `build_cylinder_shape(result)` → returns a `Part.Shape` (cylinder, oriented along axis)
- [ ] `build_sphere_shape(result)` → returns a `Part.Shape` (sphere at center)
- [ ] `build_compound(doc, all_results, source_mesh_name)` → assembles all shapes into
      `Part.makeCompound(shapes)` and inserts ONE `Part::Feature` into the document
- [ ] Name each sub-shape by setting `Shape.Tag` and store a label map as a JSON property
      on the compound object so sub-shapes stay identifiable after save/load
- [ ] Set compound transparency to 50% so underlying mesh is visible
- [ ] Set compound color per primitive type using `Part.Shape` color face API
- [ ] Name the compound object `RANSAC_<source_mesh_name>` e.g. `RANSAC_PCB_Scan`
- [ ] If a compound with that name already exists, replace its shape (re-run workflow)
- [ ] Recompute document once after inserting compound

### Key code — shape builders (geometry only, no document objects)
```python
import FreeCAD, Part
import numpy as np

def build_plane_shape(result: PlaneResult) -> Part.Shape:
    """
    Build a flat box 1mm thick representing the detected plane.
    Sized to the oriented bounding box of inlier points.
    """
    pts = result.bounding_box  # 8 corner points (numpy array)
    obb_center = pts.mean(axis=0)

    # Compute OBB dimensions from corner points
    # Use the two longest edges as width/length, 1mm as thickness
    edges = [
        np.linalg.norm(pts[1] - pts[0]),
        np.linalg.norm(pts[3] - pts[0]),
    ]
    width, length = sorted(edges, reverse=True)[:2]
    thickness = 1.0  # mm

    shape = Part.makeBox(width, length, thickness)

    # Center the box at OBB center, rotate to plane normal
    z_axis = FreeCAD.Vector(0, 0, 1)
    normal = FreeCAD.Vector(*result.normal)
    rotation = FreeCAD.Rotation(z_axis, normal)
    center = FreeCAD.Vector(*obb_center)
    offset = FreeCAD.Vector(*result.normal) * (thickness / 2)
    placement = FreeCAD.Placement(center - offset, rotation)

    shape.Placement = placement
    return shape


def build_cylinder_shape(result: CylinderResult) -> Part.Shape:
    """
    Build a cylinder solid oriented along the detected axis.
    """
    shape = Part.makeCylinder(result.radius, result.height)

    z_axis = FreeCAD.Vector(0, 0, 1)
    axis_dir = FreeCAD.Vector(*result.axis_dir)
    rotation = FreeCAD.Rotation(z_axis, axis_dir)

    half_h = axis_dir * (result.height / 2)
    origin = FreeCAD.Vector(*result.center) - half_h
    shape.Placement = FreeCAD.Placement(origin, rotation)
    return shape


def build_sphere_shape(result: SphereResult) -> Part.Shape:
    """
    Build a sphere at the detected center with detected radius.
    """
    shape = Part.makeSphere(result.radius)
    shape.Placement = FreeCAD.Placement(
        FreeCAD.Vector(*result.center),
        FreeCAD.Rotation()
    )
    return shape
```

### Key code — compound assembler
```python
import json

def build_compound(doc, planes, cylinders, spheres, source_name="Scan"):
    """
    Assemble all detected primitive shapes into a single Part::Compound
    and insert it into the FreeCAD document.

    Args:
        doc:        FreeCAD active document
        planes:     list of PlaneResult
        cylinders:  list of CylinderResult
        spheres:    list of SphereResult
        source_name: name of the source mesh object (for naming)

    Returns:
        FreeCAD document object (Part::Feature with compound shape)
    """
    shapes = []
    label_map = {}   # index → human label, stored as JSON property
    index = 0

    for i, r in enumerate(planes):
        s = build_plane_shape(r)
        shapes.append(s)
        label_map[index] = f"Plane_{i+1:03d}"
        index += 1

    for i, r in enumerate(cylinders):
        s = build_cylinder_shape(r)
        shapes.append(s)
        label_map[index] = f"Cylinder_{i+1:03d}"
        index += 1

    for i, r in enumerate(spheres):
        s = build_sphere_shape(r)
        shapes.append(s)
        label_map[index] = f"Sphere_{i+1:03d}"
        index += 1

    if not shapes:
        raise ValueError("No primitives detected — compound not created.")

    compound = Part.makeCompound(shapes)
    obj_name = f"RANSAC_{source_name}"

    # Reuse existing compound object if present (re-run case)
    existing = doc.getObject(obj_name)
    if existing:
        compound_obj = existing
    else:
        compound_obj = doc.addObject("Part::Feature", obj_name)

    compound_obj.Shape = compound

    # Store label map so sub-shapes stay named after save/load
    # Uses a custom string property on the object
    if not hasattr(compound_obj, "SubShapeLabels"):
        compound_obj.addProperty(
            "App::PropertyString",
            "SubShapeLabels",
            "RANSAC",
            "JSON map of sub-shape index to label"
        )
    compound_obj.SubShapeLabels = json.dumps(label_map)

    # Visual settings
    if FreeCAD.GuiUp:
        vobj = compound_obj.ViewObject
        vobj.Transparency = 50
        vobj.DisplayMode = "Shaded"
        # Color individual faces by primitive type
        _apply_face_colors(compound_obj, label_map)

    doc.recompute()
    return compound_obj


def _apply_face_colors(compound_obj, label_map):
    """
    Color compound sub-shapes by type:
      Plane    → blue   (0.3, 0.5, 1.0)
      Cylinder → green  (0.2, 0.8, 0.2)
      Sphere   → orange (1.0, 0.6, 0.1)
    """
    colors = []
    shape = compound_obj.Shape
    for i, sub in enumerate(shape.SubShapes):
        label = label_map.get(i, "")
        if label.startswith("Plane"):
            c = (0.3, 0.5, 1.0)
        elif label.startswith("Cylinder"):
            c = (0.2, 0.8, 0.2)
        elif label.startswith("Sphere"):
            c = (1.0, 0.6, 0.1)
        else:
            c = (0.7, 0.7, 0.7)
        # One color entry per face in this sub-shape
        face_count = len(sub.Faces)
        colors.extend([c] * face_count)

    compound_obj.ViewObject.DiffuseColor = colors
```

### Accessing sub-shapes after creation
```python
# Get the compound
comp = doc.getObject("RANSAC_PCB_Scan")
labels = json.loads(comp.SubShapeLabels)

# Iterate sub-shapes with their names
for i, sub_shape in enumerate(comp.Shape.SubShapes):
    name = labels.get(str(i), f"Unknown_{i}")
    print(f"{name}: volume={sub_shape.Volume:.2f} mm³")

# Use a specific sub-shape in a boolean operation
cylinder_shape = comp.Shape.SubShapes[2]
result = board_shape.cut(cylinder_shape)   # cut hole in board model
```

---

## Phase 5 — GUI: Task Panel

**File:** `gui/panel_detect.py`

### Goal
Qt task panel (FreeCAD sidebar style) with parameters and a Run button.

### UI Elements
```
┌─────────────────────────────────┐
│  MeshRANSAC — Detect Primitives │
├─────────────────────────────────┤
│  Selected mesh: [PCB_Scan]      │
│                                 │
│  Shape types:                   │
│  ☑ Planes                       │
│  ☑ Cylinders                    │
│  ☑ Spheres                      │
│                                 │
│  Parameters:                    │
│  Distance threshold: [0.5] mm   │
│  Min inliers:       [50]        │
│  Max iterations:    [1000]      │
│  Max shapes/type:   [10]        │
│  Voxel downsample:  [0.0] mm    │
│    (0 = disabled)               │
│                                 │
│  [▶ Run Detection]              │
│  ████████░░░░░░░░  60%          │
│                                 │
│  Results:                       │
│  ✓ 1 plane detected             │
│  ✓ 4 cylinders detected         │
│  ✗ No spheres found             │
└─────────────────────────────────┘
```

### Tasks
- [ ] Build panel as `FreeCADGui.Control.showTaskPanel()` compatible class
- [ ] Auto-detect selected mesh object on panel open
- [ ] Validate inputs before running (type checks, positive values)
- [ ] Run detection in a `QThread` to avoid freezing the UI
- [ ] Show `QProgressBar` updated via signal from engine callback
- [ ] Display result summary after completion
- [ ] "Close" button calls `FreeCAD.ActiveDocument.recompute()`

---

## Phase 6 — Commands

### Tasks
- [ ] Implement `MeshRANSAC_DetectAll` — opens panel with all types checked
- [ ] Implement `MeshRANSAC_DetectPlanes` — opens panel with only planes checked
- [ ] Implement `MeshRANSAC_DetectCylinders` — only cylinders
- [ ] Implement `MeshRANSAC_DetectSpheres` — only spheres
- [ ] Each command checks a mesh is selected, shows error if not
- [ ] Register all commands with `FreeCADGui.addCommand()`

### Command template
```python
import FreeCAD, FreeCADGui
from MeshRANSAC.gui.panel_detect import DetectPanel

class DetectAllCommand:
    def GetResources(self):
        return {
            "Pixmap": ":/icons/detect_all.svg",
            "MenuText": "Detect All Primitives",
            "ToolTip": "Run RANSAC to detect planes, cylinders and spheres",
            "Accel": "Shift+R"
        }

    def IsActive(self):
        # Enable only when a Mesh object is selected
        sel = FreeCADGui.Selection.getSelection()
        return len(sel) == 1 and sel[0].isDerivedFrom("Mesh::Feature")

    def Activated(self):
        panel = DetectPanel(detect_planes=True,
                           detect_cylinders=True,
                           detect_spheres=True)
        FreeCADGui.Control.showTaskPanel(panel)

FreeCADGui.addCommand("MeshRANSAC_DetectAll", DetectAllCommand())
```

---

## Phase 7 — Tests

### Tasks
- [ ] Generate synthetic test PLY files in `tests/sample_data/`:
  - `flat_board.ply` — single large plane with noise (simulates PCB)
  - `cylinder_part.ply` — 4 cylinders of varying radius + noise
  - `mixed_scene.ply` — plane + 3 cylinders + sphere
- [ ] `test_mesh_converter.py` — test vertex extraction, normal computation
- [ ] `test_ransac_engine.py` — test detection finds correct count and params
- [ ] `test_shape_builder.py` — test compound created with correct sub-shape count,
      labels JSON property, correct sub-shape types (solid checks), placement accuracy
- [ ] Run tests headlessly: `FreeCAD --console test_runner.py`
- [ ] Assert plane normal matches ground truth within 5°
- [ ] Assert cylinder radius matches within 0.1 mm
- [ ] Assert at least 80% of ground-truth inliers are found

### Test data generation script
```python
# tests/generate_test_data.py
import open3d as o3d
import numpy as np

def make_flat_board(noise=0.1):
    """Simulated PCB: 100x80mm flat plane with Gaussian noise."""
    x = np.random.uniform(0, 100, 5000)
    y = np.random.uniform(0, 80, 5000)
    z = np.random.normal(0, noise, 5000)
    pts = np.column_stack([x, y, z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals()
    o3d.io.write_point_cloud("sample_data/flat_board.ply", pcd)
```

---

## Phase 8 — Packaging & Addon Manager

### Tasks
- [ ] Write `package.xml` with correct metadata:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<package format="1" xmlns="https://wiki.freecad.org/Package_Metadata">
  <name>MeshRANSAC</name>
  <description>RANSAC primitive shape detection from 3D scan meshes</description>
  <version>0.1.0</version>
  <date>2026-05-12</date>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>LGPL-2.1-or-later</license>
  <url type="repository">https://github.com/yourname/MeshRANSAC</url>
  <content>
    <workbench/>
  </content>
  <pythondeps>
    <package version="&gt;=0.18">open3d</package>
  </pythondeps>
</package>
```
- [ ] Write `docs/installation.md` with step-by-step for Linux/Windows/Mac
- [ ] Write `README.md` with screenshots, usage example, and PCB workflow
- [ ] Submit to `FreeCAD/FreeCAD-addons` repo via PR to get into Addon Manager

---

## Phase 9 — Core FreeCAD PR (after addon is validated)

### Tasks
- [ ] Open discussion thread on `forum.freecad.org` under "Developer" section
- [ ] Identify target location: `src/Mod/Mesh/` or `src/Mod/ReverseEngineering/`
- [ ] Port Python RANSAC engine to C++ using CGAL `Shape_detection` module
  (CGAL is already a FreeCAD dependency)
- [ ] Wrap C++ functions with Python bindings via `src/Mod/Mesh/App/MeshPy.xml`
- [ ] Add menu items to existing Mesh workbench `Workbench.cpp`
- [ ] Write unit tests in FreeCAD's test framework
- [ ] Follow contribution process: fork → branch → PR → review
- [ ] Reference the addon for community validation evidence

---

## Implementation Order for Claude Code

Run phases in this order, completing all tasks in each before moving on:

```
Phase 1  → Scaffold (all files, no logic yet)
Phase 2  → mesh_converter.py + its tests
Phase 3  → ransac_engine.py + its tests
Phase 4  → shape_builder.py + its tests
Phase 5  → GUI panel
Phase 6  → Commands wiring
Phase 7  → Full test suite + sample data generation
Phase 8  → package.xml, README, docs
Phase 9  → (manual — requires FreeCAD dev environment + forum discussion)
```

---

## Key Technical Notes for Claude Code

1. **FreeCAD Python path** — When running FreeCAD macros, `sys.path` includes
   FreeCAD's own Python. Open3D must be installed into that Python, not the
   system Python. Detect this in `dependency_checker.py`.

2. **Coordinates** — FreeCAD uses millimeters internally. Open3D has no units.
   Ensure scan data is imported in mm before running RANSAC; `distance_threshold`
   parameters are in mm.

3. **No GUI in tests** — Tests run headless (`FreeCAD --console`). Any import of
   `FreeCADGui` must be guarded with `if FreeCAD.GuiUp:`.

4. **Thread safety** — Never call FreeCAD document API from a QThread directly.
   Use Qt signals to send results back to the main thread for document insertion.

5. **Compound sub-shape identity** — `Part.makeCompound()` does not preserve names
   on sub-shapes at the TopoDS level. Store the label map in a custom
   `App::PropertyString` on the document object as JSON. Always use `str(i)` as
   the key (JSON keys are always strings) and `int(i)` when reading back.

6. **Cylinder detection gap** — Open3D's stable API (`open3d.geometry`) does not
   have built-in cylinder RANSAC. Use the tensor API (`open3d.t.geometry`) or
   implement custom RANSAC. Prefer custom for portability.

6. **PCB-specific tuning** — For PCB scans, recommended starting parameters:
   - `distance_threshold`: 0.3–0.5 mm (scanner noise level)
   - `min_inliers`: 100 (PCB is large relative to components)
   - `voxel_size`: 0.2 mm (reduces point count without losing detail)

7. **License** — All files must have LGPL-2.1 header to be mergeable into FreeCAD.

---

## Reference Links

- FreeCAD Mesh workbench source: `github.com/FreeCAD/FreeCAD/tree/main/src/Mod/Mesh`
- FreeCAD contribution guide: `freecad.org/contributing.php`
- Open3D RANSAC docs: `open3d.org/docs/release/tutorial/geometry/pointcloud.html`
- CGAL Shape Detection: `doc.cgal.org/latest/Shape_detection/index.html`
- Schnabel et al. RANSAC paper: `hinkali.com/Education/PointCloud.pdf`
- FreeCAD Addon Manager submission: `github.com/FreeCAD/FreeCAD-addons`
- FreeCAD forum (post discussion first): `forum.freecad.org`
