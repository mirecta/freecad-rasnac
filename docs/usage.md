# Usage

## Basic workflow

1. **Import your scan** — use **File → Import** or the Mesh workbench to load
   a `.ply`, `.stl`, or `.obj` scan file. The mesh appears as a `Mesh::Feature`
   in the model tree.

2. **Select the mesh** — click the mesh object in the tree so it is highlighted.

3. **Switch to MeshRANSAC** — select the workbench from the workbench selector.

4. **Run detection** — click one of the toolbar buttons:
   - **Detect All Primitives** — finds planes, cylinders, and spheres
   - **Detect Planes** — only planar surfaces
   - **Detect Cylinders** — only cylindrical surfaces
   - **Detect Spheres** — only spherical surfaces

5. **Adjust parameters** in the task panel that appears, then click **Run Detection**.

6. **Inspect results** — a `RANSAC_<MeshName>` compound object appears in the tree.
   Sub-shapes are named `Plane_001`, `Cylinder_001`, etc.

## Parameters

| Parameter          | Default | Description                                      |
|--------------------|---------|--------------------------------------------------|
| Distance threshold | 0.5 mm  | Max distance from model surface to count as inlier |
| Min inliers        | 50      | Shapes with fewer inliers are discarded           |
| Max iterations     | 1000    | RANSAC iterations per candidate shape             |
| Max shapes / type  | 10      | Upper limit on how many shapes to find per type   |
| Voxel downsample   | 0.0 mm  | Pre-process voxel size (0 = disabled)             |

## PCB scanning tips

For typical PCB scans (scanner noise ~0.1–0.3 mm):

- `distance_threshold`: 0.3–0.5 mm
- `min_inliers`: 100 (board surface is large)
- `voxel_size`: 0.2 mm (reduces point count without losing component detail)

## Using detected shapes in PartDesign

```python
import json
comp = App.ActiveDocument.getObject("RANSAC_PCB_Scan")
labels = json.loads(comp.SubShapeLabels)
for i, sub in enumerate(comp.Shape.SubShapes):
    print(labels[str(i)], "volume =", sub.Volume)
```
