# MeshRANSAC

A FreeCAD workbench that detects geometric primitives (planes, cylinders, spheres) from imported 3D scan meshes using the RANSAC algorithm.

Output is a single `Part::Compound` object in the document tree containing all detected primitives, ready to use in Part/PartDesign for boolean operations, measurements, and references.

## Use case

You have a 3D scan of a PCB or mechanical part imported as a mesh. MeshRANSAC automatically finds the flat surfaces, cylindrical holes/pins, and spherical features — giving you clean parametric shapes without manual modelling.

```
📄 Document
 ├── 📦 PCB_Scan              (original Mesh::Feature — untouched)
 └── 🔷 RANSAC_PCB_Scan       (Part::Feature with compound shape)
      ├── Plane_001
      ├── Cylinder_001
      ├── Cylinder_002
      └── Sphere_001
```

## Requirements

| Dependency | Version   | Notes                            |
|------------|-----------|----------------------------------|
| FreeCAD    | >= 0.21   | Host application                 |
| open3d     | >= 0.18   | RANSAC point cloud processing    |
| numpy      | >= 1.24   | Included with open3d             |
| PySide2    | any       | Bundled with FreeCAD             |

## Installation

### Via FreeCAD Addon Manager (recommended)

1. Open FreeCAD → **Tools → Addon Manager**
2. Search for **MeshRANSAC**
3. Click **Install**
4. Restart FreeCAD

### Manual installation

```bash
cd ~/.local/share/FreeCAD/Mod
git clone https://github.com/mtalasek/MeshRANSAC
```

### Install open3d into FreeCAD's Python

```bash
# Linux
/usr/lib/freecad/bin/python -m pip install open3d

# Windows (adjust path to your FreeCAD installation)
"C:\Program Files\FreeCAD 0.21\bin\python.exe" -m pip install open3d

# AppImage
./FreeCAD.AppImage --appimage-extract-and-run python -m pip install open3d
```

## Usage

1. Import your scan mesh (**File → Import** or Mesh workbench)
2. Select the mesh object in the model tree
3. Switch to the **MeshRANSAC** workbench
4. Click **Detect All Primitives** (or choose a specific type)
5. Adjust parameters in the task panel and click **Run Detection**
6. The detected primitives appear as `RANSAC_<MeshName>` in the tree

### Parameters

| Parameter           | Default | Description                              |
|---------------------|---------|------------------------------------------|
| Distance threshold  | 0.5 mm  | Point-to-model tolerance (scanner noise) |
| Min inliers         | 50      | Minimum points to accept a detection     |
| Max iterations      | 1000    | RANSAC iterations per shape              |
| Max shapes / type   | 10      | Maximum primitives of each type          |
| Voxel downsample    | 0.0 mm  | Pre-downsample (0 = disabled)            |

**PCB-specific recommendations:** distance threshold 0.3–0.5 mm, min inliers 100, voxel size 0.2 mm.

## Status

| Phase | Description         | Status      |
|-------|---------------------|-------------|
| 1     | Project scaffold    | ✅ Done     |
| 2     | Mesh converter      | 🔲 Planned  |
| 3     | RANSAC engine       | 🔲 Planned  |
| 4     | Shape builder       | 🔲 Planned  |
| 5     | GUI task panel      | 🔲 Planned  |
| 6     | Commands wiring     | 🔲 Planned  |
| 7     | Tests               | 🔲 Planned  |
| 8     | Packaging / docs    | 🔲 Planned  |

## License

LGPL-2.1-or-later — compatible with FreeCAD core for future upstream merge.
