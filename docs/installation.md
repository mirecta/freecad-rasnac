# Installation

## Requirements

- FreeCAD 0.21 or later
- open3d >= 0.18 (must be installed into FreeCAD's bundled Python — see below)

## Step 1 — Install the workbench

### Via Addon Manager (recommended)

1. Open FreeCAD
2. Go to **Tools → Addon Manager**
3. Search for **MeshRANSAC**
4. Click **Install** and restart FreeCAD

### Manual

```bash
cd ~/.local/share/FreeCAD/Mod   # Linux
# C:\Users\<you>\AppData\Roaming\FreeCAD\Mod   # Windows
git clone https://github.com/mtalasek/MeshRANSAC
```

## Step 2 — Install open3d into FreeCAD's Python

FreeCAD bundles its own Python interpreter. `open3d` must be installed into
that interpreter, not your system Python.

### Linux (system package)

```bash
/usr/lib/freecad/bin/python -m pip install open3d
```

### Linux (AppImage)

```bash
./FreeCAD-<version>.AppImage --appimage-extract-and-run python -m pip install open3d
```

### Windows

```bat
"C:\Program Files\FreeCAD 0.21\bin\python.exe" -m pip install open3d
```

### macOS

```bash
/Applications/FreeCAD.app/Contents/Resources/bin/python -m pip install open3d
```

## Verifying the installation

Open FreeCAD, switch to the **MeshRANSAC** workbench. If open3d is missing,
a warning is printed to the FreeCAD console with the exact install command.
