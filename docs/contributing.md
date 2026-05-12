# Contributing

## Development setup

```bash
git clone https://github.com/mtalasek/MeshRANSAC
cd MeshRANSAC
# Symlink into FreeCAD Mod directory so FreeCAD picks up live edits
ln -s $(pwd) ~/.local/share/FreeCAD/Mod/MeshRANSAC
```

## Running tests

Tests run headlessly inside FreeCAD's Python:

```bash
freecad --console tests/test_runner.py
```

Or individually:

```bash
/usr/lib/freecad/bin/python -m pytest tests/
```

## Code structure

```
MeshRANSAC/
├── core/          — algorithm logic (no GUI, testable standalone)
├── gui/           — Qt task panels and icons
├── commands/      — FreeCAD command objects (thin wrappers)
└── utils/         — logging, dependency checks
```

Keep `core/` free of FreeCAD GUI imports so tests can run headlessly.
Guard any `FreeCADGui` import with `if FreeCAD.GuiUp:`.

## Commit style

Imperative subject line, 72 chars max. Reference phase numbers where relevant:
`Phase 3: implement cylinder RANSAC detection`

## License

All contributions must be LGPL-2.1-or-later to remain compatible with FreeCAD
core for the planned upstream merge (Phase 9).
