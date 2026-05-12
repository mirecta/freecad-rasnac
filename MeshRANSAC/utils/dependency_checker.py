# SPDX-License-Identifier: LGPL-2.1-or-later
"""Check that optional runtime dependencies are available."""

import sys


def check_dependencies(silent=False):
    """
    Verify that open3d is importable in the current Python environment.

    Returns:
        (ok: bool, message: str)

    FreeCAD bundles its own Python interpreter, so open3d must be installed
    into that interpreter, not the system Python:
        <FreeCAD_dir>/bin/python -m pip install open3d
    """
    try:
        import open3d  # noqa: F401
        return True, "All dependencies satisfied."
    except ImportError:
        msg = (
            "open3d is not installed in FreeCAD's Python environment. "
            "RANSAC detection will not work.\n"
            "Install with: <FreeCAD_dir>/bin/python -m pip install open3d"
        )
        if not silent:
            try:
                import FreeCAD
                FreeCAD.Console.PrintWarning(f"MeshRANSAC: {msg}\n")
            except ImportError:
                pass
        return False, msg


def require_open3d():
    """Raise ImportError with a helpful message if open3d is missing."""
    ok, msg = check_dependencies(silent=True)
    if not ok:
        raise ImportError(msg)
    import open3d
    return open3d
