# SPDX-License-Identifier: LGPL-2.1-or-later
# MeshRANSAC FreeCAD Workbench — headless / console-mode initialisation
#
# This file is executed by FreeCAD in both GUI and console mode.
# Keep it GUI-free: no FreeCADGui, no Qt imports.

import FreeCAD
from MeshRANSAC.utils.dependency_checker import check_dependencies

_ok, _msg = check_dependencies(silent=True)
if not _ok:
    FreeCAD.Console.PrintWarning(f"MeshRANSAC: {_msg}\n")
