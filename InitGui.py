# SPDX-License-Identifier: LGPL-2.1-or-later
# MeshRANSAC FreeCAD Workbench — GUI initialisation

import FreeCADGui
from MeshRANSAC.workbench import MeshRANSACWorkbench

FreeCADGui.addWorkbench(MeshRANSACWorkbench)
