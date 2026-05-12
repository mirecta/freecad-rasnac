# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import FreeCADGui
import FreeCAD


class MeshRANSACWorkbench(FreeCADGui.Workbench):
    MenuText = "MeshRANSAC"
    ToolTip = "RANSAC primitive detection from 3D scan meshes"
    Icon = os.path.join(os.path.dirname(__file__), "gui", "icons", "workbench.svg")

    def Initialize(self):
        from MeshRANSAC.commands import (
            cmd_detect_planes,
            cmd_detect_cylinders,
            cmd_detect_spheres,
            cmd_detect_all,
        )

        commands = [
            "MeshRANSAC_DetectAll",
            "MeshRANSAC_DetectPlanes",
            "MeshRANSAC_DetectCylinders",
            "MeshRANSAC_DetectSpheres",
        ]
        self.appendToolbar("MeshRANSAC", commands)
        self.appendMenu("MeshRANSAC", commands)

        from MeshRANSAC.utils.dependency_checker import check_dependencies
        ok, msg = check_dependencies(silent=False)
        if not ok:
            FreeCAD.Console.PrintWarning(f"MeshRANSAC: {msg}\n")

    def Activated(self):
        pass

    def Deactivated(self):
        pass

    def GetClassName(self):
        return "Gui::PythonWorkbench"
