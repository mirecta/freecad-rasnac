# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import FreeCAD
import FreeCADGui


class DetectPlanesCommand:
    def GetResources(self):
        return {
            "Pixmap": os.path.join(
                os.path.dirname(__file__), "..", "gui", "icons", "detect_planes.svg"
            ),
            "MenuText": "Detect Planes",
            "ToolTip": "Run RANSAC to detect planar surfaces",
        }

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        return len(sel) == 1 and sel[0].isDerivedFrom("Mesh::Feature")

    def Activated(self):
        from MeshRANSAC.gui.panel_detect import DetectPanel
        panel = DetectPanel(detect_planes=True, detect_cylinders=False, detect_spheres=False)
        FreeCADGui.Control.showTaskPanel(panel)


FreeCADGui.addCommand("MeshRANSAC_DetectPlanes", DetectPlanesCommand())
