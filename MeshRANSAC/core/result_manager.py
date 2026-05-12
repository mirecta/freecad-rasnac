# SPDX-License-Identifier: LGPL-2.1-or-later
"""Insert RANSAC results into the FreeCAD document tree."""


def insert_results(doc, planes, cylinders, spheres, source_name="Scan"):
    """
    Build the compound and add it to the document.
    Thin wrapper around shape_builder.build_compound kept separate
    to make future result-caching / undo logic easier to add.
    """
    from MeshRANSAC.core.shape_builder import build_compound
    return build_compound(doc, planes, cylinders, spheres, source_name)
