# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Headless test runner for FreeCAD console mode.

Usage:
    freecad --console tests/test_runner.py

Or with pytest (open3d + FreeCAD both importable):
    python -m pytest tests/ -v
"""

import sys
import os
import unittest

# Make sure the workbench package is on sys.path when run from repo root
_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo not in sys.path:
    sys.path.insert(0, _repo)

loader = unittest.TestLoader()
suite  = loader.discover(start_dir=os.path.dirname(__file__), pattern="test_*.py")

runner = unittest.TextTestRunner(verbosity=2)
result = runner.run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
