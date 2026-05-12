# SPDX-License-Identifier: LGPL-2.1-or-later
"""Thin wrappers around FreeCAD.Console for consistent prefixed logging."""

_PREFIX = "MeshRANSAC"


def _console():
    import FreeCAD
    return FreeCAD.Console


def info(msg):
    _console().PrintMessage(f"[{_PREFIX}] {msg}\n")


def warn(msg):
    _console().PrintWarning(f"[{_PREFIX}] {msg}\n")


def error(msg):
    _console().PrintError(f"[{_PREFIX}] {msg}\n")


def log(msg):
    _console().PrintLog(f"[{_PREFIX}] {msg}\n")
