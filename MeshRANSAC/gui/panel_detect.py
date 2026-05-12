# SPDX-License-Identifier: LGPL-2.1-or-later
"""Task panel UI for RANSAC primitive detection."""

import FreeCAD
import FreeCADGui

from PySide2 import QtCore, QtWidgets


# ---------------------------------------------------------------------------
# Worker thread — runs detection off the main thread
# ---------------------------------------------------------------------------

class _DetectionWorker(QtCore.QThread):
    progress = QtCore.Signal(int)           # 0-100
    finished = QtCore.Signal(list, list, list)   # planes, cylinders, spheres
    error = QtCore.Signal(str)

    def __init__(self, mesh_feature, params, parent=None):
        super().__init__(parent)
        self._mesh_feature = mesh_feature
        self._params = params

    def run(self):
        try:
            from MeshRANSAC.core.mesh_converter import freecad_mesh_to_pointcloud
            from MeshRANSAC.core.ransac_engine import detect_all

            voxel = self._params.get("voxel_size", 0.0) or None
            pcd = freecad_mesh_to_pointcloud(self._mesh_feature, voxel_size=voxel)

            total_steps = (
                self._params.get("max_planes", 10) * self._params.get("detect_planes", True)
                + self._params.get("max_cylinders", 10) * self._params.get("detect_cylinders", True)
                + self._params.get("max_spheres", 10) * self._params.get("detect_spheres", True)
            )
            total_steps = max(total_steps, 1)

            def _cb(step, _total):
                self.progress.emit(int(step / total_steps * 100))

            planes, cylinders, spheres = detect_all(pcd, self._params, progress_cb=_cb)
            self.progress.emit(100)
            self.finished.emit(planes, cylinders, spheres)

        except Exception as exc:  # noqa: BLE001
            self.error.emit(str(exc))


# ---------------------------------------------------------------------------
# Task panel
# ---------------------------------------------------------------------------

class DetectPanel:
    """
    FreeCAD task panel for RANSAC detection.

    Open with:
        FreeCADGui.Control.showTaskPanel(DetectPanel(...))
    """

    def __init__(self, detect_planes=True, detect_cylinders=True, detect_spheres=True):
        self._detect_planes    = detect_planes
        self._detect_cylinders = detect_cylinders
        self._detect_spheres   = detect_spheres
        self._worker = None

        self.form = self._build_ui()
        self._load_selected_mesh()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        widget = QtWidgets.QWidget()
        widget.setWindowTitle("MeshRANSAC — Detect Primitives")
        root = QtWidgets.QVBoxLayout(widget)
        root.setSpacing(8)

        # ── Mesh selection ──────────────────────────────────────────
        mesh_group = QtWidgets.QGroupBox("Source mesh")
        mg_layout = QtWidgets.QHBoxLayout(mesh_group)
        self._mesh_label = QtWidgets.QLabel("(none selected)")
        mg_layout.addWidget(self._mesh_label)
        root.addWidget(mesh_group)

        # ── Shape types ─────────────────────────────────────────────
        type_group = QtWidgets.QGroupBox("Shape types")
        tg_layout = QtWidgets.QVBoxLayout(type_group)
        self._chk_planes    = QtWidgets.QCheckBox("Planes")
        self._chk_cylinders = QtWidgets.QCheckBox("Cylinders")
        self._chk_spheres   = QtWidgets.QCheckBox("Spheres")
        self._chk_planes.setChecked(self._detect_planes)
        self._chk_cylinders.setChecked(self._detect_cylinders)
        self._chk_spheres.setChecked(self._detect_spheres)
        for chk in (self._chk_planes, self._chk_cylinders, self._chk_spheres):
            tg_layout.addWidget(chk)
        root.addWidget(type_group)

        # ── Parameters ──────────────────────────────────────────────
        param_group = QtWidgets.QGroupBox("Parameters")
        pg_layout = QtWidgets.QFormLayout(param_group)

        self._spin_dist = QtWidgets.QDoubleSpinBox()
        self._spin_dist.setRange(0.01, 100.0)
        self._spin_dist.setSingleStep(0.1)
        self._spin_dist.setValue(0.5)
        self._spin_dist.setSuffix(" mm")
        pg_layout.addRow("Distance threshold:", self._spin_dist)

        self._spin_min_inliers = QtWidgets.QSpinBox()
        self._spin_min_inliers.setRange(3, 100000)
        self._spin_min_inliers.setValue(50)
        pg_layout.addRow("Min inliers:", self._spin_min_inliers)

        self._spin_iterations = QtWidgets.QSpinBox()
        self._spin_iterations.setRange(10, 100000)
        self._spin_iterations.setValue(1000)
        pg_layout.addRow("Max iterations:", self._spin_iterations)

        self._spin_max_shapes = QtWidgets.QSpinBox()
        self._spin_max_shapes.setRange(1, 100)
        self._spin_max_shapes.setValue(10)
        pg_layout.addRow("Max shapes / type:", self._spin_max_shapes)

        self._spin_voxel = QtWidgets.QDoubleSpinBox()
        self._spin_voxel.setRange(0.0, 100.0)
        self._spin_voxel.setSingleStep(0.1)
        self._spin_voxel.setValue(0.0)
        self._spin_voxel.setSuffix(" mm  (0 = off)")
        pg_layout.addRow("Voxel downsample:", self._spin_voxel)

        root.addWidget(param_group)

        # ── Run button + progress ────────────────────────────────────
        self._btn_run = QtWidgets.QPushButton("▶  Run Detection")
        self._btn_run.setMinimumHeight(32)
        self._btn_run.clicked.connect(self._run)
        root.addWidget(self._btn_run)

        self._progress = QtWidgets.QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setVisible(False)
        root.addWidget(self._progress)

        # ── Results summary ──────────────────────────────────────────
        self._result_label = QtWidgets.QLabel()
        self._result_label.setWordWrap(True)
        root.addWidget(self._result_label)

        root.addStretch()
        return widget

    # ------------------------------------------------------------------
    # Mesh auto-detection
    # ------------------------------------------------------------------

    def _load_selected_mesh(self):
        sel = FreeCADGui.Selection.getSelection()
        if sel and sel[0].isDerivedFrom("Mesh::Feature"):
            self._mesh_feature = sel[0]
            self._mesh_label.setText(sel[0].Label)
        else:
            self._mesh_feature = None
            self._mesh_label.setText("(no mesh selected — select one and re-open)")

    # ------------------------------------------------------------------
    # Detection run
    # ------------------------------------------------------------------

    def _run(self):
        if self._mesh_feature is None:
            QtWidgets.QMessageBox.warning(
                self.form, "No mesh",
                "Select a Mesh::Feature object in the model tree first."
            )
            return

        if not any([self._chk_planes.isChecked(),
                    self._chk_cylinders.isChecked(),
                    self._chk_spheres.isChecked()]):
            QtWidgets.QMessageBox.warning(
                self.form, "No shape types",
                "Enable at least one shape type (planes, cylinders, or spheres)."
            )
            return

        params = {
            "detect_planes":    self._chk_planes.isChecked(),
            "detect_cylinders": self._chk_cylinders.isChecked(),
            "detect_spheres":   self._chk_spheres.isChecked(),
            "distance_threshold": self._spin_dist.value(),
            "min_inliers":      self._spin_min_inliers.value(),
            "num_iterations":   self._spin_iterations.value(),
            "max_planes":       self._spin_max_shapes.value(),
            "max_cylinders":    self._spin_max_shapes.value(),
            "max_spheres":      self._spin_max_shapes.value(),
            "voxel_size":       self._spin_voxel.value(),
        }

        self._btn_run.setEnabled(False)
        self._progress.setValue(0)
        self._progress.setVisible(True)
        self._result_label.setText("Running…")

        self._worker = _DetectionWorker(self._mesh_feature, params, self.form)
        self._worker.progress.connect(self._on_progress)
        self._worker.finished.connect(self._on_finished)
        self._worker.error.connect(self._on_error)
        self._worker.start()

    # ------------------------------------------------------------------
    # Worker callbacks (called on main thread via Qt signals)
    # ------------------------------------------------------------------

    @QtCore.Slot(int)
    def _on_progress(self, value):
        self._progress.setValue(value)

    @QtCore.Slot(list, list, list)
    def _on_finished(self, planes, cylinders, spheres):
        self._btn_run.setEnabled(True)

        doc = FreeCAD.ActiveDocument
        source_name = self._mesh_feature.Label

        try:
            from MeshRANSAC.core.result_manager import insert_results
            insert_results(doc, planes, cylinders, spheres, source_name)
        except ValueError as exc:
            self._result_label.setText(f"✗ {exc}")
            return

        lines = []
        for count, label in [
            (len(planes),    "plane"),
            (len(cylinders), "cylinder"),
            (len(spheres),   "sphere"),
        ]:
            if count:
                lines.append(f"✓ {count} {label}{'s' if count != 1 else ''} detected")
            else:
                lines.append(f"✗ No {label}s found")

        self._result_label.setText("\n".join(lines))
        FreeCADGui.Selection.clearSelection()

    @QtCore.Slot(str)
    def _on_error(self, msg):
        self._btn_run.setEnabled(True)
        self._progress.setVisible(False)
        self._result_label.setText(f"✗ Error: {msg}")
        from MeshRANSAC.utils.logger import error
        error(msg)

    # ------------------------------------------------------------------
    # Task panel protocol
    # ------------------------------------------------------------------

    def accept(self):
        if self._worker and self._worker.isRunning():
            self._worker.quit()
            self._worker.wait()
        FreeCADGui.Control.closeDialog()

    def reject(self):
        self.accept()
