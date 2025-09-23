#!/usr/bin/env python3
from pathlib import Path
import sys
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QTextEdit, QGroupBox, QFormLayout,
    QSlider, QCheckBox
)
from PyQt5.QtCore import Qt

import open3d as o3d

from utils import load_scan_csv_with_intensity


class ScanViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AUAS Scan Viewer")
        self.resize(1100, 800)

        # State
        self.o3d_vis = None
        self.current_path = None  # Path or None
        self.last_stats = {}
        self._last_mesh = None
        self._last_pcd = None
        self._wireframe_ls = None

        # Style
        self.setStyleSheet(self._stylesheet())

        # Root layout
        root = QVBoxLayout(self)

        # Header
        header = QHBoxLayout()
        title = QLabel("Scan Viewer")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch(1)
        root.addLayout(header)

        # Controls bar
        controls = QHBoxLayout()
        self.lbl = QLabel("No file loaded")
        self.lbl.setObjectName("pathLabel")
        btn_load = QPushButton("Load CSV…")
        btn_load.clicked.connect(self.load_csv)
        btn_load_ply = QPushButton("Load PLY…")
        btn_load_ply.clicked.connect(self.load_ply)
        btn_reset = QPushButton("Reset View")
        btn_reset.clicked.connect(self.reset_view)
        btn_clean = QPushButton("Clean && Align Preview")
        btn_clean.clicked.connect(self.clean_preview)
        btn_interactive = QPushButton("Interactive View")
        btn_interactive.setToolTip("Open Open3D interactive window (orbit/pan/zoom). Press H for help")
        btn_interactive.clicked.connect(self.open_interactive_view)
        controls.addWidget(self.lbl, stretch=1)
        controls.addWidget(btn_load)
        controls.addWidget(btn_load_ply)
        controls.addWidget(btn_reset)
        controls.addWidget(btn_clean)
        controls.addWidget(btn_interactive)
        root.addLayout(controls)

        # Info panels
        info_row = QHBoxLayout()
        stats_box = QGroupBox("Scan Stats")
        stats_form = QFormLayout()
        self.stat_points = QLabel("-")
        self.stat_intensity = QLabel("-")
        self.stat_bounds = QLabel("-")
        stats_form.addRow("Points:", self.stat_points)
        stats_form.addRow("Intensity min/max:", self.stat_intensity)
        stats_form.addRow("Bounds (x,y,z):", self.stat_bounds)
        stats_box.setLayout(stats_form)

        log_box = QGroupBox("Log")
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log)
        log_box.setLayout(log_layout)

        info_row.addWidget(stats_box, 1)
        info_row.addWidget(log_box, 2)
        root.addLayout(info_row)

        # Render options
        ro_box = QGroupBox("Render Options")
        ro_form = QFormLayout()
        self.chk_lighting = QCheckBox("Enable Lighting (meshes)")
        self.chk_lighting.setChecked(True)
        self.chk_lighting.stateChanged.connect(self._apply_render_options)
        self.chk_wireframe = QCheckBox("Wireframe Overlay (meshes)")
        self.chk_wireframe.setChecked(False)
        self.chk_wireframe.stateChanged.connect(self._update_wireframe)
        self.chk_darkbg = QCheckBox("Dark Background (PCD)")
        self.chk_darkbg.setChecked(True)
        self.chk_darkbg.stateChanged.connect(self._apply_render_options)
        self.sld_pointsize = QSlider(Qt.Horizontal)
        self.sld_pointsize.setMinimum(1)
        self.sld_pointsize.setMaximum(10)
        self.sld_pointsize.setValue(3)
        self.sld_pointsize.valueChanged.connect(self._apply_render_options)
        ro_form.addRow(self.chk_lighting)
        ro_form.addRow(self.chk_wireframe)
        ro_form.addRow(self.chk_darkbg)
        ro_form.addRow("Point Size (PCD)", self.sld_pointsize)
        ro_box.setLayout(ro_form)
        root.addWidget(ro_box)

        # The Open3D window is created on-demand.

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select CSV", str(Path.cwd()), "CSV Files (*.csv);;All Files (*)")
        if not path:
            return
        self.lbl.setText(str(path))
        self.current_path = Path(path)
        self._render_current()

    def reset_view(self):
        if not self.o3d_vis:
            self._log("Nothing to reset yet. Load a file first.")
            return
        try:
            if self._last_mesh is not None:
                self._fit_camera(self._last_mesh)
            elif self._last_pcd is not None:
                self._fit_camera(self._last_pcd)
            else:
                self._log("No geometry cached. Reloading current file…")
                if self.current_path and self.current_path.suffix.lower() == ".ply":
                    # Re-render PLY
                    try:
                        mesh = o3d.io.read_triangle_mesh(str(self.current_path))
                        if mesh is not None and len(mesh.triangles) > 0:
                            mesh.compute_vertex_normals()
                            self._render_mesh(mesh)
                        else:
                            pcd = o3d.io.read_point_cloud(str(self.current_path))
                            if pcd is not None and len(pcd.points) > 0:
                                self._render_pcd(pcd)
                    except Exception:
                        pass
                elif self.current_path and self.current_path.suffix.lower() == ".csv":
                    self._render_current()
            self.o3d_vis.poll_events()
            self.o3d_vis.update_renderer()
        except Exception as e:
            self._log(f"Reset view failed: {e}")

    def clean_preview(self):
        if not self.current_path:
            self._log("No file loaded")
            return
        # Run a light clean/align and render
        try:
            pts, cols, inten = load_scan_csv_with_intensity(self.current_path)
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
            pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
            pcd = pcd.voxel_down_sample(voxel_size=0.5)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            pcd.estimate_normals()
            self._ensure_vis()
            self.o3d_vis.clear_geometries()
            self._add_axis_for(pcd)
            self.o3d_vis.add_geometry(pcd)
            ro = self.o3d_vis.get_render_option()
            ro.point_size = float(self.sld_pointsize.value())
            ro.background_color = np.array([0.98, 0.98, 0.98])
            self._fit_camera(pcd)
            self.o3d_vis.poll_events()
            self.o3d_vis.update_renderer()
            self._update_stats(np.asarray(pcd.points), inten)
            self._log("Clean+align preview refreshed")
        except Exception as e:
            self._log(f"Clean preview failed: {e}")

    def _render_current(self):
        try:
            if not self.current_path:
                return
            pts, cols, inten = load_scan_csv_with_intensity(self.current_path)
            # If very large, downsample for preview to keep interaction smooth
            if pts.shape[0] > 400000:
                stride = max(1, pts.shape[0] // 300000)
                pts = pts[::stride]
                cols = cols[::stride]
                if inten is not None and inten.size:
                    inten = inten[::stride]
                self._log(f"Preview downsampled by stride {stride} (points={pts.shape[0]:,})")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
            pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
            self._last_pcd = pcd
            self._last_mesh = None
            self._wireframe_ls = None
            self._ensure_vis()
            self.o3d_vis.clear_geometries()
            self._add_axis_for(pcd)
            self.o3d_vis.add_geometry(pcd)
            ro = self.o3d_vis.get_render_option()
            ro.point_size = float(self.sld_pointsize.value())
            ro.background_color = np.array([0.06, 0.06, 0.08]) if self.chk_darkbg.isChecked() else np.array([0.98, 0.98, 0.98])
            self._fit_camera(pcd)
            self.o3d_vis.poll_events()
            self.o3d_vis.update_renderer()
            self._update_stats(pts, inten)
            if inten is not None and inten.size:
                self._log(f"Loaded CSV: points={pts.shape[0]:,}, intensity min/max={float(np.min(inten)):.2f}/{float(np.max(inten)):.2f}")
            else:
                self._log(f"Loaded CSV: points={pts.shape[0]:,}")
        except Exception as e:
            self._log(f"Load failed: {e}")

    def load_ply(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select PLY", str(Path.cwd()), "PLY Files (*.ply)")
        if not path:
            return
        self.lbl.setText(str(path))
        self.current_path = Path(path)
        try:
            mesh = o3d.io.read_triangle_mesh(path)
            if mesh is not None and len(mesh.triangles) > 0:
                mesh.compute_vertex_normals()
                self._render_mesh(mesh)
                return
        except Exception:
            pass
        try:
            pcd = o3d.io.read_point_cloud(path)
            if pcd is not None and len(pcd.points) > 0:
                self._render_pcd(pcd)
                return
        except Exception:
            pass
        self._log("Failed to load PLY as mesh or point cloud")

    def _render_mesh(self, mesh: o3d.geometry.TriangleMesh):
        self._ensure_vis()
        self.o3d_vis.clear_geometries()
        self._last_mesh = mesh
        self._last_pcd = None
        self._wireframe_ls = None
        # Color if no vertex colors
        if not mesh.has_vertex_colors():
            mesh.paint_uniform_color([0.8, 0.8, 0.85])
        self._add_axis_for(mesh)
        self.o3d_vis.add_geometry(mesh)
        ro = self.o3d_vis.get_render_option()
        ro.background_color = np.array([0.98, 0.98, 0.98])
        ro.light_on = self.chk_lighting.isChecked()
        self._fit_camera(mesh)
        self._update_wireframe()  # add overlay if checked
        self.o3d_vis.poll_events()
        self.o3d_vis.update_renderer()
        bbox = mesh.get_axis_aligned_bounding_box()
        self._update_stats(np.asarray(bbox.get_box_points()), None)
        self._log("Loaded mesh PLY")

    def _render_pcd(self, pcd: o3d.geometry.PointCloud):
        self._ensure_vis()
        self.o3d_vis.clear_geometries()
        self._last_pcd = pcd
        self._last_mesh = None
        self._wireframe_ls = None
        self._add_axis_for(pcd)
        self.o3d_vis.add_geometry(pcd)
        ro = self.o3d_vis.get_render_option()
        ro.point_size = float(self.sld_pointsize.value())
        ro.background_color = np.array([0.06, 0.06, 0.08]) if self.chk_darkbg.isChecked() else np.array([0.98, 0.98, 0.98])
        self._fit_camera(pcd)
        self.o3d_vis.poll_events()
        self.o3d_vis.update_renderer()
        self._update_stats(np.asarray(pcd.points), None)
        self._log("Loaded point cloud PLY")

    def _apply_render_options(self):
        if not self.o3d_vis:
            return
        ro = self.o3d_vis.get_render_option()
        ro.point_size = float(self.sld_pointsize.value())
        ro.light_on = self.chk_lighting.isChecked()
        # Background: dark for PCD if opted; keep light for mesh unless user insists
        if self._last_pcd is not None:
            ro.background_color = np.array([0.06, 0.06, 0.08]) if self.chk_darkbg.isChecked() else np.array([0.98, 0.98, 0.98])
        self.o3d_vis.poll_events()
        self.o3d_vis.update_renderer()

    def _update_wireframe(self):
        if not self.o3d_vis:
            return
        # Remove existing wireframe overlay
        if self._wireframe_ls is not None:
            try:
                self.o3d_vis.remove_geometry(self._wireframe_ls, reset_bounding_box=False)
            except Exception:
                pass
            self._wireframe_ls = None
        # Add new overlay if requested and we have a mesh
        if self.chk_wireframe.isChecked() and self._last_mesh is not None:
            try:
                ls = o3d.geometry.LineSet.create_from_triangle_mesh(self._last_mesh)
                if not ls.has_colors():
                    colors = np.tile(np.array([[0.1, 0.1, 0.1]]), (len(ls.lines), 1))
                    ls.colors = o3d.utility.Vector3dVector(colors)
                self._wireframe_ls = ls
                self.o3d_vis.add_geometry(ls)
            except Exception:
                pass
        self.o3d_vis.poll_events()
        self.o3d_vis.update_renderer()

    def open_interactive_view(self):
        geoms = []
        base = None
        if self._last_mesh is not None:
            base = self._last_mesh
        elif self._last_pcd is not None:
            base = self._last_pcd
        else:
            # Try to load from current path
            if self.current_path and self.current_path.suffix.lower() == ".ply":
                try:
                    mesh = o3d.io.read_triangle_mesh(str(self.current_path))
                    if mesh is not None and len(mesh.triangles) > 0:
                        mesh.compute_vertex_normals()
                        base = mesh
                    else:
                        pcd = o3d.io.read_point_cloud(str(self.current_path))
                        if pcd is not None and len(pcd.points) > 0:
                            base = pcd
                except Exception:
                    pass
        if base is None:
            self._log("Nothing to view interactively. Load a CSV or PLY first.")
            return
        try:
            bbox = base.get_axis_aligned_bounding_box()
            extent = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())
            size = float(extent * 0.1) if extent > 0 else 1.0
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
            geoms = [base, axis]
        except Exception:
            geoms = [base]
        self._log("Opening interactive window... (Left: rotate, Right: pan, Wheel: zoom, H: help)")
        try:
            o3d.visualization.draw_geometries(geoms, window_name="Interactive View", width=1024, height=768)
        except Exception as e:
            self._log(f"Interactive view failed: {e}")
            self._wireframe_ls = None
        if self.chk_wireframe.isChecked() and self._last_mesh is not None:
            try:
                self._wireframe_ls = o3d.geometry.LineSet.create_from_triangle_mesh(self._last_mesh)
                # darker lines for contrast
                if not self._wireframe_ls.has_colors():
                    colors = np.tile(np.array([[0.1, 0.1, 0.1]]), (len(self._wireframe_ls.lines), 1))
                    self._wireframe_ls.colors = o3d.utility.Vector3dVector(colors)
                self.o3d_vis.add_geometry(self._wireframe_ls)
                self.o3d_vis.poll_events()
                self.o3d_vis.update_renderer()
            except Exception:
                pass

    def _ensure_vis(self):
        if self.o3d_vis is None:
            self.o3d_vis = o3d.visualization.Visualizer()
            self.o3d_vis.create_window(window_name="Scan 3D", width=1024, height=768, visible=True)

    def _add_axis_for(self, geom: o3d.geometry.Geometry):
        try:
            bbox = geom.get_axis_aligned_bounding_box()
            extent = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())
            size = float(extent * 0.1) if extent > 0 else 1.0
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
            self.o3d_vis.add_geometry(axis)
        except Exception:
            pass

    def _fit_camera(self, geom: o3d.geometry.Geometry):
        try:
            vc = self.o3d_vis.get_view_control()
            bbox = geom.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            vc.set_lookat(center)
            vc.set_front([0, -1, 0])  # look along +Y
            vc.set_up([0, 0, 1])      # Z up
            vc.set_zoom(0.7)
        except Exception:
            pass

    def _update_stats(self, points, intensities):
        n = points.shape[0]
        self.stat_points.setText(f"{n:,}")
        if intensities is not None and intensities.size:
            imin, imax = float(np.min(intensities)), float(np.max(intensities))
            self.stat_intensity.setText(f"{imin:.3f} .. {imax:.3f}")
        else:
            self.stat_intensity.setText("-")
        if n:
            mins = points.min(axis=0)
            maxs = points.max(axis=0)
            self.stat_bounds.setText(f"min {mins.round(3)} / max {maxs.round(3)}")
        else:
            self.stat_bounds.setText("-")

    def _log(self, msg: str):
        self.log.append(msg)
        print(msg)

    def _stylesheet(self) -> str:
        return (
            "QWidget { background: #121212; color: #E0E0E0; }"
            "QGroupBox { border: 1px solid #2A2A2A; border-radius: 6px; margin-top: 12px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #90CAF9; }"
            "QPushButton { background: #1E88E5; color: white; padding: 6px 10px; border: none; border-radius: 4px; }"
            "QPushButton:hover { background: #1565C0; }"
            "#title { font-size: 22px; font-weight: 600; color: #90CAF9; }"
            "#pathLabel { color: #B0BEC5; }"
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ScanViewer()
    w.show()
    sys.exit(app.exec_())
