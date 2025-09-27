#!/usr/bin/env python3
from pathlib import Path
import sys
import numpy as np
import os
import ftplib
import io
from datetime import datetime

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QTextEdit, QGroupBox, QFormLayout,
    QSlider, QCheckBox, QLineEdit
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
        btn_save_cleaned = QPushButton("Save Cleaned PLY")
        btn_save_cleaned.setToolTip("Save cleaned point cloud to FTP server piece_reference folder")
        btn_save_cleaned.clicked.connect(self.save_cleaned_ply)
        controls.addWidget(self.lbl, stretch=1)
        controls.addWidget(btn_load)
        controls.addWidget(btn_load_ply)
        controls.addWidget(btn_reset)
        controls.addWidget(btn_clean)
        controls.addWidget(btn_interactive)
        controls.addWidget(btn_save_cleaned)
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

        # Clean options
        clean_box = QGroupBox("Clean Options")
        clean_form = QFormLayout()
        self.chk_remove_plane = QCheckBox("Detect & Remove Ground Plane")
        self.chk_remove_plane.setChecked(True)
        self.chk_keep_above = QCheckBox("Keep Only Above Plane")
        self.chk_keep_above.setChecked(True)
        self.chk_largest_cluster = QCheckBox("Keep Largest Cluster")
        self.chk_largest_cluster.setChecked(True)
        self.chk_poisson = QCheckBox("Reconstruct Surface (Poisson)")
        self.chk_poisson.setToolTip("Estimate normals, run Poisson reconstruction, and show mesh")
        self.chk_poisson.setChecked(False)
        self.txt_voxel = QLineEdit("0.4")
        self.txt_plane_dist = QLineEdit("1.8")
        self.txt_outlier_std = QLineEdit("2.8")
        self.txt_dbscan_eps = QLineEdit("2.8")
        self.txt_dbscan_min = QLineEdit("6")
        self.txt_poisson_depth = QLineEdit("9")
        clean_form.addRow(self.chk_remove_plane)
        clean_form.addRow(self.chk_keep_above)
        clean_form.addRow(self.chk_largest_cluster)
        clean_form.addRow(self.chk_poisson)
        clean_form.addRow("Voxel (mm)", self.txt_voxel)
        clean_form.addRow("Plane dist (mm)", self.txt_plane_dist)
        clean_form.addRow("Outlier std_ratio", self.txt_outlier_std)
        clean_form.addRow("DBSCAN eps (mm)", self.txt_dbscan_eps)
        clean_form.addRow("DBSCAN min_points", self.txt_dbscan_min)
        clean_form.addRow("Poisson depth", self.txt_poisson_depth)
        clean_box.setLayout(clean_form)
        root.addWidget(clean_box)

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
        if not self.current_path and self._last_pcd is None and self._last_mesh is None:
            self._log("No file loaded")
            return
        try:
            # Build a point cloud from current data (CSV/PLY PCD/Mesh)
            base_pcd = None
            base_inten = None
            if self._last_pcd is not None:
                base_pcd = self._last_pcd
            elif self._last_mesh is not None:
                try:
                    verts = np.asarray(self._last_mesh.vertices)
                    base_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(verts.astype(np.float64)))
                except Exception:
                    pass
            elif self.current_path:
                suffix = self.current_path.suffix.lower()
                if suffix == ".csv":
                    pts, cols, inten = load_scan_csv_with_intensity(self.current_path)
                    base_inten = inten
                    base_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
                    base_pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
                elif suffix == ".ply":
                    try:
                        pcd = o3d.io.read_point_cloud(str(self.current_path))
                        if pcd is not None and len(pcd.points) > 0:
                            base_pcd = pcd
                        else:
                            mesh = o3d.io.read_triangle_mesh(str(self.current_path))
                            if mesh is not None and len(mesh.vertices) > 0:
                                verts = np.asarray(mesh.vertices)
                                base_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(verts.astype(np.float64)))
                    except Exception:
                        pass

            if base_pcd is None:
                self._log("Unable to prepare point cloud for cleaning.")
                return

            # Parameters
            voxel = self._get_float(self.txt_voxel, 0.5)
            plane_dist = self._get_float(self.txt_plane_dist, 1.0)
            out_std = self._get_float(self.txt_outlier_std, 2.0)
            db_eps = self._get_float(self.txt_dbscan_eps, 2.0)
            try:
                db_min = int(float(self.txt_dbscan_min.text().strip()))
            except Exception:
                db_min = 10
            depth = int(self._get_float(self.txt_poisson_depth, 9))

            pcd = base_pcd.voxel_down_sample(max(1e-6, float(voxel))) if voxel > 1e-6 else base_pcd
            if len(pcd.points) == 0:
                self._log("No points after voxel downsample.")
                return
            # Remove sparse outliers first
            try:
                pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=float(out_std))
            except Exception:
                pass

            # Estimate normals for later steps
            try:
                pcd.estimate_normals()
            except Exception:
                pass

            # Plane segmentation and above-plane filtering
            if self.chk_remove_plane.isChecked() or self.chk_keep_above.isChecked():
                try:
                    model, inliers = pcd.segment_plane(distance_threshold=float(plane_dist), ransac_n=3, num_iterations=500)
                    a, b, c, d = model
                    n = np.array([a, b, c], dtype=np.float64)
                    nn = np.linalg.norm(n)
                    if nn > 0:
                        n = n / nn
                        d = d / nn
                    # Orient normal upwards (Z positive)
                    if n[2] < 0:
                        n = -n
                        d = -d
                    pts_np = np.asarray(pcd.points)
                    if self.chk_remove_plane.isChecked():
                        mask = np.ones((pts_np.shape[0],), dtype=bool)
                        mask[np.asarray(inliers, dtype=int)] = False
                        pts_np = pts_np[mask]
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)[mask]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                    # Keep only above plane if requested
                    if self.chk_keep_above.isChecked() and pts_np.shape[0] > 0:
                        signed = np.dot(pts_np, n) + d
                        keep = signed > 0.0
                        pts_np = pts_np[keep]
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)
                            cols_np = cols_np[keep]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                except Exception as e:
                    self._log(f"Plane segmentation skipped: {e}")

            # Largest cluster selection
            if self.chk_largest_cluster.isChecked():
                try:
                    labels = np.array(pcd.cluster_dbscan(eps=float(db_eps), min_points=int(db_min), print_progress=False))
                    if labels.size and labels.max() >= 0:
                        counts = np.bincount(labels[labels >= 0])
                        largest_label = counts.argmax()
                        mask = labels == largest_label
                        pts_np = np.asarray(pcd.points)[mask]
                        if pcd.has_colors():
                            cols_np = np.asarray(pcd.colors)[mask]
                        else:
                            cols_np = None
                        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_np))
                        if cols_np is not None:
                            pcd.colors = o3d.utility.Vector3dVector(cols_np)
                except Exception as e:
                    self._log(f"Clustering skipped: {e}")

            # Render mesh if Poisson requested
            if self.chk_poisson.isChecked():
                try:
                    pcd.estimate_normals()
                    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=int(depth))
                    bbox = pcd.get_axis_aligned_bounding_box()
                    mesh = mesh.crop(bbox)
                    if not mesh.has_vertex_colors():
                        mesh.paint_uniform_color([0.8, 0.8, 0.85])
                    mesh.compute_vertex_normals()
                    self._render_mesh(mesh)
                    self._log("Clean preview: reconstructed surface (Poisson)")
                    return
                except Exception as e:
                    self._log(f"Poisson reconstruction failed: {e}")

            # Otherwise render cleaned point cloud
            self._render_pcd(pcd)
            self._update_stats(np.asarray(pcd.points), base_inten)
            self._log("Clean preview refreshed")
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

    def _get_float(self, qle: QLineEdit, default: float) -> float:
        try:
            return float(qle.text().strip())
        except Exception:
            return float(default)

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

    def save_cleaned_ply(self):
        """Save the cleaned point cloud to real FTP server piece_reference folder"""
        if self._last_pcd is None:
            self._log("❌ No cleaned point cloud available. Run 'Clean & Align Preview' first.")
            return
            
        try:
            # Find reference.txt in scenario_inspector output folders
            piece_reference = self._find_latest_piece_reference()
            if not piece_reference:
                self._log("❌ No reference.txt found in recent inspections. Run an inspection with scan data first.")
                return
            
            # Get FTP configuration from environment
            ftp_host = os.getenv('FTP_HOST', 'localhost')
            ftp_port = int(os.getenv('FTP_PORT', 21))
            ftp_username = os.getenv('FTP_USERNAME', 'inspection_engine')
            ftp_password = os.getenv('FTP_PASSWORD', 'admin')
            ftp_base_path = os.getenv('FTP_BASE_PATH', '/')
            
            # Generate timestamp and filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cleaned_filename = f"cleaned_scan_{timestamp}.ply"
            params_filename = f"cleaning_params_{timestamp}.txt"
            
            # Connect to FTP server
            self._log(f"🔗 Connecting to FTP server {ftp_host}:{ftp_port}...")
            ftp = ftplib.FTP()
            ftp.connect(ftp_host, ftp_port)
            ftp.login(ftp_username, ftp_password)
            
            # Navigate to piece_reference folder
            ftp_piece_path = f"{ftp_base_path.rstrip('/')}/piece_reference/{piece_reference}"
            self._log(f"📁 Creating FTP directory: {ftp_piece_path}")
            
            # Create directory structure (handle existing directories)
            try:
                ftp.mkd("piece_reference")
            except ftplib.error_perm:
                pass  # Directory already exists
            
            try:
                ftp.mkd(f"piece_reference/{piece_reference}")
            except ftplib.error_perm:
                pass  # Directory already exists
            
            # Change to the piece directory
            ftp.cwd(f"piece_reference/{piece_reference}")
            
            # Save PLY file to temporary location first
            temp_ply_path = Path(f"temp_{cleaned_filename}")
            success = o3d.io.write_point_cloud(str(temp_ply_path), self._last_pcd)
            
            if not success:
                self._log("❌ Failed to save temporary PLY file")
                ftp.quit()
                return
            
            # Upload PLY file to FTP
            self._log(f"⬆️ Uploading {cleaned_filename}...")
            with open(temp_ply_path, 'rb') as f:
                ftp.storbinary(f'STOR {cleaned_filename}', f)
            
            # Create and upload parameters file
            params_content = f"""Cleaning Parameters Used:
Voxel Size (mm): {self.txt_voxel.text()}
Plane Distance (mm): {self.txt_plane_dist.text()}
Outlier Std Ratio: {self.txt_outlier_std.text()}
DBSCAN Eps (mm): {self.txt_dbscan_eps.text()}
DBSCAN Min Points: {self.txt_dbscan_min.text()}
Poisson Depth: {self.txt_poisson_depth.text()}

Options:
Remove Ground Plane: {self.chk_remove_plane.isChecked()}
Keep Above Plane: {self.chk_keep_above.isChecked()}
Keep Largest Cluster: {self.chk_largest_cluster.isChecked()}
Poisson Reconstruction: {self.chk_poisson.isChecked()}

Original PLY: {self.current_path if self.current_path else 'N/A'}
Cleaned PLY: {cleaned_filename}
Timestamp: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
FTP Path: {ftp_piece_path}/{cleaned_filename}
"""
            
            # Upload parameters file
            self._log(f"⬆️ Uploading {params_filename}...")
            params_io = io.BytesIO(params_content.encode('utf-8'))
            ftp.storbinary(f'STOR {params_filename}', params_io)
            
            # Close FTP connection
            ftp.quit()
            
            # Clean up temporary file
            temp_ply_path.unlink()
            
            self._log(f"✅ Cleaned PLY uploaded to FTP: {ftp_piece_path}/{cleaned_filename}")
            self._log(f"📁 Piece reference: {piece_reference}")
            self._log(f"📝 Parameters uploaded: {ftp_piece_path}/{params_filename}")
                
        except ftplib.all_errors as e:
            self._log(f"❌ FTP Error: {e}")
        except Exception as e:
            self._log(f"❌ Error saving cleaned PLY: {e}")

    def _find_latest_piece_reference(self) -> str | None:
        """Find the most recent piece reference from scenario_inspector output folders"""
        try:
            # Path to scenario_inspector output directory
            output_root = Path("../scenario_inspector/output")
            if not output_root.exists():
                return None
            
            # Look for reference.txt files in all inspection folders, sorted by date (newest first)
            reference_files = []
            
            for date_folder in output_root.iterdir():
                if date_folder.is_dir() and date_folder.name.count('-') == 2:  # Date format YYYY-MM-DD
                    for inspection_folder in date_folder.iterdir():
                        if inspection_folder.is_dir():
                            ref_file = inspection_folder / "reference.txt"
                            if ref_file.exists():
                                try:
                                    # Get modification time and content
                                    mtime = ref_file.stat().st_mtime
                                    content = ref_file.read_text().strip()
                                    if content:
                                        reference_files.append((mtime, content, ref_file))
                                except Exception:
                                    continue
            
            if not reference_files:
                return None
            
            # Sort by modification time (newest first) and return the most recent reference
            reference_files.sort(key=lambda x: x[0], reverse=True)
            latest_ref = reference_files[0][1]
            latest_file = reference_files[0][2]
            
            self._log(f"📋 Found latest piece reference: {latest_ref}")
            self._log(f"📂 From: {latest_file}")
            
            return latest_ref
            
        except Exception as e:
            self._log(f"❌ Error finding piece reference: {e}")
            return None

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
