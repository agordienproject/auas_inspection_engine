#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import argparse
import re
import sys
from datetime import datetime
from typing import Iterable, List, Tuple

import numpy as np
import open3d as o3d

from utils import load_scan_csv, save_cleaned_csv

DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
TRACK_FILE = "last_processed_date.txt"


def list_date_folders(root: Path) -> List[Path]:
    insp = root / "inspections"
    if not insp.exists():
        return []
    return sorted([p for p in insp.iterdir() if p.is_dir() and DATE_RE.match(p.name)])


def read_last_date(root: Path) -> str | None:
    f = root / TRACK_FILE
    if not f.exists():
        return None
    try:
        s = f.read_text(encoding="utf-8").strip()
        return s or None
    except Exception:
        return None


def write_last_date(root: Path, date_str: str) -> None:
    (root).mkdir(parents=True, exist_ok=True)
    (root / TRACK_FILE).write_text(date_str + "\n", encoding="utf-8")


def should_process(date_name: str, last_date: str | None) -> bool:
    if last_date is None:
        return True
    # Strictly newer than last_date
    return date_name > last_date


def clean_and_align(points: np.ndarray) -> np.ndarray:
    """Basic cleaning + alignment pipeline with Open3D.

    Steps:
    - Voxel downsample
    - Statistical outlier removal
    - Estimate normals
    - PCA-based orientation alignment to principal axes
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

    # Voxel downsample (tune voxel_size as needed)
    pcd = pcd.voxel_down_sample(voxel_size=0.5)

    # Statistical outlier removal
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Estimate normals (needed for some algorithms and visualization)
    pcd.estimate_normals()

    # PCA alignment: align to principal axes
    pts = np.asarray(pcd.points)
    if pts.shape[0] >= 3:
        mean = pts.mean(axis=0)
        centered = pts - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        R = eigvecs[:, order]
        aligned = centered @ R
    else:
        aligned = np.asarray(pcd.points)

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(aligned.astype(np.float64))
    return np.asarray(pcd_out.points)


def process_csv_file(csv_path: Path) -> Tuple[Path, Path]:
    pts, cols = load_scan_csv(csv_path)
    cleaned = clean_and_align(pts)

    out_dir = csv_path.parent / "processed"
    out_dir.mkdir(exist_ok=True)

    # Save PCD
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cleaned.astype(np.float64))
    pcd_path = out_dir / (csv_path.stem + "_cleaned.pcd")
    o3d.io.write_point_cloud(str(pcd_path), pcd)

    # Save cleaned CSV (no intensity currently)
    csv_out = out_dir / (csv_path.stem + "_cleaned.csv")
    save_cleaned_csv(csv_out, cleaned, intensities=None)

    return pcd_path, csv_out


def process_root(root: Path) -> None:
    last_date = read_last_date(root)
    dates = list_date_folders(root)

    newest_seen: str | None = last_date

    for d in dates:
        if not should_process(d.name, last_date):
            continue
        # For each folder, search for CSV with the header
        for csv_path in d.rglob("*.csv"):
            try:
                with open(csv_path, "r") as f:
                    first = f.readline().strip()
                if first != "# x,z,intensity":
                    continue
                print(f"Processing {csv_path}")
                process_csv_file(csv_path)
            except Exception as e:
                print(f"Failed to process {csv_path}: {e}")
        # Track newest date name processed
        if newest_seen is None or d.name > newest_seen:
            newest_seen = d.name

    if newest_seen and newest_seen != last_date:
        write_last_date(root, newest_seen)
        print(f"Updated {TRACK_FILE} to {newest_seen}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Process new inspection scans from FTP root")
    parser.add_argument("--root", type=str, required=True, help="FTP root path containing 'inspections' folder")
    args = parser.parse_args(argv)

    root = Path(args.root)
    if not root.exists():
        print(f"Root path not found: {root}")
        return 1

    process_root(root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
