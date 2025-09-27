from __future__ import annotations
import csv
from pathlib import Path
from typing import Tuple
import numpy as np

HEADER = "# x,z,intensity"


def load_scan_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load scan CSV expecting first line to equal HEADER.

    Returns (points, colors):
    - points: (N,3) float32 array [x, y, z]. As input has only x and z,
      we set y=0 for a strip scan; later alignment may populate Y from sequences.
    - colors: (N,3) float32 array in [0,1] from intensity (grayscale)
    """
    with open(path, "r", newline="") as f:
        first = f.readline().strip()
        if first != HEADER:
            raise ValueError(f"Invalid header in {path.name}: '{first}' (expected '{HEADER}')")

        reader = csv.reader(f)
        xs, zs, intensities = [], [], []
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[0])
                z = float(row[1])
                inten = float(row[2]) if len(row) > 2 else 0.0
            except (ValueError, IndexError):
                continue
            xs.append(x)
            zs.append(z)
            intensities.append(inten)

    x = np.asarray(xs, dtype=np.float32)
    z = np.asarray(zs, dtype=np.float32)
    inten = np.asarray(intensities, dtype=np.float32)

    # Normalize intensity to [0,1] safely
    if inten.size:
        imin, imax = np.min(inten), np.max(inten)
        if imax > imin:
            norm = (inten - imin) / (imax - imin)
        else:
            norm = np.zeros_like(inten)
    else:
        norm = np.zeros_like(inten)

    # points as (x, y, z) with y=0 placeholder
    pts = np.stack([x, np.zeros_like(x), z], axis=1)
    cols = np.stack([norm, norm, norm], axis=1)
    return pts, cols


def load_scan_csv_with_intensity(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load scan CSV and also return raw intensity array.

    Accepts header line that starts with '#' and contains 'x' and 'z'.
    Supports comma, semicolon, or whitespace delimiters.
    Returns (points, colors, intensities).
    """
    # Fast path: numpy genfromtxt with comma delimiter
    def _fast_load(delim: str | None):
        try:
            arr = np.genfromtxt(
                str(path), delimiter=delim, comments="#", dtype=np.float32, usecols=(0, 1, 2)
            )
            if arr.ndim == 1:
                if arr.size == 0:
                    return None
                arr = arr.reshape(1, -1)
            if arr.shape[1] < 2:
                return None
            # If intensity missing, fill zeros
            if arr.shape[1] == 2:
                arr = np.column_stack([arr, np.zeros((arr.shape[0],), dtype=np.float32)])
            return arr
        except Exception:
            return None

    arr = _fast_load(",")
    if arr is None:
        # Try semicolons by pre-reading and replacing
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            text = None
        if text:
            text2 = text.replace(";", ",")
            # Save to temporary numpy buffer
            arr = np.genfromtxt(
                (line for line in text2.splitlines()),
                delimiter=",",
                comments="#",
                dtype=np.float32,
                usecols=(0, 1, 2),
            )
            if arr.ndim == 1:
                if arr.size == 0:
                    arr = None
                else:
                    arr = arr.reshape(1, -1)

    if arr is None:
        # Last resort: whitespace split and manual parse
        xs: list[float] = []
        zs: list[float] = []
        intensities: list[float] = []
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for i, line in enumerate(f):
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                parts = [p for p in s.replace(";", " ").replace(",", " ").split() if p]
                if len(parts) < 2:
                    continue
                try:
                    x = float(parts[0])
                    z = float(parts[1])
                    inten = float(parts[2]) if len(parts) > 2 else 0.0
                except ValueError:
                    continue
                xs.append(x)
                zs.append(z)
                intensities.append(inten)
        if not xs:
            raise ValueError(f"No valid data rows parsed from {path}")
        x = np.asarray(xs, dtype=np.float32)
        z = np.asarray(zs, dtype=np.float32)
        inten = np.asarray(intensities, dtype=np.float32)
    else:
        x = arr[:, 0].astype(np.float32)
        z = arr[:, 1].astype(np.float32)
        inten = arr[:, 2].astype(np.float32)

    # Filter out invalid rows (NaN/Inf)
    mask = np.isfinite(x) & np.isfinite(z) & np.isfinite(inten)
    if not np.all(mask):
        x = x[mask]
        z = z[mask]
        inten = inten[mask]
    if x.size == 0:
        raise ValueError(f"No valid numeric rows found in {path}")

    # Normalize intensity to [0,1]
    if inten.size:
        imin, imax = float(np.min(inten)), float(np.max(inten))
        if imax > imin:
            norm = (inten - imin) / (imax - imin)
        else:
            norm = np.zeros_like(inten)
    else:
        norm = np.zeros_like(inten)

    pts = np.stack([x, np.zeros_like(x), z], axis=1)
    cols = np.stack([norm, norm, norm], axis=1)
    return pts, cols, inten


def save_cleaned_csv(path: Path, points: np.ndarray, intensities: np.ndarray | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z", "intensity"])  # cleaned format
        n = points.shape[0]
        if intensities is None:
            for i in range(n):
                writer.writerow([float(points[i,0]), float(points[i,1]), float(points[i,2]), 0.0])
        else:
            for i in range(n):
                writer.writerow([float(points[i,0]), float(points[i,1]), float(points[i,2]), float(intensities[i])])
