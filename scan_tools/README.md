# AUAS Scan Tools

Interactive viewer and exech processor for AUAS scan data (PLY/CSV).

Contents
- scan_viewer.py — GUI viewer with “Clean & Align Preview” and FTP upload of cleaned PLY to piece_reference
- enhanced_scan_processor.py — exech processor that scans FTP inspection folders, cleans PLYs using the same pipeline as the viewer, and writes cleaned PLYs
- utils.py — CSV parsing and helpers
- ply_viewer.exe — one-click launcher for the viewer
- ply_processor.exe — one-click launcher for the processor
- .env.template — environment template (copied to .env by installer)
- requirements.txt — Python dependencies for this folder only

## Quick start

1) Dependencies

```powershell
python -m pip install -r .\scan_tools\requirements.txt
```

2) Environment file

Copy .env.template to .env and keep the values (viewer/processor read FTP and local paths from here):

```powershell
copy .\scan_tools\.env.template .\scan_tools\.env
```

3) Run

- Viewer (recommended: double-click ply_viewer.exe)

```powershell
.\u005cscan_tools\ply_viewer.exe
```

- Processor (recommended: double-click ply_processor.exe)

```powershell
.\u005cscan_tools\ply_processor.exe
```

## Viewer (scan_viewer.py)

What it does
- Loads CSV or PLY
- Clean & Align Preview: voxel downsample, outlier removal, plane removal/keep-above, largest cluster, optional Poisson mesh
- Save Cleaned PLY: uploads cleaned point cloud to the FTP piece_reference/<reference> folder and writes a parameters txt next to it

Tips
- Use Interactive View for orbit/pan/zoom (Open3D window)
- Render options: lighting (meshes), wireframe overlay, background, point size

## exech Processor (enhanced_scan_processor.py)

What it does
- Scans LOCAL_FTP_ROOT/inspections for date folders
- Cleans each PLY using the same pipeline and parameters as the viewer
- Saves cleaned PLY next to the original and generates a scan_analysis_report.txt

Config (from .env)
- LOCAL_FTP_ROOT — base folder containing inspections and piece_reference

Notes
- Cleaned PLYs are saved as binary PLY for compact size
- Tracking file last_processed_date.txt is written in the FTP root to avoid reprocessing old days

## CSV format (utils)

- Expected header: `# x,z,intensity`
- Rows: `x,z,intensity`
- utils.load_scan_csv and utils.load_scan_csv_with_intensity convert to XYZ with Y=0

## Troubleshooting

- If PyQt/Open3D fail to import, ensure you used the requirements.txt in this folder
- If the viewer’s FTP upload fails, verify FTP_HOST, FTP_PORT, FTP_USERNAME, FTP_PASSWORD in .env
- For processor runs without .env, pass --root to point at the local FTP root
