# AUAS Scan Tools

Utilities to visualize and process scanner CSV outputs.

- scan_viewer.py: Minimal GUI to load a CSV scan (`# x,z,intensity`) and render as a 3D point cloud using Open3D.
- scan_processor.py: Batch processor that scans an FTP root folder for new inspection dates and runs a clean+align pipeline on each CSV.

## Install dependencies

```powershell
python -m pip install -r .\scan_tools\requirements.txt
```

## Run viewer

```powershell
python .\scan_tools\scan_viewer.py
```

## Run processor (local FTP root path)

```powershell
python .\scan_tools\scan_processor.py --root "C:\\path\\to\\FTP"
```

- The processor expects an `inspections` folder inside the given root.
- It maintains `last_processed_date.txt` at the root. If missing/empty, all dates are processed.

## CSV format

First line must be: `# x,z,intensity`
Subsequent lines: `x,z,intensity` values separated by commas.

## Outputs

Processed artifacts are saved next to the source CSV under a `processed` folder:
- `<name>_cleaned.pcd` – cleaned and aligned point cloud
- `<name>_cleaned.csv` – cleaned points (x,y,z,intensity)
