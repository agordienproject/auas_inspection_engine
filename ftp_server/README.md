# AUAS FTP Server

Simple, configurable FTP server used by the AUAS Inspection Engine.

## What’s new

- Reads configuration from a .env file (project root or current folder)
- Creates a piece_reference folder (used by the viewer to upload cleaned PLYs)
- Initializes last_processed_date.txt at the base path (used by the processor)
- Keeps anonymous read-only access for quick testing

## Quick start (Python)

```powershell
cd ftp_server
python server.py
```

Defaults if no .env is present:
- Host: 0.0.0.0 (connect via 127.0.0.1 locally)
- Port: 21
- Base Path: C:\Users\Agordien\Documents\projects\AUAS\FTP

Credentials:
- Username: inspection_engine
- Password: admin
- Anonymous: enabled (read-only)

## Configuration via .env

Place a .env in the project root or in ftp_server/ with any of:

```
FTP_HOST=0.0.0.0
FTP_PORT=21
FTP_BASE_PATH=C:\\Users\\Agordien\\Documents\\projects\\AUAS\\FTP
```

On startup the server ensures these locations exist under FTP_BASE_PATH:
- inspections/ — upload area for raw inspection data
- piece_reference/ — destination for cleaned PLYs by piece reference (from the viewer)
- last_processed_date.txt — tracking file used by the batch processor
- server_info.txt — basic server info

## Integration with scan_tools

- Viewer (scan_tools/scan_viewer.py) uploads cleaned PLYs to: piece_reference/<reference>
- Processor (scan_tools/enhanced_scan_processor.py) reads LOCAL_FTP_ROOT from scan_tools/.env and expects the same directory tree created by the FTP server

## Testing

Any FTP client works. Example with Python:

```python
import ftplib
ftp = ftplib.FTP('127.0.0.1')
ftp.login('inspection_engine', 'admin')
print(ftp.nlst())
ftp.quit()
```

## Stopping

Press Ctrl+C in the terminal where the server is running.
