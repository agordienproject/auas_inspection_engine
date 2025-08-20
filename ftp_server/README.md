# AUAS FTP Server

Simple Python FTP server for the AUAS Inspection Engine project.

## Features

- ✅ Simple setup and configuration
- ✅ User authentication (inspection_engine/admin)
- ✅ Anonymous read-only access
- ✅ No complex TLS configuration needed
- ✅ Perfect for development and testing

## Quick Start

1. **Start the server:**
```bash
cd ftp_server
python start_server.py
```

2. **Connect to server:**
- **Host:** 127.0.0.1
- **Port:** 21
- **Username:** inspection_engine
- **Password:** admin
- **Base Path:** C:\Users\Agordien\Documents\projects\AUAS\FTP

## Configuration

The server is configured with:
- **Main user:** `inspection_engine` with full permissions
- **Anonymous access:** Read-only access enabled
- **Base directory:** `C:\Users\Agordien\Documents\projects\AUAS\FTP`
- **Inspections folder:** `C:\Users\Agordien\Documents\projects\AUAS\FTP\inspections`

## Files

- `server.py` - Main FTP server implementation
- `start_server.py` - Launcher script with dependency installation
- `README.md` - This file

## Testing

You can test the server with any FTP client:
```bash
# Command line
ftp 127.0.0.1

# Python ftplib
import ftplib
ftp = ftplib.FTP('127.0.0.1')
ftp.login('inspection_engine', 'admin')
```

## Stopping

Press `Ctrl+C` in the terminal where the server is running.
