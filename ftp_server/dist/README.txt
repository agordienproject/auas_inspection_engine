# AUAS FTP Server Executable

## Quick Start
1. Double-click `AUAS_FTP_Server.exe` to start the server
2. The server will create a `FTP_Data` folder next to the executable
3. Connect using any FTP client with these credentials:
   - Host: 127.0.0.1 (localhost)
   - Port: 21
   - Username: inspection_engine
   - Password: admin

## Features
- ✅ Simple double-click startup
- ✅ Automatic folder creation
- ✅ User authentication (inspection_engine/admin)
- ✅ Anonymous read-only access
- ✅ Console logging
- ✅ Graceful shutdown with Ctrl+C

## File Structure
After first run, you'll see:
```
AUAS_FTP_Server.exe
FTP_Data/
├── inspections/     (for inspection uploads)
└── server_info.txt  (server information)
```

## Connection Details
- **FTP URL**: ftp://127.0.0.1:21
- **Username**: inspection_engine
- **Password**: admin
- **Permissions**: Full read/write access
- **Anonymous**: Read-only access enabled

## Troubleshooting
- If port 21 is in use, close other FTP servers first
- Run as Administrator if you encounter permission issues
- Check Windows Firewall settings if external connections fail

## Integration
The inspection engine can upload files using these settings:
- Host: localhost (127.0.0.1)
- Port: 21
- Username: inspection_engine
- Password: admin
