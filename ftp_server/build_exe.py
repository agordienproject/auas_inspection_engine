#!/usr/bin/env python3
"""
Build script to create FTP Server executable using PyInstaller
"""
import sys
import subprocess
from pathlib import Path

def install_build_dependencies():
    """Install PyInstaller and dependencies"""
    print("Installing build dependencies...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", 
            "-r", "requirements.txt"
        ])
        print("✅ Build dependencies installed")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to install dependencies: {e}")
        return False

def build_executable():
    """Build the FTP server executable"""
    print("Building FTP Server executable...")
    
    script_dir = Path(__file__).parent
    main_script = script_dir / "server.py"
    
    if not main_script.exists():
        print(f"Main script not found: {main_script}")
        return False
    
    # PyInstaller command
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--onefile",                    # Single executable file
        "--console",                    # Console application
        "--name", "AUAS_FTP_Server",   # Executable name
        "--icon", "NONE",              # No icon for now
        "--distpath", str(script_dir / "dist"),
        "--workpath", str(script_dir / "build"),
        "--specpath", str(script_dir),
        str(main_script)
    ]
    
    try:
        print(f"Running: {' '.join(cmd)}")
        subprocess.check_call(cmd, cwd=script_dir)
        
        exe_path = script_dir / "dist" / "AUAS_FTP_Server.exe"
        if exe_path.exists():
            print("Executable created successfully!")
            print(f"Location: {exe_path}")
            print(f"Size: {exe_path.stat().st_size / 1024 / 1024:.1f} MB")
            return True
        else:
            print("Executable not found after build")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"Build failed: {e}")
        return False

def create_readme():
    """Create a README for the executable"""
    readme_content = """# AUAS FTP Server Executable

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
"""
    
    script_dir = Path(__file__).parent
    readme_path = script_dir / "dist" / "README.txt"
    
    try:
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(readme_content)
        print(f"README created: {readme_path}")
        return True
    except Exception as e:
        print(f"Failed to create README: {e}")
        return False

def main():
    """Main build process"""
    print("AUAS FTP Server Build Process")
    print("=" * 50)
    
    # Step 1: Install dependencies
    if not install_build_dependencies():
        return 1
    
    # Step 2: Build executable
    if not build_executable():
        return 1
    
    # Step 3: Create documentation
    create_readme()
    
    print("\n" + "=" * 50)
    print("Build completed successfully!")
    print("Check the 'dist' folder for your executable")
    print("Double-click AUAS_FTP_Server.exe to start!")
    print("=" * 50)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
