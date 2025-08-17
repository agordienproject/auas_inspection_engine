#!/usr/bin/env python3
"""
FTP Server launcher script for AUAS Inspection Engine
Installs dependencies and starts the server
"""
import subprocess
import sys
import os

def install_dependencies():
    """Install required FTP server dependencies"""
    print("📦 Installing FTP server dependencies...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", 
            "pyftpdlib", "--quiet"
        ])
        print("✅ Dependencies installed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False

def start_server():
    """Start the FTP server"""
    try:
        from server import main
        return main()
    except ImportError:
        print("❌ Server module not found")
        return 1
    except Exception as e:
        print(f"❌ Server error: {e}")
        return 1

if __name__ == "__main__":
    print("🏗️ AUAS FTP Server Launcher")
    print("=" * 40)
    
    # Install dependencies first
    if not install_dependencies():
        sys.exit(1)
    
    # Start server
    sys.exit(start_server())
