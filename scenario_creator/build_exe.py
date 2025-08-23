#!/usr/bin/env python3
"""
Build script to create Scenario Creator executable using PyInstaller
"""
import sys
import subprocess
import os
from pathlib import Path

def install_build_dependencies():
    """Install PyInstaller and dependencies"""
    print("📦 Installing build dependencies...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", 
            "pyinstaller", "PyQt5", "--quiet"
        ])
        print("✅ Build dependencies installed")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False

def build_executable():
    """Build the Scenario Creator executable"""
    print("🔨 Building Scenario Creator executable...")
    
    script_dir = Path(__file__).parent
    main_script = script_dir / "scenario_creator.py"
    
    if not main_script.exists():
        print(f"❌ Main script not found: {main_script}")
        return False
    
    # PyInstaller command
    dist_path = script_dir / "dist"
    build_path = script_dir / "build"
    spec_path = script_dir
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--onefile",                    # Single executable file
        "--windowed",                   # No console window
        "--name", "Scenario_Creator",  # Executable name
        "--distpath", str(dist_path),   # Output directory
        "--workpath", str(build_path),  # Build directory
        "--specpath", str(spec_path),   # Spec file directory
        str(main_script)
    ]
    try:
        subprocess.check_call(cmd)
        print("✅ Executable built successfully!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Build failed: {e}")
        return False

def main():
    if not install_build_dependencies():
        return
    build_executable()

if __name__ == "__main__":
    main()
