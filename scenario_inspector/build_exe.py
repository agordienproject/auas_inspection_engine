#!/usr/bin/env python3
"""
Build script to create Scenario Inspector executable using PyInstaller
"""
import sys
import subprocess
import os
from pathlib import Path

def install_build_dependencies():
    print("📦 Installing build dependencies...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", 
            "pyinstaller", "--quiet"
        ])
        print("✅ Build dependencies installed")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False

def build_executable():
    print("🔨 Building Scenario Inspector executable...")
    script_dir = Path(__file__).parent
    main_script = script_dir / "src" / "main.py"
    dist_path = script_dir / "dist"
    build_path = script_dir / "build"
    spec_path = script_dir
    # Add data folders to bundle
    datas = [
        ("programs", "programs"),
        ("logs", "logs"),
        ("output", "output"),
        ("libs", "libs"),
        ("config", "config"),
        ("libs/python_bindings/pyllt", "pyllt"),
    ]
    datas_args = []
    for src, dest in datas:
        datas_args += ["--add-data", f"{src}{os.pathsep}{dest}"]
    # Ensure all numpy binaries and data are collected
    numpy_collect_args = ["--collect-all", "numpy"]
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--onefile",
        "--windowed",
        "--name", "Scenario_Inspector",
        "--distpath", str(dist_path),
        "--workpath", str(build_path),
        "--specpath", str(spec_path),
    ] + datas_args + numpy_collect_args + [str(main_script)]
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
