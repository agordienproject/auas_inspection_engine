@echo off
REM Build Scenario Creator executable using PyInstaller
cd /d %~dp0
python build_exe.py
