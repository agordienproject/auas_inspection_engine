#!/usr/bin/env python3
"""
AUAS Inspection Engine - Quick Installation Script
Run this script to install and configure the entire AUAS Inspection Engine
"""
import os
import sys
import subprocess
import shutil
import importlib
from pathlib import Path
from typing import List

def print_header():
    """Print installation header"""
    print("\n" + "="*70)
    print("AUAS Inspection Engine - Installation Script")
    print("="*70)
    print("This script will install all components:")
    print("  Scenario Inspector (Main Application)")
    print("  FTP Server (Data Transfer)")
    print("  CRI Library (Gantry Control)")
    print("  Configuration Files")
    print("="*70)

def check_python_version():
    """Check if Python version is compatible"""
    if sys.version_info < (3, 8):
        print("Error: Python 3.8 or higher is required")
        print(f"   Current version: {sys.version}")
        sys.exit(1)
    else:
        print(f"Python version: {sys.version.split()[0]}")

def install_scenario_inspector():
    """Install the scenario inspector with all dependencies"""
    print("\nInstalling Scenario Inspector...")
    scenario_dir = Path(__file__).parent / "scenario_inspector"
    
    if not scenario_dir.exists():
        print("Error: scenario_inspector directory not found")
        return False
    
    try:
        # Change to scenario_inspector directory and install dependencies
        original_cwd = os.getcwd()
        os.chdir(scenario_dir)
        ok = True

        # 1) Install Python requirements explicitly (more reliable than setup.py install)
        req_file = scenario_dir / "requirements.txt"
        if req_file.exists():
            print("Installing Scenario Inspector requirements...")
            req_result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(req_file)],
                                        capture_output=True, text=True, check=False)
            if req_result.returncode != 0:
                print("Error installing Scenario Inspector requirements:")
                print(req_result.stderr)
                ok = False
            else:
                print("Scenario Inspector requirements installed")
        else:
            print("Warning: requirements.txt not found in scenario_inspector; continuing...")

        # 2) Install package itself via pip (avoids deprecated setup.py install)
        if ok:
            print("Installing Scenario Inspector package (pip install .)...")
            pkg_result = subprocess.run([sys.executable, "-m", "pip", "install", "."],
                                        capture_output=True, text=True, check=False)
            if pkg_result.returncode != 0:
                print("Error installing Scenario Inspector package:")
                print(pkg_result.stderr)
                ok = False
            else:
                print("Scenario Inspector package installed")

        # 3) Run post-install steps that setup.py used to do (env file, directories)
        try:
            if not create_root_env_file():
                print("Warning: Failed to create/update .env file")
            # Copy .env into ftp_server/dist for the exe runtime
            copy_env_to_ftp_dist()
            ensure_scenario_output_directories()
        except Exception as e:
            print(f"Warning during post-install steps: {e}")

        return ok
            
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        os.chdir(original_cwd)

def create_root_env_file() -> bool:
    """Create a .env file at repository root if it doesn't exist, mirroring setup.py behavior."""
    try:
        project_root = Path(__file__).parent
        env_file = project_root / ".env"
        if env_file.exists():
            print(".env file already exists at repository root; leaving unchanged")
            return True

        print("Creating .env file at repository root...")
        username = os.getenv('USERNAME', 'User')
        content = (
            "# .env file for AUAS Inspection Engine\n"
            "# Set the FTP server host, port, and base path here\n"
            "FTP_HOST=127.0.0.1\n"
            "FTP_PORT=21\n"
            f"FTP_BASE_PATH=C:\\Users\\{username}\\Documents\\projects\\AUAS\\FTP\n\n"
            "# Project Root Path (for CRI lib and other dependencies)\n"
            f"PROJECT_ROOT_PATH={str(project_root).replace('\\', '\\\\')}\n\n"
            "# Scanner System Defaults\n"
            "SCANNER_DEFAULT_IP=192.168.3.2\n"
            "SCANNER_LLT_SDK_PATH=C:\\scanCONTROL-Windows-SDK\\python_bindings\n\n"
            "# Gantry System Defaults  \n"
            "GANTRY_DEFAULT_IP=192.168.3.11\n\n"
            "# XArm System Defaults\n"
            "XARM_DEFAULT_IP=192.168.1.222\n\n"
            "# API Defaults\n"
            "API_DEFAULT_URL=127.0.0.1:3000/api\n\n"
            "# FTP Defaults (for inspector)\n"
            "FTP_DEFAULT_SERVER=ftp://127.0.0.1\n"
        )
        env_file.write_text(content, encoding="utf-8")
        print(f"Created .env file: {env_file}")
        return True
    except Exception as e:
        print(f"Error creating .env: {e}")
        return False

def ensure_scenario_output_directories():
    """Ensure scenario_inspector output and logs directories exist."""
    scenario_dir = Path(__file__).parent / "scenario_inspector"
    for d in [scenario_dir / "output", scenario_dir / "logs"]:
        try:
            d.mkdir(parents=True, exist_ok=True)
            # Provide feedback only if created new
            print(f"Ensured directory exists: {d}")
        except Exception as e:
            print(f"Warning: could not ensure directory {d}: {e}")

def copy_env_to_ftp_dist() -> bool:
    """Copy the repository-root .env file into ftp_server/dist/.env for the exe runtime."""
    try:
        project_root = Path(__file__).parent
        env_src = project_root / ".env"
        if not env_src.exists():
            print("Warning: .env not found at repository root; skipping copy to ftp_server/dist")
            return False

        dist_dir = project_root / "ftp_server" / "dist"
        dist_dir.mkdir(parents=True, exist_ok=True)
        target = dist_dir / ".env"
        shutil.copy2(env_src, target)
        print(f"Copied .env to {target}")
        return True
    except Exception as e:
        print(f"Warning: Failed to copy .env to ftp_server/dist: {e}")
        return False

def install_ftp_server_dependencies():
    """Install FTP server dependencies"""
    print("\nInstalling FTP Server dependencies...")
    ftp_dir = Path(__file__).parent / "ftp_server"
    
    if not ftp_dir.exists():
        print("Warning: ftp_server directory not found, skipping...")
        return True
    
    try:
        requirements_file = ftp_dir / "requirements.txt"
        if requirements_file.exists():
            result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(requirements_file)], 
                                  capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print("FTP Server dependencies installed")
                return True
            else:
                print("Error installing FTP Server dependencies:")
                print(result.stderr)
                return False
        else:
            print("No requirements.txt found for FTP server")
            return True
            
    except Exception as e:
        print(f"Error: {e}")
        return False


def install_scenario_creator():
    """Install the scenario creator GUI dependencies"""
    print("\nInstalling Scenario Creator...")
    creator_dir = Path(__file__).parent / "scenario_creator"

    if not creator_dir.exists():
        print("Warning: scenario_creator directory not found, skipping...")
        return True

    try:
        requirements_file = creator_dir / "requirements.txt"
        if requirements_file.exists():
            result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(requirements_file)],
                                    capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print("Scenario Creator dependencies installed")
            else:
                print("Error installing Scenario Creator dependencies:")
                print(result.stderr)
                return False
        # If there's a setup.py, attempt a local install
        setup_file = creator_dir / "setup.py"
        if setup_file.exists():
            result = subprocess.run([sys.executable, str(setup_file), "install"], capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print("Scenario Creator installed via setup.py")
            else:
                print("Warning: scenario_creator setup.py install failed:")
                print(result.stderr)
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

def install_scan_tools():
    """Install the scan_tools utilities (viewer + processor).

    - Installs Python dependencies from scan_tools/requirements.txt
    - Creates scan_tools/.env from scan_tools/.env.template (copy as-is)
    """
    print("\nInstalling Scan Tools (viewer & processor)...")
    tools_dir = Path(__file__).parent / "scan_tools"

    if not tools_dir.exists():
        print("Warning: scan_tools directory not found, skipping...")
        return True

    ok = True
    try:
        # 1) Install Python requirements
        req = tools_dir / "requirements.txt"
        if req.exists():
            print(f"Installing Python packages from {req} ...")
            result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(req)],
                                    capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print("Scan Tools dependencies installed")
            else:
                print("Error installing Scan Tools dependencies:")
                print(result.stderr)
                ok = False
        else:
            print("No requirements.txt found for scan_tools")

        # 2) Create .env from template (copy values as-is)
        env_template = tools_dir / ".env.template"
        env_target = tools_dir / ".env"
        if env_template.exists():
            if env_target.exists():
                print("scan_tools/.env already exists; leaving existing values unchanged")
            else:
                env_content = env_template.read_text(encoding="utf-8")
                env_target.write_text(env_content, encoding="utf-8")
                print("Created scan_tools/.env from .env.template (values copied unchanged)")
        else:
            print("Warning: scan_tools/.env.template not found; skipping .env creation")

    except Exception as e:
        print(f"Error installing Scan Tools: {e}")
        ok = False

    return ok

def _escape_yaml_windows_path(p: Path) -> str:
    """Return a YAML-safe double-quoted Windows path string.

    YAML with double quotes treats backslash as escape, so we need to escape
    them. Example: C:\\dir\\file
    """
    s = str(p)
    return '"' + s.replace('\\', '\\\\') + '"'

def _patch_programs_paths_in_yaml_lines(lines: List[str], gantry_path: Path, xarm_path: Path) -> List[str]:
    """Update programs_path for gantry and xarm inside systems section.

    This avoids adding external YAML dependencies by performing a
    structure-aware line edit based on indentation.
    """
    new_lines = list(lines)

    in_systems = False
    current_section = None  # 'gantry' | 'xarm' | None

    gantry_value = _escape_yaml_windows_path(gantry_path)
    xarm_value = _escape_yaml_windows_path(xarm_path)

    def replace_line(i: int, indent: int, value: str):
        new_lines[i] = (" " * indent) + f"programs_path: {value}\n"

    for i, line in enumerate(new_lines):
        stripped = line.lstrip()
        if stripped.startswith('#'):
            continue
        indent = len(line) - len(stripped)

        # Enter systems top-level
        if not in_systems and stripped.startswith('systems:') and indent == 0:
            in_systems = True
            current_section = None
            continue

        if in_systems:
            # Leaving systems when a new top-level key appears
            if stripped and indent == 0 and not stripped.startswith('systems:'):
                in_systems = False
                current_section = None
                continue

            # Detect subsections (e.g., '  gantry:' or '  xarm:')
            if indent == 2 and stripped.startswith('gantry:'):
                current_section = 'gantry'
                continue
            if indent == 2 and stripped.startswith('xarm:'):
                current_section = 'xarm'
                continue

            # If we dedent back to subsection level or blank line, keep state until new section
            if current_section and indent >= 4:
                # Inside current section body
                if stripped.startswith('programs_path:'):
                    if current_section == 'gantry':
                        replace_line(i, indent, gantry_value)
                    elif current_section == 'xarm':
                        replace_line(i, indent, xarm_value)
                    continue

    return new_lines

def prepare_app_config() -> bool:
    """Ensure app_config.yaml exists and has correct local programs paths.

    - Copies app_config_template.yaml to app_config.yaml if missing.
    - Updates systems.gantry.programs_path and systems.xarm.programs_path
      to absolute paths within this checkout.
    """
    try:
        repo_root = Path(__file__).parent
        config_dir = repo_root / 'scenario_inspector' / 'config'
        template_file = config_dir / 'app_config_template.yaml'
        target_file = config_dir / 'app_config.yaml'

        if not template_file.exists():
            print(f"Warning: Template config not found: {template_file}")
            return False

        if not target_file.exists():
            # Copy template -> target
            content = template_file.read_text(encoding='utf-8')
            target_file.write_text(content, encoding='utf-8')
            print("Created app_config.yaml from template")
        else:
            print("app_config.yaml already exists; leaving existing values in place")
            # Still attempt to patch paths to ensure local correctness

        # Compute real paths
        systems_dir = repo_root / 'scenario_inspector' / 'src' / 'systems'
        gantry_programs = systems_dir / 'gantry_programs'
        xarm_programs = systems_dir / 'xarm_programs'

        if not gantry_programs.exists():
            print(f"Warning: gantry_programs directory not found: {gantry_programs}")
        if not xarm_programs.exists():
            print(f"Warning: xarm_programs directory not found: {xarm_programs}")

        # Patch YAML text lines
        lines = target_file.read_text(encoding='utf-8').splitlines(keepends=True)
        patched = _patch_programs_paths_in_yaml_lines(lines, gantry_programs, xarm_programs)
        target_file.write_text(''.join(patched), encoding='utf-8')
        print("Patched programs_path for gantry and xarm in app_config.yaml")
        return True
    except Exception as e:
        print(f"Error preparing app_config.yaml: {e}")
        return False

def verify_installation():
    """Verify the installation was successful"""
    print("\nVerifying installation...")
    
    # Check if main modules can be imported
    try:
        # Test scenario inspector
        scenario_src = Path(__file__).parent / "scenario_inspector" / "src"
        if scenario_src.exists():
            sys.path.insert(0, str(scenario_src))

            # Try importing main components using importlib to avoid unused import lint
            importlib.import_module("config.config_manager")
            importlib.import_module("systems.system_manager")
            print("Scenario Inspector modules: OK")

    except ImportError as e:
        print(f"Warning: Some modules may not be properly installed: {e}")
    
    # Check configuration files
    config_file = Path(__file__).parent / "scenario_inspector" / "config" / "app_config.yaml"
    if config_file.exists():
        print("Configuration files: OK")
    else:
        print("Warning: Configuration files not found")
    
    # Check .env file
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        print("Environment file: OK")
    else:
        print("Warning: .env file not found")

def print_completion_info():
    """Print completion information"""
    print("\n" + "="*70)
    print("Installation completed!")
    print("="*70)
    print("Next steps:")
    print("  1. Navigate to scenario_inspector/src/")
    print("  2. Run: python main.py")
    print("  3. Configure systems in the Settings tab")
    print("")
    print("To start FTP server:")
    print("  1. Navigate to ftp_server/")
    print("  2. Run: python server.py")
    print("")
    print("To start Scenario Creator (authoring tool):")
    print("  1. Navigate to scenario_creator/")
    print("  2. Run: python scenario_creator.py")
    print("")
    print("Scan Tools (viewer & processor):")
    print("  - Location: scan_tools/")
    print("  - Start Viewer: start-viewer.bat or python scan_viewer.py")
    print("  - Start Processor: start-processor.bat or python enhanced_scan_processor.py")
    print("  - Config: scan_tools/.env (created from .env.template)")
    print("")
    print("Configuration files:")
    print("  - Main config: scenario_inspector/config/app_config.yaml")
    print("  - Environment: .env")
    print("")
    print("For help and documentation, check the README.md files")
    print("="*70)

def main():
    """Main installation process"""
    print_header()
    
    # Check Python version
    check_python_version()
    
    # Install components
    success = True
    
    if not install_scenario_inspector():
        success = False
    
    if not install_ftp_server_dependencies():
        success = False
    
    # Install Scenario Creator (authoring tool)
    if not install_scenario_creator():
        success = False
    
    # Install Scan Tools (viewer & processor)
    if not install_scan_tools():
        success = False

    # Prepare configuration (copy template and patch paths)
    if not prepare_app_config():
        success = False

    # Verify installation
    verify_installation()
    
    # Print completion info
    if success:
        print_completion_info()
        return 0
    else:
        print("\nInstallation completed with errors. Please check the output above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
