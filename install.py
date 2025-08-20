#!/usr/bin/env python3
"""
AUAS Inspection Engine - Quick Installation Script
Run this script to install and configure the entire AUAS Inspection Engine
"""
import os
import sys
import subprocess
from pathlib import Path

def print_header():
    """Print installation header"""
    print("\n" + "="*70)
    print("🏗️  AUAS Inspection Engine - Installation Script")
    print("="*70)
    print("This script will install all components:")
    print("  📊 Scenario Inspector (Main Application)")
    print("  📁 FTP Server (Data Transfer)")
    print("  🔧 CRI Library (Gantry Control)")
    print("  ⚙️  Configuration Files")
    print("="*70)

def check_python_version():
    """Check if Python version is compatible"""
    if sys.version_info < (3, 8):
        print("❌ Error: Python 3.8 or higher is required")
        print(f"   Current version: {sys.version}")
        sys.exit(1)
    else:
        print(f"✅ Python version: {sys.version.split()[0]}")

def install_scenario_inspector():
    """Install the scenario inspector with all dependencies"""
    print("\n📊 Installing Scenario Inspector...")
    scenario_dir = Path(__file__).parent / "scenario_inspector"
    
    if not scenario_dir.exists():
        print("❌ Error: scenario_inspector directory not found")
        return False
    
    try:
        # Change to scenario_inspector directory and run setup
        original_cwd = os.getcwd()
        os.chdir(scenario_dir)
        
        # Install using setup.py which will handle all dependencies and post-install setup
        result = subprocess.run([sys.executable, "setup.py", "install"], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✅ Scenario Inspector installed successfully")
            return True
        else:
            print("❌ Error installing Scenario Inspector:")
            print(result.stderr)
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    finally:
        os.chdir(original_cwd)

def install_ftp_server_dependencies():
    """Install FTP server dependencies"""
    print("\n📁 Installing FTP Server dependencies...")
    ftp_dir = Path(__file__).parent / "ftp_server"
    
    if not ftp_dir.exists():
        print("⚠️  Warning: ftp_server directory not found, skipping...")
        return True
    
    try:
        requirements_file = ftp_dir / "requirements.txt"
        if requirements_file.exists():
            result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(requirements_file)], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print("✅ FTP Server dependencies installed")
                return True
            else:
                print("❌ Error installing FTP Server dependencies:")
                print(result.stderr)
                return False
        else:
            print("⚠️  No requirements.txt found for FTP server")
            return True
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def verify_installation():
    """Verify the installation was successful"""
    print("\n🔍 Verifying installation...")
    
    # Check if main modules can be imported
    try:
        # Test scenario inspector
        scenario_src = Path(__file__).parent / "scenario_inspector" / "src"
        if scenario_src.exists():
            sys.path.insert(0, str(scenario_src))
            
            # Try importing main components
            import config.config_manager
            import systems.system_manager
            print("✅ Scenario Inspector modules: OK")
            
    except ImportError as e:
        print(f"⚠️  Warning: Some modules may not be properly installed: {e}")
    
    # Check configuration files
    config_file = Path(__file__).parent / "scenario_inspector" / "config" / "app_config.yaml"
    if config_file.exists():
        print("✅ Configuration files: OK")
    else:
        print("⚠️  Warning: Configuration files not found")
    
    # Check .env file
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        print("✅ Environment file: OK")
    else:
        print("⚠️  Warning: .env file not found")

def print_completion_info():
    """Print completion information"""
    print("\n" + "="*70)
    print("🎉 Installation completed!")
    print("="*70)
    print("📋 Next steps:")
    print("  1. Navigate to scenario_inspector/src/")
    print("  2. Run: python main.py")
    print("  3. Configure systems in the Settings tab")
    print("")
    print("📁 To start FTP server:")
    print("  1. Navigate to ftp_server/")
    print("  2. Run: python server.py")
    print("")
    print("⚙️  Configuration files:")
    print(f"  - Main config: scenario_inspector/config/app_config.yaml")
    print(f"  - Environment: .env")
    print("")
    print("📖 For help and documentation, check the README.md files")
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
    
    # Verify installation
    verify_installation()
    
    # Print completion info
    if success:
        print_completion_info()
        return 0
    else:
        print("\n❌ Installation completed with errors. Please check the output above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
