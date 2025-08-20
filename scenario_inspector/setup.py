#!/usr/bin/env python3
"""
Setup script for AUAS Scenario Inspector
This script installs all dependencies and sets up the application for use
"""
import os
import sys
import shutil
from pathlib import Path
from setuptools import setup, find_packages
from setuptools.command.install import install

class CustomInstallCommand(install):
    """Custom installation command to handle post-install setup"""
    
    def run(self):
        # Run the standard installation
        install.run(self)
        
        # Post-install setup
        self.post_install_setup()
    
    def post_install_setup(self):
        """Perform post-installation setup tasks"""
        print("\n" + "="*60)
        print("AUAS Scenario Inspector - Post-Install Setup")
        print("="*60)
        
        # Create .env file if it doesn't exist
        self.create_env_file()
        
        # Verify configuration files
        self.verify_config_files()
        
        # Create output directories
        self.create_output_directories()
        
        print("\nSetup completed successfully!")
        print("You can now run the application with: python src/main.py")
        print("="*60)
    
    def create_env_file(self):
        """Create .env file in project root if it doesn't exist"""
        project_root = Path(__file__).parent.parent
        env_file = project_root / ".env"
        
        if not env_file.exists():
            print("Creating .env file...")
            env_content = """# .env file for AUAS Inspection Engine
# Set the FTP server host, port, and base path here
FTP_HOST=127.0.0.1
FTP_PORT=21
FTP_BASE_PATH=C:\\Users\\{username}\\Documents\\projects\\AUAS\\FTP

# Project Root Path (for CRI lib and other dependencies)
PROJECT_ROOT_PATH={project_root}

# Scanner System Defaults
SCANNER_DEFAULT_IP=192.168.3.2
SCANNER_LLT_SDK_PATH=C:\\scanCONTROL-Windows-SDK\\python_bindings

# Gantry System Defaults  
GANTRY_DEFAULT_IP=192.168.3.11

# XArm System Defaults
XARM_DEFAULT_IP=192.168.1.222

# API Defaults
API_DEFAULT_URL=127.0.0.1:3000/api

# FTP Defaults (for inspector)
FTP_DEFAULT_SERVER=ftp://127.0.0.1""".format(
                username=os.getenv('USERNAME', 'User'),
                project_root=str(project_root).replace('\\', '\\\\')
            )
            
            with open(env_file, 'w') as f:
                f.write(env_content)
            print(f"Created .env file at: {env_file}")
        else:
            print(".env file already exists")
    
    def verify_config_files(self):
        """Verify configuration files exist"""
        config_dir = Path(__file__).parent / "config"
        app_config = config_dir / "app_config.yaml"
        app_config_template = config_dir / "app_config_template.yaml"
        
        if not app_config.exists() and app_config_template.exists():
            print("Creating app_config.yaml from template...")
            shutil.copy2(app_config_template, app_config)
            print("Created app_config.yaml")
        elif app_config.exists():
            print("app_config.yaml exists")
        else:
            print("Warning: No configuration files found")
    
    def create_output_directories(self):
        """Create necessary output directories"""
        scenario_dir = Path(__file__).parent
        
        directories = [
            scenario_dir / "output",
            scenario_dir / "logs",
        ]
        
        for directory in directories:
            if not directory.exists():
                directory.mkdir(parents=True, exist_ok=True)
                print(f"Created directory: {directory}")
            else:
                print(f"Directory exists: {directory}")

setup(
    name='auas-scenario-inspector',
    version='1.0.0',
    author='AUAS Inspection Team',
    author_email='inspection-team@auas.nl',
    description='Automated inspection scenario execution system for AUAS',
    long_description="""
    AUAS Scenario Inspector is a comprehensive inspection automation system that provides:
    - Multi-system coordination (Camera, Scanner, Gantry, XArm, Rotating Table)
    - Automated inspection scenario execution
    - Real-time system monitoring and control
    - Data collection and FTP upload capabilities
    - Windows-optimized setup for laboratory environments
    """,
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        # Core application dependencies
        'PyQt5>=5.15.0',
        'PyYAML>=6.0',
        'python-dateutil>=2.8.0',
        'typing-extensions>=4.0.0',
        
        # Database and API
        'psycopg2-binary>=2.9.0',  # PostgreSQL adapter
        'requests>=2.25.0',        # HTTP requests for API
        'bcrypt>=4.0.0',           # Password hashing
        
        # Hardware interface dependencies
        'pyserial>=3.5',           # Serial communication (rotating table)
        'pyrealsense2>=2.50.0',    # Intel RealSense SDK for Windows
        'opencv-python>=4.5.0',    # Computer vision
        'numpy>=1.21.0',           # Numerical operations
        
        # Robot control
        'xarm-python-sdk>=1.11.0', # XArm robot control
        
        # Windows-specific
        'WMI>=1.5.1',              # Windows device management (optional)
        'python-dotenv>=1.0.0',    # Environment variable loading
        
        # FTP server (for standalone FTP functionality)
        'pyftpdlib>=1.5.9',        # FTP server implementation
        
        # Development and packaging (optional)
        'pyinstaller>=5.13.0',     # For creating executables
    ],
    extras_require={
        'dev': [
            'pytest>=6.0.0',
            'pytest-qt>=4.0.0',
            'black>=21.0.0',
            'flake8>=3.9.0',
        ],
        'scanner': [
            # Note: scanCONTROL SDK must be installed separately from vendor
            # Path should be configured in app_config.yaml under systems.scanControl.llt_path
        ],
    },
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            'scenario-inspector=main:main',
            'auas-ftp-server=ftp_server.server:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Manufacturing',
        'Topic :: Scientific/Engineering :: Quality Control',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Operating System :: Microsoft :: Windows',
        'License :: OSI Approved :: MIT License',
    ],
    project_urls={
        'Bug Reports': 'https://github.com/agordienproject/auas_inspection_engine/issues',
        'Source': 'https://github.com/agordienproject/auas_inspection_engine',
        'Documentation': 'https://github.com/agordienproject/auas_inspection_engine/wiki',
    },
)