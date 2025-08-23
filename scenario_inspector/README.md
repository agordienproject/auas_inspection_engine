# AUAS Inspection Engine - Scenario Inspector

## Overview
The AUAS Scenario Inspector is the central orchestration component of the AUAS Inspection Engine, a comprehensive automated inspection system for aircraft parts. This application provides unified control over multiple hardware systems including scanCONTROL laser scanners, Intel RealSense cameras, CRI gantry systems, and serial-controlled rotating tables. 

The system bridges the gap between individual hardware control applications and automated inspection workflows, enabling complex multi-stage inspections through an intuitive interface while maintaining real-time hardware monitoring and status reporting.

## Key Features

### 🔧 Enhanced System Integration
- **Multi-System Coordination**: Seamlessly integrates scanCONTROL laser scanners, Intel RealSense cameras, CRI gantry systems, and serial-controlled rotating tables
- **Dynamic SDK Loading**: Configurable paths for hardware SDKs loaded at runtime from `app_config.yaml` - no more hardcoded paths
- **Real Hardware Connection Testing**: Uses actual connection functions from GUI applications rather than simple network pings
- **Modular Architecture**: Standardized system interfaces make it easy to add new inspection systems
- **Intelligent Fallback**: Graceful degradation when hardware is unavailable, with detailed status reporting
- **Configuration-Driven**: All system parameters managed through centralized YAML configuration

### 📋 Advanced Scenario Management
- **YAML-based Programs**: Define complex multi-stage inspection workflows using intuitive YAML syntax
- **Coordinated System Actions**: Sophisticated inspection processes with synchronized hardware operations
- **Real-time Execution Monitoring**: Live progress tracking with detailed logging and step-by-step feedback
- **Comprehensive Error Handling**: Detailed diagnostics, recovery options, and execution rollback capabilities
- **Pre-execution Validation**: Program and system availability validation before starting inspections
- **Flexible Program Structure**: Support for parallel and sequential operations across multiple systems

### 🔐 Enterprise-Grade Security
- **bcrypt Password Hashing**: Secure password storage with configurable salt rounds (default: 12)
- **Database Security**: Encrypted credential storage and secure PostgreSQL connections
- **User Authentication**: Robust login system with session management
- **Audit Trail**: Complete inspection and user activity logging
- **Session Security**: Configurable timeouts and secure session handling

### 📊 Intelligent Data Management
- **Automatic Organization**: Smart folder structure with timestamps and inspection metadata
- **PostgreSQL Integration**: Comprehensive database for inspection records, user management, and system logs
- **Multi-format Support**: Handles CSV, images, videos, reports, and custom data formats
- **Inspection Reports**: Automated generation of detailed reports with success metrics and system status
- **Data Archival**: Structured long-term storage with easy retrieval and analysis capabilities
- **Traceability**: Full inspection traceability with part references and execution history

### 🖥️ Modern User Interface
- **Intuitive Tabbed Design**: Clean PyQt5-based interface with organized workflow management
- **Real-time System Dashboard**: Live connection monitoring with visual indicators (✅❌⚠️) for all hardware
- **Interactive Testing**: Manual and automatic testing of all configured systems with detailed diagnostics
- **Execution Control Panel**: Progress tracking, live logs, pause/resume capabilities, and execution control
- **Dynamic Configuration**: Real-time system configuration updates with immediate validation feedback
- **Status Logging**: Comprehensive connection and execution logs with timestamps

## System Architecture

The AUAS Inspection Engine follows a layered architecture that separates concerns and enables flexible integration:

```
┌────────────────────────────────────────────────────────────────────────────┐
│                   AUAS Inspection Engine                                   │
├────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐                                                   │
│  │  Scenario Inspector │                                                   │
│  │  (Main Orchestrator)│                                                   │
│  │                     │                                                   │
│  │  - PyQt5 Interface  │                                                   │
│  │  - Program Execution│                                                   │
│  │  - System Management│                                                   │
│  └─────────────────────┘                                                   │
├────────────────────────────────────────────────────────────────────────────┤
│                    System Manager Layer                                    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐ ┌─────────┐ |
│  │ Scanner     │ │ Camera      │ │ Gantry      │ │ Table     │ │ xArm    │ |
│  │ System      │ │ System      │ │ System      │ │ System    │ │ System  │ |
│  │             │ │             │ │             │ │           │ │         │ |
│  │ - SDK Loader│ │ - USBControl│ │ - CRI Client│ │ - Serial  │ │ - SDK   │ |
│  │ - Connection│ │ - Connection│ │ - TCP/IP    │ │ - Control │ │ - TCP/IP│ |
│  │ - Data Acq  │ │ - Recording │ │ - Programs  │ │ - Rotation│ │ - Motion│ |
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘ └─────────┘ |
├────────────────────────────────────────────────────────────────────────────┤
│                    Hardware Abstraction Layer                              │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐ ┌─────────┐ |
│  │scanCONTROL  │ │Intel        │ │ CRI Gantry  │ │ Serial    │ │ xArm    │ |
│  │ Laser       │ │RealSense    │ │ System      │ │ Rotating  │ │ Robot   │ |
│  │ Scanner     │ │ Camera      │ │ (TCP/IP)    │ │ Table     │ │ (SDK)   │ |
│  │             │ │ (USB)       │ │             │ │           │ │         │ |
│  │ - TCP/IP    │ │ - USB 3.0   │ │ - Ethernet  │ │ - RS232   │ │ - TCP/IP│ |
│  │ - Profile   │ │ - RGB+Depth │ │ - XML Progs │ │ - Stepper │ │ - Motion│ |
│  │ - Precision │ │ - Video Rec │ │ - Safety    │ │ - Encoder │ │ - Safety│ |
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘ └─────────┘ |
└────────────────────────────────────────────────────────────────────────────┘
```

### Key Architectural Features:
- **Separation of Concerns**: Clear boundaries between UI, business logic, and hardware interfaces
- **Configuration-Driven**: All system parameters managed through `app_config.yaml`
- **Dynamic Loading**: Runtime loading of SDKs and libraries based on configuration
- **Real Connection Functions**: Uses proven connection logic from existing GUI applications
- **Modular Design**: Easy to extend with new systems following the established patterns

## Supported Hardware Systems

### scanCONTROL Laser Scanners
- **Models**: 25xx, 26xx, 27xx, 29xx, 30xx series
- **Connection**: TCP/IP (configurable IP, default: 192.168.3.2)
- **SDK**: The program uses scanCONTROL-Windows-SDK (python_bindings)
- **Features**: High-precision profile measurement, real-time data acquisition, calibration management
- **Data Output**: CSV profiles, measurement data, calibration information
- **Integration**: **ONLY FOR WINDOWS**

### Intel RealSense Camera Systems  
- **Models**: D400, D500 series (RGB-D cameras)
- **Connection**: USB 3.0
- **Features**: RGB imaging, depth sensing, video recording, synchronized capture
- **Formats**: JPEG, PNG images; MP4 video recording; depth data
- **Integration**: Direct USB control and [Intel RealSense SDK (librealsense)](https://github.com/IntelRealSense/librealsense)
- **Capabilities**: Live preview, timed capture, multi-camera coordination

### CRI Gantry Systems
- **Controller**: Common Robot Interface (CRI) compatible systems
- **Connection**: TCP/IP (configurable IP:port, default: 192.168.3.11:3920)
- **Features**: Precise 3D positioning, XML program execution, safety monitoring, status feedback
- **Programs**: XML-based motion programs with coordinate systems and safety limits
- **Integration**: Direct TCP communication using CRI protocol
- **Safety**: Real-time safety monitoring and emergency stop capabilities

### Serial Rotating Tables
- **Connection**: USB serial (configurable port, default: /dev/ttyUSB0 for linux and COM3 for Windows)
- **Features**: Continuous rotation, precise angular positioning, speed control
- **Control**: Custom serial protocol with rotation commands and position feedback
- **Configuration**: Configurable baud rate, timeout, and communication parameters
- **Integration**: Direct serial communication with error handling and status monitoring

### Ufactory Xarm

- **Models**: xArm 6, xArm 7, and compatible Ufactory robotic arms
- **Connection**: TCP/IP (configurable IP, default: 192.168.1.222)
- **SDK**: Integrated using the official [Ufactory xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
- **Features**: 6/7-axis robotic motion, precise positioning, programmable movement sequences, safety monitoring
- **Control**: Supports loading and executing motion programs, real-time joint and Cartesian control, and feedback monitoring
- **Integration**: Modular system interface for easy addition to inspection workflows


## Configuration System

The system uses a centralized YAML configuration file (`config/app_config.yaml`) that defines all system parameters and is loaded dynamically at runtime:

```yaml
api:
  url: "127.0.0.1:3000/api"
  login_endpoint: "/auth/login"
  inspection_endpoint: "/inspections"
  timeout: 30
  use_https: false  # Set to true if using HTTPS

systems:
  scanControl:
    connection_type: "tcp"
    llt_path: "C:\\scanCONTROL-Windows-SDK\\python_bindings"
    ip: "192.168.3.2"
    timeout: 30
    type: "scanner_system"
  camera:
    connection_type: "usb"
    device_id: "Intel RealSense D435I"
    timeout: 15
    type: "camera_system"
    use_realsense_sdk: true
    resolution: "ultra"  # Options: "standard" (640x480), "medium" (848x480), "high" (1280x720), "ultra" (1920x1080)
  table:
    connection_type: "serial"
    port: "COM3"
    baudrate: 9600
    timeout: 10
    type: "rotating_table"
  gantry:
    connection_type: "tcp"
    # ip: "192.168.3.11"
    ip: "127.0.0.1"
    port: 3921
    timeout: 20
    programs_path: "C:\\Users\\Agordien\\Documents\\projects\\AUAS\\auas_inspection_engine\\scenario_inspector\\src\\systems\\gantry_programs"
    type: "gantry_system"
  xarm:
    connection_type: "tcp"
    ip: "192.168.1.222"
    timeout: 30
    programs_path: "C:\\Users\\Agordien\\Documents\\projects\\AUAS\\auas_inspection_engine\\scenario_inspector\\src\\systems\\xarm_programs"
    type: "xarm_system"
programs_path: "programs"

ftp:
  server: "ftp://127.0.0.1"
  username: "inspection_engine"
  password: "admin"
  base_path: "/inspections"
  passive_mode: true  # Use passive mode for better compatibility

output:
  base_directory: "output"
  inspection_folder_prefix: "inspection_"
  date_format: "%Y-%m-%d"
  time_format: "%Y%m%d_%H%M%S"

logging:
  level: "DEBUG"
  log_folder_path: "logs"
  log_file_prefix: "scenario_inspector"
  console: true

security:
  bcrypt_rounds: 12
  session_timeout: 3600

gui:
  window_title: "AUAS Scenario Inspector"
  theme: "default"
  default_geometry: "1200x800"
```

### Key Configuration Features:
- **Dynamic SDK Loading**: The `llt_path` parameter enables runtime loading of scanCONTROL SDK from any location
- **Hot Configuration**: Most settings can be updated without restarting the application
- **Environment Specific**: Easy to maintain different configurations for development, testing, and production
- **Validation**: Configuration is validated at startup with clear error messages for missing or invalid parameters
## Project Structure
## Project Structure
```
scenario_inspector/
├── src/
│   ├── main.py                      # Application entry point
│   ├── auth/
│   │   ├── login.py                 # User authentication logic
│   │   ├── password_utils.py        # bcrypt password hashing utilities
│   │   └── __init__.py
│   ├── config/
│   │   ├── config_manager.py        # Centralized configuration management
│   │   └── __init__.py
│   ├── database/
|   |   |── api_connection.py        # Use the API to interract with the database for login and write inspections
│   │   ├── models.py                # Data models (User, Inspection, System)
│   │   └── __init__.py
│   ├── gui/
│   │   ├── dialogs/                            
│   │   │   ├── __init__.py                     # Dialogs package initializer
│   │   │   └── inspection_database_dialog.py   # Dialog for inspection database operations
│   │   ├── tabs/                               
│   │   │   ├── __init__.py                     # Tabs package initializer
│   │   │   ├── execution_tab.py                # Execution control and monitoring tab
│   │   │   ├── piece_tab.py                    # Part information entry tab
│   │   │   ├── program_tab.py                  # Inspection program selection and editing tab
│   │   │   ├── settings_tab.py                 # Application and system settings tab
│   │   │   └── status_tab.py                   # System status and diagnostics tab
│   │   ├── widgets/                            
│   │   │   ├── __init__.py                     # Widgets package initializer
│   │   │   ├── nowheel_spinbox.py              # Spinbox widget without mouse wheel support
│   │   │   └── execution_thread.py             # Thread for background execution tasks
│   │   └── main_window.py                      # Main application window and layout
│   ├── inspector/
│   │   ├── __init__.py
│   │   ├── data_processor.py        # Data processing and transformation utilities
│   │   └── scenario_engine.py       # Scenario execution and orchestration logic
│   ├── systems/
│   │   ├── system_manager.py        # Main system orchestrator with real connection testing
│   │   ├── base_system.py           # Abstract base system class interface
│   │   ├── scanner_system.py        # scanCONTROL integration with dynamic SDK loading
│   │   ├── camera_system.py         # Intel RealSense camera integration with SDK integration
│   │   ├── gantry_system.py         # CRI gantry integration with TCP communication
│   │   ├── table_system.py          # Serial rotating table integration
│   │   └── xarm_system.py           # Ufactory Xarm with SDK
│   ├── utils/                      
│   |   ├── __init__.py              # Utils package initializer
│   |   ├── date_utils.py            # Date/time utilities for data organization
│   |   ├── file_manager.py          # Data organization and file management utilities
│   |   └── ftp_manager.py           # FTP upload/download helpers
├── config/
│   └── app_config.yaml              # Centralized system configuration file
├── libs/                            # Third-party and vendor libraries
│   ├── cri_lib/                     # CRI gantry system client library
│   └── python_bindings/             # scanCONTROL SDK Python bindings
├── programs/                        # YAML-based inspection program definitions
│   └── example_inspection.yaml      # Sample inspection program
├── output/                          # Generated inspection data with organized structure
│   └── YYYY-MM-DD/                  # Date-based organization
│       └── program_piece_ref_time/  # Inspection-specific folders
├── logs/                            # Application and system logs
│   └── YYYY-MM-DD/                  # Date-based organization
├── requirements.txt                 # Python dependencies including new packages
├── Scnario_Inspector.exe            # The exe file of the app to use it
├── setup.py                         # Package setup configuration
└── README.md                        # This comprehensive documentation
```

## Usage

### Starting the Application
```bash
# Start the main application
cd src
python main.py

```

### User Interface Overview

#### 1. Login Authentication
- **Secure Login**: bcrypt-verified credentials stored in PostgreSQL database
- **Session Management**: Automatic session handling with configurable timeouts
- **User Registration**: Create new accounts through the interface (if enabled)
- **Password Security**: All passwords automatically hashed with salt rounds

#### 2. Main Interface Tabs

**Piece Info Tab**
- **Part Information Entry**: Enter part name, reference number, and inspection details
- **Required Fields**: Part name and reference number are mandatory for inspection execution
- **Validation**: Real-time validation ensures all required information is provided
- **History**: Access to previous part inspections and data

**Programs Tab**
- **Program Library**: Browse and select from available YAML inspection programs
- **Program Upload**: Upload new YAML program files with validation
- **Program Preview**: View program structure, stages, and steps before execution
- **Program Validation**: Pre-execution validation of program syntax and system requirements
- **Program Editor**: Basic editing capabilities for program modifications

**Execution Tab**
- **Inspection Control**: Start, pause, resume, and stop inspection execution
- **Real-time Progress**: Live progress tracking with stage and step indicators
- **Execution Logs**: Detailed logs with timestamps for each action and system response
- **Error Handling**: Clear error messages and recovery options during execution
- **Data Monitoring**: Real-time display of acquired data and system status

**System Status Tab (Enhanced)**
- **Live Connection Status**: Real-time monitoring with visual indicators:
  - ✅ Connected and operational
  - ❌ Connection failed or system unavailable  
  - ⚠️ Warning or partial functionality
- **System Details**: Comprehensive information for each system:
  - IP addresses, ports, and connection parameters
  - Hardware status and capabilities
  - Last successful connection time
  - Error messages and diagnostics
- **Manual Testing**: "Test All Connections" button for comprehensive system check
- **Connection Log**: Timestamped log of all connection attempts and results
- **System Configuration**: View and validate current system settings

**Settings tab**
- **Read config app**: Directly prints all the properties of the file config/app_config.yaml
- **Modification**: You can directly modify the properties of the app inside it
- **Saving modification**: Once you are done modifying properties, save and the file will be updated 
- **Application restarts**: The application will restart by itself

### Intelligent Data Organization
Automatic creation of organized output structure with comprehensive metadata:

```
output/
├── 2025-01-15/                              # Date-based organization
│   └── WingInspection_PartA123_REF001_14-30-25/  # Inspection-specific folder
│       ├── inspection_metadata.json        # Complete inspection details
│       ├── program_definition.yaml         # Copy of executed program
│       ├── system_status_log.txt          # System status during execution
│       ├── Setup_Load_Gantry_Program/     # Stage and step organization
│       │   ├── execution_log.txt
│       │   └── gantry_program.xml
│       ├── Data_Acquisition_Laser_Scan/
│       │   ├── wing_scan_profiles.csv
│       │   ├── scan_parameters.json
│       │   └── calibration_data.cal
│       ├── Data_Acquisition_Images/
│       │   ├── wing_ref_001.jpg
│       │   ├── wing_ref_002.jpg
│       │   └── capture_metadata.json
│       └── Quality_Check_Report/
│           ├── wing_inspection_report.pdf
│           ├── quality_metrics.csv
│           └── inspection_summary.json
```

## Contributing

Create a virtual environment, install the app-specific requirements, and run the entry point for the application you work on. Open a PR and describe the change and tests.

## Authors and license

Maintained by the AUAS Engineering Team (Alexis Gordien). See application READMEs for more details and any licensing notes.