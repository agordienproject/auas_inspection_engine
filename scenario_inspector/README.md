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
┌─────────────────────────────────────────────────────────────────┐
│                   AUAS Inspection Engine                        │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐    ┌─────────────────────────────────┐ │
│  │  Scenario Inspector │◄──►│      GUI Applications           │ │
│  │  (Main Orchestrator)│    │   (Individual System Control)   │ │
│  │                     │    │   - Scanner GUI                 │ │
│  │  - PyQt5 Interface  │    │   - Camera Control              │ │
│  │  - Program Execution│    │   - Gantry Control              │ │
│  │  - System Management│    │   - Table Control               │ │
│  └─────────────────────┘    └─────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                    System Manager Layer                         │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐  │
│  │ Scanner     │ │ Camera      │ │ Gantry      │ │ Table     │  │
│  │ System      │ │ System      │ │ System      │ │ System    │  │
│  │             │ │             │ │             │ │           │  │
│  │ - SDK Loader│ │ - ROS2 Integ│ │ - CRI Client│ │ - Serial  │  │
│  │ - Connection│ │ - USBControl│ │ - TCP/IP    │ │ - Control │  │
│  │ - Data Acq  │ │ - Recording │ │ - Programs  │ │ - Rotation│  │
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                    Hardware Abstraction Layer                   │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐  │
│  │scanCONTROL  │ │Intel        │ │ CRI Gantry  │ │ Serial    │  │
│  │ Laser       │ │RealSense    │ │ System      │ │ Rotating  │  │
│  │ Scanner     │ │ Camera      │ │ (TCP/IP)    │ │ Table     │  │
│  │             │ │ (USB/ROS2)  │ │             │ │           │  │
│  │ - TCP/IP    │ │ - USB 3.0   │ │ - Ethernet  │ │ - RS232   │  │
│  │ - Profile   │ │ - RGB+Depth │ │ - XML Progs │ │ - Stepper │  │
│  │ - Precision │ │ - Video Rec │ │ - Safety    │ │ - Encoder │  │
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘  │
└─────────────────────────────────────────────────────────────────┘
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
- **SDK**: Dynamic loading from configurable `llt_path` in `app_config.yaml`
- **Features**: High-precision profile measurement, real-time data acquisition, calibration management
- **Data Output**: CSV profiles, measurement data, calibration information
- **Integration**: Uses proven connection functions from scanCONTROL GUI application

### Intel RealSense Camera Systems  
- **Models**: D400, D500 series (RGB-D cameras)
- **Connection**: USB 3.0 with ROS2 integration support
- **Features**: RGB imaging, depth sensing, video recording, synchronized capture
- **Formats**: JPEG, PNG images; MP4 video recording; depth data
- **Integration**: Direct USB control with fallback to ROS2 when available
- **Capabilities**: Live preview, timed capture, multi-camera coordination

### CRI Gantry Systems
- **Controller**: Common Robot Interface (CRI) compatible systems
- **Connection**: TCP/IP (configurable IP:port, default: 192.168.3.11:3920)
- **Features**: Precise 3D positioning, XML program execution, safety monitoring, status feedback
- **Programs**: XML-based motion programs with coordinate systems and safety limits
- **Integration**: Direct TCP communication using CRI protocol
- **Safety**: Real-time safety monitoring and emergency stop capabilities

### Serial Rotating Tables
- **Connection**: RS232/USB serial (configurable port, default: /dev/ttyUSB0)
- **Features**: Continuous rotation, precise angular positioning, speed control
- **Control**: Custom serial protocol with rotation commands and position feedback
- **Configuration**: Configurable baud rate, timeout, and communication parameters
- **Integration**: Direct serial communication with error handling and status monitoring

## Configuration System

The system uses a centralized YAML configuration file (`config/app_config.yaml`) that defines all system parameters and is loaded dynamically at runtime:

```yaml
# Database Configuration
database:
  host: "localhost"
  port: 5432
  username: "postgres"
  password: "your_password"
  database: "auas_inspection"

# System Hardware Configuration
systems:
  scanControl:
    connection_type: "tcp"
    llt_path: "/path/to/scanCONTROL/SDK/python_bindings"  # Dynamic SDK loading path
    ip: "192.168.3.2"
    timeout: 30
    type: "laser_scanner"
    
  camera:
    connection_type: "usb"
    device_id: "Intel Corp"  # USB device identifier
    ros2_enabled: true       # Enable ROS2 integration if available
    timeout: 15
    type: "camera_system"
    
  gantry:
    connection_type: "tcp"
    ip: "192.168.3.11"
    port: 3920
    timeout: 20
    type: "gantry_system"
    safety_enabled: true     # Enable safety monitoring
    
  table:
    connection_type: "serial"
    port: "/dev/ttyUSB0"
    baudrate: 9600
    timeout: 10
    type: "rotating_table"

# Application Settings
application:
  output_directory: "./output"
  log_level: "INFO"
  max_log_files: 10
  connection_retry_attempts: 3
  default_inspection_timeout: 300

# Security Settings
security:
  bcrypt_rounds: 12          # Password hashing strength
  session_timeout: 3600      # Session timeout in seconds
  max_login_attempts: 5      # Maximum failed login attempts
```

### Key Configuration Features:
- **Dynamic SDK Loading**: The `llt_path` parameter enables runtime loading of scanCONTROL SDK from any location
- **Hot Configuration**: Most settings can be updated without restarting the application
- **Environment Specific**: Easy to maintain different configurations for development, testing, and production
- **Validation**: Configuration is validated at startup with clear error messages for missing or invalid parameters
## Project Structure
```
scenario_inspector/
├── src/
│   ├── main.py                      # Application entry point
│   ├── config/
│   │   ├── config_manager.py        # Centralized configuration management
│   │   └── __init__.py
│   ├── auth/
│   │   ├── login.py                 # User authentication logic
│   │   ├── password_utils.py        # bcrypt password hashing utilities
│   │   └── __init__.py
│   ├── database/
│   │   ├── connection.py            # Database connection with bcrypt authentication
│   │   ├── models.py                # Data models (User, Inspection, System)
│   │   └── __init__.py
│   ├── systems/
│   │   ├── system_manager.py        # Main system orchestrator with real connection testing
│   │   ├── base_system.py           # Abstract base system class interface
│   │   ├── scanner_system.py        # scanCONTROL integration with dynamic SDK loading
│   │   ├── camera_system_updated.py # Intel RealSense camera integration with ROS2
│   │   ├── gantry_system.py         # CRI gantry integration with TCP communication
│   │   ├── table_system.py          # Serial rotating table integration
│   │   └── __init__.py
│   ├── gui/
│   │   ├── main_window.py           # Main PyQt5 interface with enhanced status monitoring
│   │   ├── login_dialog.py          # Secure authentication dialog
│   │   ├── gui_launcher.py          # GUI initialization and setup
│   │   └── __init__.py
│   └── utils/
│       ├── file_manager.py          # Data organization and file management utilities
│       ├── scanner_sdk_loader.py    # Dynamic SDK loading utility for scanCONTROL
│       ├── date_utils.py            # Date/time utilities for data organization
│       └── __init__.py
├── config/
│   └── app_config.yaml              # Centralized system configuration file
├── programs/                        # YAML-based inspection program definitions
│   └── example_inspection.yaml      # Sample inspection program
├── output/                          # Generated inspection data with organized structure
│   └── YYYY-MM-DD/                  # Date-based organization
│       └── program_piece_ref_time/  # Inspection-specific folders
├── logs/                            # Application and system logs
├── requirements.txt                 # Python dependencies including new packages
├── setup.py                        # Package setup configuration
├── test_system_connections.py      # Standalone connection testing utility
└── README.md                       # This comprehensive documentation
```

### Recent Enhancements:
- **Dynamic SDK Loading**: `scanner_sdk_loader.py` enables runtime loading of scanCONTROL SDK
- **Enhanced Security**: bcrypt password hashing in `password_utils.py` and secure database authentication
- **Real Connection Testing**: Updated system classes use actual hardware connection functions
- **Improved GUI**: Enhanced status monitoring with visual indicators and detailed system information
- **Configuration Management**: Centralized YAML-based configuration with validation

## Installation & Setup

### Prerequisites
- **Python 3.8+** with pip package manager
- **PostgreSQL 12+** database server
- **Hardware systems** (scanCONTROL scanners, Intel RealSense cameras, CRI gantry, rotating table) - optional for testing
- **Operating System**: Linux (preferred), Windows 10+, or macOS
- **Network Access**: Ethernet connectivity for networked hardware systems

### 1. Clone Repository
```bash
git clone <repository-url>
cd auas_inspection_engine/scenario_inspector
```

### 2. Install Python Dependencies
```bash
# Install all required packages including new additions
pip install -r requirements.txt

# Key packages installed:
# - PyQt5 (GUI framework)
# - psycopg2-binary (PostgreSQL driver)
# - bcrypt (secure password hashing)
# - pyserial (serial communication)
# - opencv-python (camera operations)
# - pyaml (configuration management)
```

### 3. Database Setup
Create PostgreSQL database and configure credentials:
```sql
-- Create database
CREATE DATABASE auas_inspection;

-- Create user (optional)
CREATE USER auas_user WITH PASSWORD 'secure_password';
GRANT ALL PRIVILEGES ON DATABASE auas_inspection TO auas_user;
```

Update `config/app_config.yaml` with your database settings:
```yaml
database:
  host: "localhost"        # Your PostgreSQL host
  port: 5432              # PostgreSQL port
  username: "auas_user"   # Database username
  password: "secure_password"  # Database password
  database: "auas_inspection"  # Database name
```

### 4. Hardware System Configuration
Update system configurations in `config/app_config.yaml`:

**scanCONTROL Configuration:**
```yaml
systems:
  scanControl:
    llt_path: "/path/to/your/scanCONTROL/SDK/python_bindings"  # CRITICAL: Set correct SDK path
    ip: "192.168.3.2"     # Scanner IP address
    timeout: 30
```

**Camera Configuration:**
```yaml
  camera:
    device_id: "Intel Corp"  # USB device identifier for RealSense
    ros2_enabled: true       # Enable if ROS2 is available
```

**Network Systems:**
```yaml
  gantry:
    ip: "192.168.3.11"      # CRI gantry IP address
    port: 3920              # CRI communication port
  
  table:
    port: "/dev/ttyUSB0"    # Serial port for rotating table
    baudrate: 9600          # Serial communication speed
```

### 5. System Connection Testing
Test all configured systems before first use:
```bash
# Run standalone connection test
python test_system_connections.py

# Expected output:
# ✅ scanControl: Connected successfully
# ✅ camera: Connected successfully  
# ✅ gantry: Connected successfully
# ✅ table: Connected successfully
```

### 6. User Account Setup
Create initial user account (will be prompted on first login):
- Default admin credentials can be set during first database initialization
- Passwords are automatically hashed using bcrypt for security

## Usage

### Starting the Application
```bash
# Start the main application
cd src
python main.py

# Alternative: Start with debug logging
python main.py --debug
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

### System Status Dashboard Features:
- **Automatic Refresh**: Periodic status updates every 30 seconds
- **Manual Refresh**: Instant status check with refresh button
- **Detailed Diagnostics**: Expandable details for each system showing:
  - SDK/library versions (where applicable)
  - Communication protocols and settings
  - Last known good configuration
  - Troubleshooting suggestions

### Creating Inspection Programs

Programs are defined in YAML format with support for complex multi-stage inspections:

```yaml
program:
  name: "Aircraft Wing Inspection"
  description: "Complete wing inspection with laser scanning and photography"
  version: "1.2"
  author: "AUAS Inspection Team"
  
  # Global program settings
  settings:
    max_execution_time: 1800  # 30 minutes
    retry_on_failure: true
    save_intermediate_data: true
    
  # Multi-stage inspection workflow
  stages:
    - stage: 1
      name: "Setup and Positioning"
      description: "Prepare systems and position hardware"
      timeout: 300  # 5 minutes
      steps:
        - step: 1
          name: "Load Gantry Program"
          system: "gantry"
          action: "load_program"
          parameters:
            program_name: "wing_position.xml"
            coordinate_system: "part_frame"
          expected_duration: 30
          
        - step: 2
          name: "Initialize Scanner"
          system: "scanControl"
          action: "initialize"
          parameters:
            profile_frequency: 2000  # Hz
            measurement_range: 25    # mm
            calibration_file: "wing_calibration.cal"
          expected_duration: 45
          
        - step: 3
          name: "Start Table Rotation"
          system: "table"
          action: "rotate"
          parameters:
            speed: 10         # RPM
            direction: "cw"   # clockwise
            continuous: true
          expected_duration: 10
          
    - stage: 2
      name: "Data Acquisition"
      description: "Perform laser scanning and image capture"
      timeout: 900  # 15 minutes
      parallel_execution: true  # Allow parallel operations
      steps:
        - step: 1
          name: "Continuous Laser Scanning"
          system: "scanControl"
          action: "scan_continuous"
          parameters:
            duration: 600     # 10 minutes
            save_profiles: true
            profile_format: "csv"
            output_prefix: "wing_scan"
          expected_duration: 600
          
        - step: 2
          name: "Capture Reference Images"
          system: "camera"
          action: "capture_sequence"
          parameters:
            image_count: 20
            interval: 30      # seconds
            image_format: "jpg"
            resolution: "1920x1080"
            image_prefix: "wing_ref"
          expected_duration: 600
          parallel_with: [1]  # Run parallel with laser scanning
          
    - stage: 3
      name: "Quality Check and Finalization"
      description: "Verify data quality and generate reports"
      timeout: 180  # 3 minutes
      steps:
        - step: 1
          name: "Stop Table Rotation"
          system: "table"
          action: "stop"
          expected_duration: 5
          
        - step: 2
          name: "Return Gantry to Home"
          system: "gantry"
          action: "execute_program"
          parameters:
            program_name: "return_home.xml"
          expected_duration: 60
          
        - step: 3
          name: "Generate Inspection Report"
          system: "scanControl"
          action: "generate_report"
          parameters:
            include_statistics: true
            report_format: "pdf"
            report_name: "wing_inspection_report"
          expected_duration: 30

# Error handling and recovery
error_handling:
  on_system_failure:
    - stop_all_systems: true
    - save_partial_data: true
    - generate_error_report: true
  
  retry_policy:
    max_retries: 3
    retry_delay: 10  # seconds
    
# Data output configuration
output:
  base_directory: "./output"
  create_subdirectories: true
  compression: "zip"
  metadata_file: "inspection_metadata.json"
```

### Advanced Program Features:
- **Parallel Execution**: Steps can run simultaneously for efficiency
- **Error Recovery**: Comprehensive error handling and retry mechanisms
- **Flexible Parameters**: System-specific parameters for fine-tuned control
- **Validation**: Pre-execution validation ensures program compatibility
- **Timeouts**: Individual step and stage timeouts prevent hanging
- **Data Management**: Automatic organization and compression of output data

## Advanced Features

### Dynamic SDK Loading
The system dynamically loads hardware SDKs based on configuration, eliminating hardcoded paths:

```python
# Configuration-driven SDK loading
from utils.scanner_sdk_loader import load_scanner_sdk_from_config

# Loads scanCONTROL SDK from path specified in app_config.yaml
config = load_config()
success = load_scanner_sdk_from_config(config)

if success:
    # SDK is now available for import
    from llt import gocatorsdk as gocator
    
# Benefits:
# - No hardcoded paths in source code
# - Easy deployment across different environments
# - Automatic fallback when SDK unavailable
# - Clear error messages for missing dependencies
```

### Real Hardware Connection Testing
Uses actual hardware connection functions from existing GUI applications:

```python
# Real connection testing, not just network pings
def test_connection(self) -> Dict[str, Any]:
    try:
        # Use actual scanCONTROL connection logic
        if self.scanner_controller.test_connection():
            return {
                'status': 'connected',
                'message': 'Scanner communication verified',
                'details': {
                    'sdk_version': self.get_sdk_version(),
                    'device_info': self.get_device_info(),
                    'calibration_status': self.get_calibration_status()
                }
            }
    except Exception as e:
        return {
            'status': 'error',
            'message': f'Connection failed: {str(e)}',
            'troubleshooting': self.get_troubleshooting_tips()
        }
```

### Enhanced Security Implementation
```python
# bcrypt password hashing with configurable strength
class PasswordUtils:
    def __init__(self, rounds: int = 12):
        self.rounds = rounds
    
    def hash_password(self, password: str) -> str:
        salt = bcrypt.gensalt(rounds=self.rounds)
        return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')
    
    def verify_password(self, password: str, hashed: str) -> bool:
        return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))

# Database integration with secure authentication
def authenticate_user(self, email: str, password: str) -> Optional[User]:
    stored_hash = self.get_user_password_hash(email)
    if self.password_utils.verify_password(password, stored_hash):
        return self.get_user_by_email(email)
    return None
```

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

### System Integration Architecture
```python
# Modular system design with standardized interfaces
class SystemManager:
    def __init__(self, config):
        self.systems = {
            'scanControl': ScannerSystem(config['systems']['scanControl']),
            'camera': CameraSystem(config['systems']['camera']),
            'gantry': GantrySystem(config['systems']['gantry']),
            'table': TableSystem(config['systems']['table'])
        }
    
    def get_all_systems_status(self) -> Dict[str, Dict]:
        """Get real-time status from all configured systems"""
        status = {}
        for name, system in self.systems.items():
            try:
                status[name] = system.test_connection()
            except Exception as e:
                status[name] = {
                    'status': 'error',
                    'message': f'System test failed: {str(e)}'
                }
        return status
```

### Configuration Validation
```python
# Comprehensive configuration validation
def validate_configuration(config: Dict) -> List[str]:
    errors = []
    
    # Validate system configurations
    for system_name, system_config in config.get('systems', {}).items():
        if system_name == 'scanControl':
            llt_path = system_config.get('llt_path')
            if not llt_path or not os.path.exists(llt_path):
                errors.append(f"scanControl llt_path not found: {llt_path}")
        
        # Validate network configurations
        if system_config.get('connection_type') == 'tcp':
            if not system_config.get('ip'):
                errors.append(f"{system_name}: IP address required for TCP connection")
    
    return errors
```

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.