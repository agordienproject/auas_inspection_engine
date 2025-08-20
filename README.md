# AUAS Inspection Engine

A comprehensive inspection system designed for automated quality control using robotics and computer vision. The system consists of two main applications that work together to provide both manual control and automated inspection scenarios.

**Platform:** Windows (converted from Linux/ROS2 to Windows standalone)

## 🚀 Quick Start

**New to this project?** See [QUICK_START.md](QUICK_START.md) for one-command installation!

```bash
git clone https://github.com/agordienproject/auas_inspection_engine.git
cd auas_inspection_engine
python install.py
```

## System Overview

The AUAS Inspection Engine integrates multiple hardware systems:
- **Gantry Robot** - Igus ReBeL robot for precise positioning
- **xArm Robot** - For additional manipulation tasks
- **3D Scanner** - scanCONTROL for surface scanning
- **Camera System** - Intel RealSense for RGBD image capture (no ROS2 required)
- **Rotating Table** - For 360° object inspection

## Main Applications

### 1. GUI Application (`gui_application/`)

A standalone GUI for manual control and configuration of individual systems.

**Purpose:**
- Manual control of individual hardware systems
- System configuration and testing
- Real-time monitoring and status checking
- Quick system validation

**Key Features:**
- Gantry robot control with program execution
- xArm robot control (SDK-based, no ROS2 required)
- 3D scanner configuration and data acquisition
- Intel RealSense camera control with depth capture
- Rotating table control
- System status monitoring

**Entry Point:** `gui_application/gui_launcher.py`

**Dependencies:**
```bash
# Core GUI
PyQt5>=5.15.0

# For scanner functionality
matplotlib>=3.5.0
numpy>=1.21.0
opencv-python>=4.6.0
h5py>=3.7.0

# For xArm control
xarm-python-sdk>=1.16.0

# Intel RealSense camera (Windows)
pyrealsense2>=2.50.0

# Serial communication (table)
pyserial>=3.5

# Windows device management (optional)
WMI>=1.5.1

# Note: scanCONTROL SDK (pylinllt) must be installed separately
```

### 2. Scenario Inspector (`scenario_inspector/`)

A comprehensive inspection automation system with database integration and user management.

**Purpose:**
- Automated inspection scenario execution
- User authentication and access control
- Database logging and result storage
- Coordinated multi-system operations

**Key Features:**
- User login system with encrypted passwords
- YAML-based inspection program definitions
- Multi-threaded program execution
- PostgreSQL database integration
- Comprehensive logging system
- Real-time progress monitoring
- Inspection result archival

**Entry Point:** `scenario_inspector/src/main.py`

**Dependencies:**
```bash
PyQt5>=5.15.0          # GUI framework
PyYAML>=6.0            # YAML program parsing
python-dateutil>=2.8.0 # Date utilities
psycopg2-binary>=2.9.0 # PostgreSQL database
bcrypt>=4.0.0          # Password encryption
pyserial>=3.5          # Serial communication
opencv-python>=4.5.0   # Image processing
pyrealsense2>=2.50.0   # Intel RealSense SDK
numpy>=1.21.0          # Numerical computing
WMI>=1.5.1             # Windows device management (optional)
```

## Hardware Systems

### System Connections

| System | Connection Type | Address/Port | Notes |
|--------|----------------|--------------|-------|
| Gantry Robot | TCP/IP | 192.168.3.11:3920 | Igus ReBeL |
| xArm Robot | TCP/IP | 192.168.1.222 | Direct SDK |
| 3D Scanner | TCP/IP | 192.168.3.2 | scanCONTROL |
| Camera | USB | Intel RealSense | USB 3.0 required, RGBD capture |
| Rotating Table | Serial | COM3 (or similar) | Arduino-based |

### Database Configuration

The system uses PostgreSQL for data persistence:
- **Host:** 145.109.213.243:5433
- **Database:** lab_inspection
- **Features:** User management, inspection logs, results storage

## Installation & Setup

### Prerequisites

1. **Python 3.8+** with pip
2. **PostgreSQL** database access
3. **Intel RealSense SDK** for Windows
4. **Hardware drivers** installed for each system
5. **Windows 10/11** operating system

### Hardware Setup Notes

**Camera (Intel RealSense):**
- Install Intel RealSense SDK from Intel's website
- Ensure USB 3.0 connection for optimal performance
- Use Device Manager to verify camera recognition

**Rotating Table:**
- Connect Arduino via USB
- Identify COM port using Device Manager
- Update `app_config.yaml` with correct COM port

**Network Systems:**
- Ensure all robots are on the same network
- Test connectivity with ping commands

### Installation Steps

1. **Clone the repository:**
   ```bash
   git clone [repository-url]
   cd auas_inspection_engine
   ```

2. **Install Intel RealSense SDK:**
   - Download from Intel's website
   - Install Windows runtime and development packages

3. **Install GUI Application dependencies:**
   ```bash
   cd gui_application\ros2gui
   pip install -r scanner_requirements.txt
   pip install -r xarm_simple_requirements.txt
   ```

4. **Install Scenario Inspector dependencies:**
   ```bash
   cd scenario_inspector
   pip install -r requirements.txt
   ```

5. **Install scanCONTROL SDK:**
   - Download scanCONTROL-Windows-SDK from Micro-Epsilon
   - Install pylinllt Python bindings
   - Update SDK path in `scenario_inspector\config\app_config.yaml`

6. **Configure system paths:**
   - Update all file paths in `app_config.yaml` to Windows format
   - Verify COM port for rotating table
   - Test network connectivity to robots

## Usage

### Running the GUI Application

```powershell
cd gui_application
python gui_launcher.py
```

**Features:**
- Individual system control tabs
- Real-time status monitoring
- Manual program execution
- System configuration
- Intel RealSense camera with depth capture

### Running the Scenario Inspector

```powershell
cd scenario_inspector\src
python main.py
```

**Workflow:**
1. Login with credentials
2. Select inspection program from `programs\` directory
3. Configure inspection parameters
4. Execute automated inspection
5. Review results and logs

## Windows-Specific Changes

### What Was Removed:
- **ROS2 dependencies** - All ROS2 imports and functionality removed
- **Linux system calls** - `lsusb`, `pkill`, etc. replaced with Windows equivalents
- **Unix paths** - All paths converted to Windows format

### What Was Added:
- **Intel RealSense SDK** - Direct camera control with RGBD capture
- **Windows device management** - Using WMI for device detection
- **Windows process management** - Using `taskkill` instead of `pkill`
- **COM port detection** - Windows serial port enumeration

### Camera System Changes:
- **Before:** ROS2 camera nodes with image topics
- **After:** Direct Intel RealSense SDK with `pyrealsense2`
- **New Features:** 
  - Native depth image capture
  - Better performance without ROS2 overhead
  - Direct access to camera parameters

## Program Structure

### Inspection Programs (`scenario_inspector/src/programs/`)

YAML-based program definitions for different inspection scenarios:
- `all_systems_test.yaml` - Complete system integration test
- `camera_photo.yaml` - Simple camera capture
- `gantry_camera_table.yaml` - Coordinated multi-system inspection
- `xarm_simple.yaml` - Basic xArm movement

### System Architecture

```
auas_inspection_engine/
├── gui_application/           # Manual control GUI
│   ├── gui_launcher.py       # Entry point
│   └── ros2gui/              # System control modules
├── scenario_inspector/       # Automated inspection system
│   ├── src/main.py          # Entry point
│   ├── config/              # Configuration files
│   ├── src/systems/         # Hardware system interfaces
│   ├── src/programs/        # Inspection scenarios
│   └── src/gui/             # User interface
├── cri_lib/                 # CRI robot communication
└── notes.md                 # System notes and troubleshooting
```

## Troubleshooting

### Common Issues

**Camera System:**
- Install Intel RealSense SDK before running applications
- Verify camera connection in Device Manager
- Ensure USB 3.0 connection for depth capture
- Use OpenCV fallback if RealSense is unavailable

**COM Port Issues:**
- Check Device Manager for Arduino COM port
- Update `app_config.yaml` with correct COM port (e.g., COM3, COM4)
- Ensure Arduino drivers are installed

**Network Connectivity:**
- Test robot connections with: `ping 192.168.3.11` (gantry), `ping 192.168.1.222` (xArm)
- Verify network adapter configuration
- Check firewall settings for robot communication

**Process Management:**
- Use Task Manager to terminate stuck processes
- Ensure proper cleanup of camera resources
- Restart applications if device conflicts occur

### Device Detection Commands (Windows)

**Check COM ports:**
```powershell
# In PowerShell
Get-WmiObject -Class Win32_SerialPort | Select-Object Name,DeviceID
```

**Check camera devices:**
```powershell
# In PowerShell
Get-WmiObject -Class Win32_PnPEntity | Where-Object {$_.Name -like "*RealSense*"}
```

**Alternative camera check:**
```python
# In Python
import pyrealsense2 as rs
ctx = rs.context()
print(f"Found {len(ctx.query_devices())} RealSense devices")
```

### Logging

The Scenario Inspector creates detailed logs in:
- Daily folders: `output\YYYY-MM-DD\`
- Timestamped files: `inspection_YYYYMMDD_HHMMSS.log`

## Configuration Update Required

**⚠️ IMPORTANT:** You need to update the following paths in your configuration:

1. **Scanner SDK Path** in `scenario_inspector\config\app_config.yaml`:
   ```yaml
   llt_path: "C:\\scanCONTROL-Windows-SDK\\python_bindings"  # Update to your actual path
   ```

2. **COM Port** in `scenario_inspector\config\app_config.yaml`:
   ```yaml
   port: "COM3"  # Update to your actual COM port (check Device Manager)
   ```

3. **Program Paths** in `scenario_inspector\config\app_config.yaml`:
   - Verify all `programs_path` entries use Windows path format
   - Ensure paths point to correct locations on your system

## Development

### Adding New Systems

1. Create system class in `src/systems/`
2. Inherit from `base_system.BaseSystem`
3. Implement required methods
4. Add to `system_manager.py`
5. Update configuration files

### Creating Inspection Programs

1. Create YAML file in `src/programs/`
2. Define steps with system actions
3. Specify parameters and timeouts
4. Test with manual execution

## License

ALEXIS GORDIEN - Hogechool Van Amsterdam

## Contributors

AUAS Engineering Team

---

For detailed system notes and additional troubleshooting, see `notes.md`.
