# AUAS Inspection Engine

A comprehensive inspection system designed for automated quality control using robotics and computer vision. The system consists of two main applications that work together to provide both manual control and automated inspection scenarios.

## System Overview

The AUAS Inspection Engine integrates multiple hardware systems:
- **Gantry Robot** - Igus ReBeL robot for precise positioning
- **xArm Robot** - For additional manipulation tasks
- **3D Scanner** - scanCONTROL for surface scanning
- **Camera System** - Intel RealSense for image capture
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

# Serial communication (table)
pyserial>=3.5

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
```

## Hardware Systems

### System Connections

| System | Connection Type | Address/Port | Notes |
|--------|----------------|--------------|-------|
| Gantry Robot | TCP/IP | 192.168.3.11:3920 | Igus ReBeL |
| xArm Robot | TCP/IP | 192.168.1.222 | Direct SDK |
| 3D Scanner | TCP/IP | 192.168.3.2 | scanCONTROL |
| Camera | USB | Intel RealSense | USB 3.0 required |
| Rotating Table | Serial | /dev/ttyACM0 | Arduino-based |

### Database Configuration

The system uses PostgreSQL for data persistence:
- **Host:** 145.109.213.243:5433
- **Database:** lab_inspection
- **Features:** User management, inspection logs, results storage

## Installation & Setup

### Prerequisites

1. **Python 3.8+** with pip
2. **PostgreSQL** database access
3. **Hardware drivers** installed for each system
4. **VM Configuration** (if applicable):
   - USB 3.0 enabled for camera
   - Proper network bridge for robot communication

### Installation Steps

1. **Clone the repository:**
   ```bash
   git clone [repository-url]
   cd auas_inspection_engine
   ```

2. **Install GUI Application dependencies:**
   ```bash
   cd gui_application/ros2gui
   pip install -r scanner_requirements.txt
   pip install -r xarm_simple_requirements.txt
   ```

3. **Install Scenario Inspector dependencies:**
   ```bash
   cd scenario_inspector
   pip install -r requirements.txt
   ```

4. **Install scanCONTROL SDK:**
   - Download scanCONTROL-Linux-SDK from Micro-Epsilon
   - Install pylinllt Python bindings
   - Update paths in configuration files

5. **Configure database connection:**
   - Update `scenario_inspector/config/app_config.yaml`
   - Ensure database connectivity

## Usage

### Running the GUI Application

```bash
cd gui_application
python gui_launcher.py
```

**Features:**
- Individual system control tabs
- Real-time status monitoring
- Manual program execution
- System configuration

### Running the Scenario Inspector

```bash
cd scenario_inspector/src
python main.py
```

**Workflow:**
1. Login with credentials
2. Select inspection program from `programs/` directory
3. Configure inspection parameters
4. Execute automated inspection
5. Review results and logs

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

**Gantry System:**
- Connect Python program first, then igusRobotControl
- Set execution mode to "single" to avoid loops
- Check TCP connection on port 3920

**USB Devices in VM:**
- Start VM before connecting USB devices
- Use USB 3.0 configuration for camera
- Verify connections with `lsusb` and `ls -la /dev/tty*`

**ROS2 Environment:**
```bash
source /opt/ros/humble/setup.bash
unset GTK_PATH
export QT_QPA_PLATFORM=xcb
```

### Logging

The Scenario Inspector creates detailed logs in:
- Daily folders: `output/YYYY-MM-DD/`
- Timestamped files: `inspection_YYYYMMDD_HHMMSS.log`

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
