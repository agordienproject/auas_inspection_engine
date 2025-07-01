# AUAS Scenario Inspector

## Overview
The AUAS Scenario Inspector is an automated inspection system that executes predefined inspection scenarios using various systems like laser scanners and cameras. It provides a graphical user interface for managing and executing inspection programs with full data organization and database integration.

## Key Features

### 🔧 System Integration
- **Laser Scanner Control**: Integration with scanCONTROL systems for precise measurements
- **Camera Systems**: Video and image capture capabilities with configurable parameters
- **Modular Architecture**: Easy to extend with new inspection systems

### 📋 Scenario Management
- **YAML-based Programs**: Define inspection workflows using simple YAML configuration
- **Multi-stage Execution**: Support for complex inspection processes with multiple stages and steps
- **Real-time Monitoring**: Live progress tracking and logging during execution

### 🔐 User Authentication
- **Secure Login**: bcrypt password hashing with configurable salt rounds
- **User Management**: Role-based access control with user tracking
- **Session Management**: Secure session handling throughout the application

### 📊 Data Management
- **Organized Output**: Automatic folder creation with date/time stamps
- **Database Integration**: PostgreSQL database for inspection records and user management
- **File Organization**: Structured data storage with inspection reports

### 🖥️ User Interface
- **Modern GUI**: PyQt5-based interface with tabbed layout
- **Real-time Logs**: Live execution monitoring with detailed logging
- **History Tracking**: Complete inspection history with search capabilities
- **Configuration Management**: Easy system configuration through the interface

## Project Structure
```
scenario_inspector
├── src
│   ├── __init__.py
│   ├── main.py
│   ├── config
│   │   ├── __init__.py
│   │   └── config_manager.py
│   ├── auth
│   │   ├── __init__.py
│   │   ├── login.py
│   │   └── password_utils.py
│   ├── database
│   │   ├── __init__.py
│   │   ├── connection.py
│   │   └── models.py
│   ├── inspector
│   │   ├── __init__.py
│   │   ├── inspection_engine.py
│   │   └── data_processor.py
│   ├── gui
│   │   ├── __init__.py
│   │   ├── gui_launcher.py
│   │   ├── main_window.py
│   │   └── login_dialog.py
│   └── utils
│       ├── __init__.py
│       ├── file_manager.py
│       └── date_utils.py
├── config
│   └── config.yaml
├── output
│   └── .gitkeep
├── requirements.txt
├── setup.py
└── README.md
```

## Features
- **YAML Configuration**: Load and parse inspection parameters from a YAML file.
- **User Authentication**: Secure login interface with password hashing for user credentials.
- **Data Management**: Organize output data into folders named with the current date.
- **Database Integration**: Store inspection data in a PostgreSQL database for persistence and analysis.

## Installation
1. Clone the repository:
   ```
   git clone <repository-url>
   cd scenario_inspector
   ```
2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage
1. Configure the inspection parameters in `config/config.yaml`.
2. Run the application:
   ```
   python src/main.py
   ```
3. Follow the on-screen instructions to log in and start the inspection process.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.