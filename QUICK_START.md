# AUAS Inspection Engine - Quick Start Guide

## 🚀 One-Command Installation

For a fresh PC setup, simply run:

```bash
python install.py
```

This will automatically:
- ✅ Install all Python dependencies
- ✅ Set up configuration files
- ✅ Create necessary directories
- ✅ Configure environment variables
- ✅ Verify the installation

## 🏃‍♂️ Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/agordienproject/auas_inspection_engine.git
cd auas_inspection_engine
```

### 2. Run Installation
```bash
python install.py
```

### 3. Start the Application
```bash
cd scenario_inspector/src
python main.py
```

### 4. Start FTP Server (Optional)
```bash
cd ftp_server
python server.py
```

## 📋 System Requirements

- **Python**: 3.8 or higher
- **Operating System**: Windows 10/11
- **Memory**: 4GB RAM minimum, 8GB recommended
- **Storage**: 2GB free space

## 🔧 Manual Installation (Alternative)

If you prefer manual installation:

### Scenario Inspector
```bash
cd scenario_inspector
pip install -r requirements.txt
python setup.py install
```

### FTP Server
```bash
cd ftp_server
pip install -r requirements.txt
```

## ⚙️ Configuration

### Environment Variables
The installation creates a `.env` file with default settings:
- FTP server configuration
- Default IP addresses for systems
- Project paths

### Application Configuration
Main configuration is in `scenario_inspector/config/app_config.yaml`:
- System-specific settings
- Connection parameters
- Hardware interfaces

## 🔌 Hardware Systems

The system supports:
- **Camera**: Intel RealSense D435i
- **Scanner**: scanCONTROL LLT (requires vendor SDK)
- **Gantry**: CRI robot controller
- **XArm**: Universal robot arm
- **Rotating Table**: Serial/Arduino controlled

## 🐛 Troubleshooting

### Common Issues

**Import Errors**: Ensure all dependencies are installed
```bash
pip install -r scenario_inspector/requirements.txt
```

**Permission Errors**: Run as administrator if needed

**Hardware Not Detected**: Check configuration in Settings tab

**Scanner SDK**: Install scanCONTROL SDK separately from vendor

## 📞 Support

- Check log files in `scenario_inspector/logs/`
- Review configuration in Settings tab
- Verify hardware connections in System Status tab

## 🎯 Features

- **Multi-System Coordination**: Control multiple inspection systems
- **Automated Scenarios**: Execute pre-defined inspection sequences
- **Real-time Monitoring**: Live system status and connection monitoring
- **Data Management**: Automatic file organization and FTP upload
- **Windows Optimized**: Native Windows support with GUI

---

**Ready to inspect!** 🔍✨
