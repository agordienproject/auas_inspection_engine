# scanCONTROL Scanner Integration for ROS2 GUI

This module provides a complete integration of scanCONTROL laser line sensors into the ROS2 GUI system.

## Features

### 🔌 Connection Management
- **Device Discovery**: Automatically discover available scanCONTROL devices on the network
- **Custom IP Support**: Connect to scanners using custom IP addresses
- **Connection Status**: Real-time connection status monitoring
- **Error Handling**: Robust error handling with detailed error messages

### ⚙️ Parameter Configuration
- **Exposure Time**: Adjust laser exposure time (1-10000 μs)
- **Idle Time**: Configure measurement idle time (100-50000 μs)
- **Trigger Mode**: Select between internal and external trigger modes
- **Real-time Apply**: Apply parameter changes without reconnection

### 📊 Real-time Data Visualization
- **Live Profile Display**: Real-time height and intensity profile plotting
- **Historical Overlay**: View last 50 profiles with fading overlay effect
- **Dual Plot View**: Separate plots for height (Z) and intensity data
- **Auto-scaling**: Automatic axis scaling for optimal visualization
- **Data Filtering**: Automatic filtering of invalid data points

### 💾 Data Recording and Export
- **Continuous Recording**: Record profile data during measurement
- **Multiple Export Formats**:
  - **CSV**: Standard comma-separated values for analysis
  - **NPZ**: Compressed NumPy arrays for Python processing
  - **AVI**: Video format showing profile evolution over time
  - **HDF5**: Hierarchical data format for advanced analysis
- **Recording Statistics**: Profile count, duration, and acquisition rate
- **Timestamp Logging**: Precise timestamp for each recorded profile

### 🎯 Based on Your Tests
The implementation is based on analysis of your test files:
- `test_scanner_detection.py` - Device discovery functionality
- `scanner_data_acquisition.py` - Core data acquisition logic
- `live_scanner_monitor.py` - Real-time monitoring capabilities
- `complete_scanner_test.py` - Parameter configuration methods

## Installation

### Prerequisites
1. **scanCONTROL SDK**: Install the Micro-Epsilon scanCONTROL Linux SDK
2. **Python Dependencies**: Install required Python packages

```bash
pip install -r scanner_requirements.txt
```

### SDK Setup
Ensure the scanCONTROL SDK is available at:
```
/home/agordien/Documents/scanCONTROL-Linux-SDK-1-0-1/python_bindings
```

## Usage

### From ROS2 GUI Main Window
1. Launch the ROS2 GUI application
2. Click "Configure scanCONTROL Scanner" button
3. The scanner window will open with all controls

### Standalone Testing
```bash
cd ros2gui
python3 test_scanner_window.py
```

### Basic Workflow
1. **Connect**: 
   - Discover devices or enter custom IP
   - Click "Connect" to establish connection
   
2. **Configure**:
   - Set exposure time, idle time, and trigger mode
   - Click "Apply Parameters"
   
3. **Measure**:
   - Click "Start Measurement" to begin data acquisition
   - View real-time profiles in the plot area
   
4. **Record** (Optional):
   - Click "Start Recording" to begin data capture
   - Click "Stop Recording" when finished
   - Choose export format and save data

## Hardware Configuration

### Network Setup
- **Scanner IP**: Typically 192.168.3.2 (factory default)
- **PC Network**: Configure PC network interface for scanner subnet
- **Ethernet Connection**: Use direct Ethernet connection for best performance

### Scanner Models Supported
- scanCONTROL 25xx series
- scanCONTROL 26xx series  
- scanCONTROL 27xx series
- scanCONTROL 29xx series
- scanCONTROL 30xx series

## File Structure

```
ros2gui/
├── scanner_config.py          # Main scanner configuration window
├── test_scanner_window.py     # Standalone test script
├── scanner_requirements.txt   # Python dependencies
└── README_scanner.md         # This documentation
```

## Technical Details

### Data Acquisition Thread
- **Threaded Processing**: Data acquisition runs in separate thread
- **Thread Safety**: Qt signals for safe GUI updates
- **Error Resilience**: Automatic error recovery and reporting
- **Rate Control**: Configurable acquisition rate (up to 20 Hz)

### Data Formats

#### Profile Data Structure
Each profile contains:
- `timestamp`: Acquisition timestamp
- `x`: X-coordinate array (mm)
- `z`: Height/Z-coordinate array (mm)  
- `intensity`: Laser intensity array

#### Export Formats
- **CSV**: `timestamp,point_index,x,z,intensity` per row
- **NPZ**: Compressed arrays with metadata
- **AVI**: Video visualization at 10 fps
- **HDF5**: Structured format with metadata and arrays

### Simulation Mode
When no real scanner is connected:
- **Simulated Data**: Generates realistic test profiles
- **Full Functionality**: All features work with simulated data
- **Testing Support**: Perfect for development and testing

## Troubleshooting

### Connection Issues
1. **Check Network**: Verify scanner and PC are on same network
2. **Check IP**: Confirm scanner IP address (use discovery)
3. **Check Power**: Ensure scanner is powered on
4. **Check SDK**: Verify scanCONTROL SDK installation

### Common Error Messages
- `"Error getting interfaces: -X"`: Network connection issue
- `"Error connecting to device: -X"`: Scanner not responding
- `"No scanCONTROL devices detected"`: Check network setup
- `"SDK not available"`: Install scanCONTROL SDK

### Performance Tips
1. **Reduce History**: Lower max_history for better performance
2. **Adjust Rate**: Lower acquisition rate if CPU usage is high
3. **Close Plots**: Minimize plot updates during recording
4. **Use SSD**: Save large recordings to SSD for better performance

## Integration with ROS2 System

The scanner window integrates seamlessly with the ROS2 GUI:
- **Status Indicators**: Connection status shown in main window
- **Hardware Checks**: Automatic SDK availability detection
- **Window Management**: Proper window lifecycle management
- **Resource Cleanup**: Automatic cleanup on application exit

## Future Enhancements

Potential future improvements:
- **ROS2 Publishers**: Publish profile data as ROS2 messages
- **Point Cloud Generation**: Convert profiles to 3D point clouds
- **Multi-Scanner Support**: Support for multiple scanners simultaneously
- **Advanced Filtering**: Additional data filtering and processing options
- **Calibration Tools**: Built-in calibration and measurement tools

## Support

For issues related to:
- **Scanner Hardware**: Contact Micro-Epsilon support
- **SDK Issues**: Check scanCONTROL SDK documentation
- **GUI Issues**: Check console output for error messages
- **Integration Issues**: Verify all dependencies are installed

## License

This integration follows the same license as the main ROS2 GUI project.
The scanCONTROL SDK has its own licensing terms from Micro-Epsilon.
