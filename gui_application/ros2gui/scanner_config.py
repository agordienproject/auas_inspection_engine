#!/usr/bin/env python3
"""
scanCONTROL Scanner Configuration Window for ROS2 GUI

This module provides a complete interface for controlling scanCONTROL sensors:
- Connection management
- Parameter configuration
- Real-time data visualization
- Data recording and saving

Based on the tested scanner functionality from the scanCONTROL-Linux-SDK examples.
"""

import sys
import os
import time
import ctypes as ct
import numpy as np
import cv2
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                             QLabel, QPushButton, QLineEdit, QSpinBox, 
                             QComboBox, QTextEdit, QGroupBox, QCheckBox,
                             QProgressBar, QTabWidget, QSlider, QFileDialog,
                             QMessageBox, QFrame)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QPixmap
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.animation as animation

# Add scanCONTROL SDK path
sys.path.insert(0, '/home/agordien/projects/auas_inspection_engine/scanCONTROL-Linux-SDK-1-0-1/python_bindings')

try:
    import pylinllt as llt
    SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: scanCONTROL SDK not available: {e}")
    SDK_AVAILABLE = False


class ScannerDataThread(QThread):
    """Thread for continuous scanner data acquisition"""
    
    data_ready = pyqtSignal(np.ndarray, np.ndarray, np.ndarray)  # x, z, intensity
    error_occurred = pyqtSignal(str)
    
    def __init__(self, scanner_controller):
        super().__init__()
        self.scanner_controller = scanner_controller
        self.running = False
        self.recording = False
        self.recorded_profiles = []
        
    def start_recording(self):
        """Start recording profiles"""
        self.recording = True
        self.recorded_profiles = []
        
    def stop_recording(self):
        """Stop recording and return data"""
        self.recording = False
        return self.recorded_profiles.copy()
        
    def run(self):
        """Main data acquisition loop"""
        self.running = True
        
        while self.running:
            try:
                if self.scanner_controller.is_connected and self.scanner_controller.is_measuring:
                    x_data, z_data, intensity_data = self.scanner_controller.acquire_profile()
                    
                    if x_data is not None:
                        self.data_ready.emit(x_data, z_data, intensity_data)
                        
                        # Store data if recording
                        if self.recording:
                            timestamp = time.time()
                            self.recorded_profiles.append({
                                'timestamp': timestamp,
                                'x': x_data.copy(),
                                'z': z_data.copy(),
                                'intensity': intensity_data.copy()
                            })
                    
                    # Control acquisition rate
                    time.sleep(0.05)  # 20 Hz max
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                self.error_occurred.emit(str(e))
                time.sleep(0.5)
                
    def stop(self):
        """Stop the data acquisition thread"""
        self.running = False
        self.wait()


class ScannerController:
    """Controller class for scanCONTROL sensor operations"""
    
    def __init__(self):
        self.hLLT = None
        self.scanner_type = ct.c_int(0)
        self.resolution = 640
        self.is_connected = False
        self.is_measuring = False
        self.available_devices = []
        
    def discover_devices(self):
        """Discover available scanCONTROL devices"""
        if not SDK_AVAILABLE:
            return ["192.168.3.2"]  # Fallback for testing
            
        try:
            available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
            available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))
            
            ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
            if ret < 1:
                # Try known IP as fallback
                return ["192.168.3.2"]
                
            discovered_devices = []
            for i in range(ret):
                device_name = available_interfaces[i].value.decode('utf-8')
                if device_name:
                    discovered_devices.append(device_name)
                    
            return discovered_devices if discovered_devices else ["192.168.3.2"]
            
        except Exception:
            return ["192.168.3.2"]
    
    def connect(self, device_interface="192.168.3.2"):
        """Connect to scanCONTROL device"""
        if not SDK_AVAILABLE:
            self.is_connected = True
            return True
            
        try:
            # Create device handle
            self.hLLT = llt.create_llt_device()
            
            # Set device interface
            ret = llt.set_device_interface(self.hLLT, device_interface.encode('utf-8'))
            if ret < 1:
                raise ValueError(f"Error setting device interface: {ret}")
            
            # Connect to device
            ret = llt.connect(self.hLLT)
            if ret < 1:
                raise ConnectionError(f"Error connecting to device: {ret}")
            
            self.is_connected = True
            
            # Get scanner information
            self._get_scanner_info()
            
            return True
            
        except Exception as e:
            if self.hLLT:
                try:
                    llt.del_device(self.hLLT)
                except:
                    pass
                self.hLLT = None
            raise e
    
    def _get_scanner_info(self):
        """Get scanner type and available resolutions"""
        # Get scanner type
        ret = llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type))
        if ret < 1:
            raise ValueError(f"Error getting scanner type: {ret}")
        
        # Get available resolutions
        available_resolutions = (ct.c_uint*4)()
        ret = llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
        if ret < 1:
            raise ValueError(f"Error getting resolutions: {ret}")
        
        resolutions = [available_resolutions[i] for i in range(ret) if available_resolutions[i] > 0]
        
        # Set maximum resolution
        self.resolution = max(resolutions) if resolutions else 640
        ret = llt.set_resolution(self.hLLT, self.resolution)
        if ret < 1:
            raise ValueError(f"Error setting resolution: {ret}")
    
    def configure_measurement(self, exposure_time=100, idle_time=3900, trigger_mode="internal"):
        """Configure measurement parameters"""
        if not SDK_AVAILABLE:
            return True
            
        if not self.is_connected:
            raise ConnectionError("Not connected to scanner")
        
        # Set profile config
        ret = llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
        if ret < 1:
            raise ValueError(f"Error setting profile config: {ret}")
        
        # Set trigger mode
        trigger_val = llt.TRIG_INTERNAL if trigger_mode == "internal" else llt.TRIG_EXTERNAL
        ret = llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_TRIGGER, trigger_val)
        if ret < 1:
            raise ValueError(f"Error setting trigger: {ret}")
        
        # Set exposure time
        ret = llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_time)
        if ret < 1:
            raise ValueError(f"Error setting exposure time: {ret}")
        
        # Set idle time
        ret = llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, idle_time)
        if ret < 1:
            raise ValueError(f"Error setting idle time: {ret}")
        
        return True
    
    def start_measurement(self):
        """Start profile measurement"""
        if not SDK_AVAILABLE:
            self.is_measuring = True
            return True
            
        if not self.is_connected:
            raise ConnectionError("Not connected to scanner")
        
        ret = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError(f"Error starting transfer: {ret}")
        
        self.is_measuring = True
        time.sleep(0.2)  # Allow warm-up time (like in working examples)
        return True
    
    def stop_measurement(self):
        """Stop profile measurement"""
        if not SDK_AVAILABLE:
            self.is_measuring = False
            return True
            
        if not self.is_connected:
            return True
        
        try:
            ret = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
            self.is_measuring = False
            return True
        except:
            self.is_measuring = False
            return False
    
    def acquire_profile(self):
        """Acquire a single profile from the scanner"""
        if not SDK_AVAILABLE:
            # Generate simulated data for testing
            x = np.linspace(-50, 50, 640)
            z = 10 * np.sin(0.1 * x) + np.random.normal(0, 0.5, 640)
            intensity = 1000 + 200 * np.sin(0.2 * x) + np.random.normal(0, 50, 640)
            return x, z, intensity.astype(np.uint16)
            
        if not self.is_connected or not self.is_measuring:
            return None, None, None
        
        # Prepare data buffers - use numpy arrays with ctypes pointers like in examples
        profile_buffer = (ct.c_ubyte*(self.resolution*64))()
        x = np.empty(self.resolution, dtype=float)
        z = np.empty(self.resolution, dtype=float)
        x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
        z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))
        intensities = (ct.c_ushort * self.resolution)()
        lost_profiles = ct.c_uint()
        timestamp = (ct.c_ubyte * 16)()
        
        # Null pointers for data we don't need
        null_ptr_short = ct.POINTER(ct.c_ushort)()
        null_ptr_int = ct.POINTER(ct.c_uint)()
        
        try:
            # Get profile data
            ret = llt.get_actual_profile(self.hLLT, profile_buffer, len(profile_buffer), 
                                       llt.TProfileConfig.PROFILE, ct.byref(lost_profiles))
            
            if ret != len(profile_buffer):
                # Error -104 typically means "no data available" which is normal when no object is present
                if ret == -104:
                    return None, None, None  # No object in field of view
                else:
                    print(f"Scanner data acquisition error: {ret}")
                    return None, None, None
            
            # Convert to values - use the numpy array pointers
            ret = llt.convert_profile_2_values(profile_buffer, len(profile_buffer), self.resolution, 
                                             llt.TProfileConfig.PROFILE, self.scanner_type, 0,
                                             null_ptr_short, intensities, null_ptr_short, 
                                             x_p, z_p, null_ptr_int, null_ptr_int)
            
            # Check for successful conversion including intensity data
            if ret & llt.CONVERT_X == 0 or ret & llt.CONVERT_Z == 0 or ret & llt.CONVERT_MAXIMUM == 0:
                return None, None, None
            
            # Extract timestamp (like in examples)
            for i in range(16):
                timestamp[i] = profile_buffer[self.resolution * 64 - 16 + i]
            
            # Convert intensity array to numpy
            intensity_data = np.array([intensities[i] for i in range(self.resolution)])
            
            return x.copy(), z.copy(), intensity_data
            
        except Exception as e:
            # Uncomment for debugging: print(f"Error in acquire_profile: {e}")
            return None, None, None
    
    def disconnect(self):
        """Disconnect from the scanner"""
        if not SDK_AVAILABLE:
            self.is_connected = False
            return True
            
        if self.hLLT and self.is_connected:
            try:
                if self.is_measuring:
                    self.stop_measurement()
                
                llt.disconnect(self.hLLT)
                llt.del_device(self.hLLT)
                
                self.is_connected = False
                self.hLLT = None
                return True
                
            except Exception:
                self.is_connected = False
                self.hLLT = None
                return False
        
        return True


class PlotWidget(QWidget):
    """Custom widget for real-time plotting"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.reset_data()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(12, 8), facecolor='white')
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        # Create subplots
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)
        
        # Configure plots
        self.ax1.set_title('Height Profile (Z)', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel('X Position (mm)')
        self.ax1.set_ylabel('Z Height (mm)')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_title('Intensity Profile', fontsize=12, fontweight='bold')
        self.ax2.set_xlabel('X Position (mm)')
        self.ax2.set_ylabel('Intensity')
        self.ax2.grid(True, alpha=0.3)
        
        self.figure.tight_layout()
        
        self.setLayout(layout)
        
    def reset_data(self):
        """Reset plot data"""
        self.x_data_history = []
        self.z_data_history = []
        self.intensity_data_history = []
        self.max_history = 50  # Keep last 50 profiles for overlay
        
    def update_plot(self, x_data, z_data, intensity_data):
        """Update plots with new data"""
        # Store data in history
        self.x_data_history.append(x_data)
        self.z_data_history.append(z_data)
        self.intensity_data_history.append(intensity_data)
        
        # Limit history size
        if len(self.x_data_history) > self.max_history:
            self.x_data_history.pop(0)
            self.z_data_history.pop(0)
            self.intensity_data_history.pop(0)
        
        # Clear plots
        self.ax1.clear()
        self.ax2.clear()
        
        # Filter valid data points
        valid_indices = (z_data > -1000) & (z_data < 1000) & (x_data != 0)
        x_valid = x_data[valid_indices]
        z_valid = z_data[valid_indices]
        intensity_valid = intensity_data[valid_indices]
        
        # Plot height profile
        self.ax1.set_title('Height Profile (Z)', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel('X Position (mm)')
        self.ax1.set_ylabel('Z Height (mm)')
        self.ax1.grid(True, alpha=0.3)
        
        if len(x_valid) > 0:
            # Plot historical data as faded lines
            for i, (x_hist, z_hist) in enumerate(zip(self.x_data_history[:-1], self.z_data_history[:-1])):
                alpha = 0.1 + 0.4 * (i / len(self.x_data_history))
                valid_hist = (z_hist > -1000) & (z_hist < 1000) & (x_hist != 0)
                if np.any(valid_hist):
                    self.ax1.plot(x_hist[valid_hist], z_hist[valid_hist], 
                                'g-', alpha=alpha, linewidth=0.5)
            
            # Plot current data prominently
            self.ax1.plot(x_valid, z_valid, 'g-', linewidth=2, 
                         label=f'Current ({len(x_valid)} points)')
            self.ax1.legend()
        
        # Plot intensity profile
        self.ax2.set_title('Intensity Profile', fontsize=12, fontweight='bold')
        self.ax2.set_xlabel('X Position (mm)')
        self.ax2.set_ylabel('Intensity')
        self.ax2.grid(True, alpha=0.3)
        
        if len(x_valid) > 0:
            self.ax2.plot(x_valid, intensity_valid, 'r-', linewidth=2, 
                         label='Intensity')
            self.ax2.legend()
        
        self.figure.tight_layout()
        self.canvas.draw()


class ScannerConfigWindow(QWidget):
    """Main scanner configuration window"""
    
    def __init__(self):
        super().__init__()
        self.scanner = ScannerController()
        self.data_thread = None
        self.setup_ui()
        self.setup_connections()
        self.discover_devices()
        
    def setup_ui(self):
        self.setWindowTitle("scanCONTROL Scanner Configuration")
        self.setGeometry(100, 100, 1400, 900)
        
        # Main layout
        main_layout = QHBoxLayout()
        
        # Left panel for controls
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, 1)
        
        # Right panel for plotting
        self.plot_widget = PlotWidget()
        main_layout.addWidget(self.plot_widget, 2)
        
        self.setLayout(main_layout)
        
    def create_control_panel(self):
        """Create the control panel with tabs"""
        control_widget = QWidget()
        control_widget.setMaximumWidth(450)
        control_layout = QVBoxLayout()
        
        # Create tab widget
        tab_widget = QTabWidget()
        
        # Connection tab
        connection_tab = self.create_connection_tab()
        tab_widget.addTab(connection_tab, "Connection")
        
        # Parameters tab
        parameters_tab = self.create_parameters_tab()
        tab_widget.addTab(parameters_tab, "Parameters")
        
        # Recording tab
        recording_tab = self.create_recording_tab()
        tab_widget.addTab(recording_tab, "Recording")
        
        control_layout.addWidget(tab_widget)
        
        # Status panel
        status_panel = self.create_status_panel()
        control_layout.addWidget(status_panel)
        
        control_widget.setLayout(control_layout)
        return control_widget
        
    def create_connection_tab(self):
        """Create connection configuration tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Device selection
        device_group = QGroupBox("Device Selection")
        device_layout = QGridLayout()
        
        device_layout.addWidget(QLabel("Available Devices:"), 0, 0)
        self.device_combo = QComboBox()
        device_layout.addWidget(self.device_combo, 0, 1)
        
        self.discover_btn = QPushButton("🔍 Discover")
        device_layout.addWidget(self.discover_btn, 0, 2)
        
        device_layout.addWidget(QLabel("Custom IP:"), 1, 0)
        self.custom_ip_edit = QLineEdit("192.168.3.2")
        device_layout.addWidget(self.custom_ip_edit, 1, 1, 1, 2)
        
        device_group.setLayout(device_layout)
        layout.addWidget(device_group)
        
        # Connection controls
        connection_group = QGroupBox("Connection")
        connection_layout = QVBoxLayout()
        
        self.connect_btn = QPushButton("🔌 Connect")
        self.connect_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; }")
        connection_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("❌ Disconnect")
        self.disconnect_btn.setEnabled(False)
        connection_layout.addWidget(self.disconnect_btn)
        
        connection_group.setLayout(connection_layout)
        layout.addWidget(connection_group)
        
        # Scanner info
        info_group = QGroupBox("Scanner Information")
        info_layout = QVBoxLayout()
        
        self.scanner_info_text = QTextEdit()
        self.scanner_info_text.setMaximumHeight(100)
        self.scanner_info_text.setReadOnly(True)
        info_layout.addWidget(self.scanner_info_text)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_parameters_tab(self):
        """Create parameters configuration tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Measurement parameters
        params_group = QGroupBox("Measurement Parameters")
        params_layout = QGridLayout()
        
        # Exposure time
        params_layout.addWidget(QLabel("Exposure Time (μs):"), 0, 0)
        self.exposure_spin = QSpinBox()
        self.exposure_spin.setRange(1, 10000)
        self.exposure_spin.setValue(100)
        params_layout.addWidget(self.exposure_spin, 0, 1)
        
        # Idle time
        params_layout.addWidget(QLabel("Idle Time (μs):"), 1, 0)
        self.idle_spin = QSpinBox()
        self.idle_spin.setRange(100, 50000)
        self.idle_spin.setValue(3900)
        params_layout.addWidget(self.idle_spin, 1, 1)
        
        # Trigger mode
        params_layout.addWidget(QLabel("Trigger Mode:"), 2, 0)
        self.trigger_combo = QComboBox()
        self.trigger_combo.addItems(["internal", "external"])
        params_layout.addWidget(self.trigger_combo, 2, 1)
        
        # Apply button
        self.apply_params_btn = QPushButton("⚙️ Apply Parameters")
        self.apply_params_btn.setEnabled(False)
        params_layout.addWidget(self.apply_params_btn, 3, 0, 1, 2)
        
        params_group.setLayout(params_layout)
        layout.addWidget(params_group)
        
        # Measurement controls
        measurement_group = QGroupBox("Measurement Control")
        measurement_layout = QVBoxLayout()
        
        self.start_measure_btn = QPushButton("🚀 Start Measurement")
        self.start_measure_btn.setEnabled(False)
        self.start_measure_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
        measurement_layout.addWidget(self.start_measure_btn)
        
        self.stop_measure_btn = QPushButton("⏹️ Stop Measurement")
        self.stop_measure_btn.setEnabled(False)
        self.stop_measure_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #f44336; color: white; }")
        measurement_layout.addWidget(self.stop_measure_btn)
        
        measurement_group.setLayout(measurement_layout)
        layout.addWidget(measurement_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_recording_tab(self):
        """Create recording configuration tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Recording controls
        recording_group = QGroupBox("Data Recording")
        recording_layout = QVBoxLayout()
        
        self.record_btn = QPushButton("🔴 Start Recording")
        self.record_btn.setEnabled(False)
        self.record_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #ff6b6b; color: white; }")
        recording_layout.addWidget(self.record_btn)
        
        self.stop_record_btn = QPushButton("⏹️ Stop Recording")
        self.stop_record_btn.setEnabled(False)
        recording_layout.addWidget(self.stop_record_btn)
        
        self.recording_status = QLabel("Not recording")
        recording_layout.addWidget(self.recording_status)
        
        recording_group.setLayout(recording_layout)
        layout.addWidget(recording_group)
        
        # Save options
        save_group = QGroupBox("Save Options")
        save_layout = QGridLayout()
        
        save_layout.addWidget(QLabel("Format:"), 0, 0)
        self.save_format_combo = QComboBox()
        self.save_format_combo.addItems(["CSV", "NPZ (NumPy)", "AVI (Video)", "HDF5"])
        save_layout.addWidget(self.save_format_combo, 0, 1)
        
        self.save_btn = QPushButton("💾 Save Last Recording")
        self.save_btn.setEnabled(False)
        save_layout.addWidget(self.save_btn, 1, 0, 1, 2)
        
        save_group.setLayout(save_layout)
        layout.addWidget(save_group)
        
        # Recording statistics
        stats_group = QGroupBox("Recording Statistics")
        stats_layout = QVBoxLayout()
        
        self.recording_stats = QTextEdit()
        self.recording_stats.setMaximumHeight(120)
        self.recording_stats.setReadOnly(True)
        stats_layout.addWidget(self.recording_stats)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_status_panel(self):
        """Create status panel"""
        status_group = QGroupBox("Status")
        layout = QVBoxLayout()
        
        # Status indicators
        status_layout = QGridLayout()
        
        status_layout.addWidget(QLabel("Connection:"), 0, 0)
        self.connection_status = QLabel("❌ Disconnected")
        status_layout.addWidget(self.connection_status, 0, 1)
        
        status_layout.addWidget(QLabel("Measurement:"), 1, 0)
        self.measurement_status = QLabel("⏹️ Stopped")
        status_layout.addWidget(self.measurement_status, 1, 1)
        
        status_layout.addWidget(QLabel("Recording:"), 2, 0)
        self.record_status = QLabel("⚪ Not recording")
        status_layout.addWidget(self.record_status, 2, 1)
        
        layout.addLayout(status_layout)
        
        # Message log
        layout.addWidget(QLabel("Messages:"))
        self.message_log = QTextEdit()
        self.message_log.setMaximumHeight(100)
        self.message_log.setReadOnly(True)
        layout.addWidget(self.message_log)
        
        status_group.setLayout(layout)
        return status_group
        
    def setup_connections(self):
        """Setup signal connections"""
        self.discover_btn.clicked.connect(self.discover_devices)
        self.connect_btn.clicked.connect(self.connect_scanner)
        self.disconnect_btn.clicked.connect(self.disconnect_scanner)
        self.apply_params_btn.clicked.connect(self.apply_parameters)
        self.start_measure_btn.clicked.connect(self.start_measurement)
        self.stop_measure_btn.clicked.connect(self.stop_measurement)
        self.record_btn.clicked.connect(self.start_recording)
        self.stop_record_btn.clicked.connect(self.stop_recording)
        self.save_btn.clicked.connect(self.save_data)
        
    def discover_devices(self):
        """Discover available scanCONTROL devices"""
        self.log_message("🔍 Discovering devices...")
        
        try:
            devices = self.scanner.discover_devices()
            self.device_combo.clear()
            self.device_combo.addItems(devices)
            
            if devices:
                self.log_message(f"✓ Found {len(devices)} device(s): {', '.join(devices)}")
            else:
                self.log_message("⚠️ No devices found, using default IP")
                self.device_combo.addItem("192.168.3.2")
                
        except Exception as e:
            self.log_message(f"❌ Discovery failed: {e}")
            
    def connect_scanner(self):
        """Connect to selected scanner"""
        device = self.device_combo.currentText() or self.custom_ip_edit.text()
        
        if not device:
            self.log_message("❌ No device selected")
            return
            
        self.log_message(f"🔌 Connecting to {device}...")
        
        try:
            success = self.scanner.connect(device)
            
            if success:
                self.log_message("✓ Connected successfully")
                self.connection_status.setText("✅ Connected")
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                self.apply_params_btn.setEnabled(True)
                
                # Update scanner info
                info = f"Scanner Type: {self.scanner.scanner_type.value}\n"
                info += f"Resolution: {self.scanner.resolution} points\n"
                info += f"Device: {device}"
                self.scanner_info_text.setText(info)
                
            else:
                self.log_message("❌ Connection failed")
                
        except Exception as e:
            self.log_message(f"❌ Connection error: {e}")
            
    def disconnect_scanner(self):
        """Disconnect from scanner"""
        self.log_message("🔌 Disconnecting...")
        
        # Stop measurement and recording first
        if self.scanner.is_measuring:
            self.stop_measurement()
            
        try:
            self.scanner.disconnect()
            self.log_message("✓ Disconnected")
            self.connection_status.setText("❌ Disconnected")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.apply_params_btn.setEnabled(False)
            self.start_measure_btn.setEnabled(False)
            self.scanner_info_text.clear()
            
        except Exception as e:
            self.log_message(f"⚠️ Disconnect error: {e}")
            
    def apply_parameters(self):
        """Apply measurement parameters"""
        if not self.scanner.is_connected:
            self.log_message("❌ Not connected to scanner")
            return
            
        exposure = self.exposure_spin.value()
        idle = self.idle_spin.value()
        trigger = self.trigger_combo.currentText()
        
        self.log_message(f"⚙️ Applying parameters: exposure={exposure}μs, idle={idle}μs, trigger={trigger}")
        
        try:
            success = self.scanner.configure_measurement(exposure, idle, trigger)
            
            if success:
                self.log_message("✓ Parameters applied")
                self.start_measure_btn.setEnabled(True)
            else:
                self.log_message("❌ Failed to apply parameters")
                
        except Exception as e:
            self.log_message(f"❌ Parameter error: {e}")
            
    def start_measurement(self):
        """Start scanner measurement"""
        if not self.scanner.is_connected:
            self.log_message("❌ Not connected to scanner")
            return
            
        self.log_message("🚀 Starting measurement...")
        
        try:
            success = self.scanner.start_measurement()
            
            if success:
                self.log_message("✓ Measurement started")
                self.measurement_status.setText("▶️ Running")
                self.start_measure_btn.setEnabled(False)
                self.stop_measure_btn.setEnabled(True)
                self.record_btn.setEnabled(True)
                
                # Start data acquisition thread
                self.data_thread = ScannerDataThread(self.scanner)
                self.data_thread.data_ready.connect(self.update_plot)
                self.data_thread.error_occurred.connect(self.handle_data_error)
                self.data_thread.start()
                
            else:
                self.log_message("❌ Failed to start measurement")
                
        except Exception as e:
            self.log_message(f"❌ Measurement start error: {e}")
            
    def stop_measurement(self):
        """Stop scanner measurement"""
        self.log_message("⏹️ Stopping measurement...")
        
        # Stop recording if active
        if hasattr(self, 'recording') and self.recording:
            self.stop_recording()
        
        # Stop data thread
        if self.data_thread:
            self.data_thread.stop()
            self.data_thread = None
            
        try:
            self.scanner.stop_measurement()
            self.log_message("✓ Measurement stopped")
            self.measurement_status.setText("⏹️ Stopped")
            self.start_measure_btn.setEnabled(True)
            self.stop_measure_btn.setEnabled(False)
            self.record_btn.setEnabled(False)
            
        except Exception as e:
            self.log_message(f"⚠️ Stop error: {e}")
            
    def start_recording(self):
        """Start data recording"""
        if not self.data_thread:
            self.log_message("❌ No active measurement")
            return
            
        self.log_message("🔴 Starting recording...")
        self.data_thread.start_recording()
        self.recording = True
        self.recording_start_time = time.time()
        
        self.record_btn.setEnabled(False)
        self.stop_record_btn.setEnabled(True)
        self.record_status.setText("🔴 Recording")
        self.recording_status.setText("Recording active...")
        
    def stop_recording(self):
        """Stop data recording"""
        if not hasattr(self, 'recording') or not self.recording:
            return
            
        self.log_message("⏹️ Stopping recording...")
        
        if self.data_thread:
            self.recorded_data = self.data_thread.stop_recording()
            
        self.recording = False
        recording_time = time.time() - self.recording_start_time
        
        self.record_btn.setEnabled(True)
        self.stop_record_btn.setEnabled(False)
        self.record_status.setText("⚪ Not recording")
        self.save_btn.setEnabled(True)
        
        # Update statistics
        stats = f"Recording completed:\n"
        stats += f"Duration: {recording_time:.1f} seconds\n"
        stats += f"Profiles: {len(self.recorded_data)}\n"
        stats += f"Rate: {len(self.recorded_data)/recording_time:.1f} Hz"
        
        self.recording_stats.setText(stats)
        self.recording_status.setText(f"Last recording: {len(self.recorded_data)} profiles")
        
    def save_data(self):
        """Save recorded data"""
        if not hasattr(self, 'recorded_data') or not self.recorded_data:
            self.log_message("❌ No recorded data to save")
            return
            
        format_type = self.save_format_combo.currentText()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        filename, _ = QFileDialog.getSaveFileName(
            self, 
            f"Save {format_type} File",
            f"scanner_data_{timestamp}",
            self.get_file_filter(format_type)
        )
        
        if not filename:
            return
            
        self.log_message(f"💾 Saving {len(self.recorded_data)} profiles as {format_type}...")
        
        try:
            if format_type == "CSV":
                self.save_csv(filename)
            elif format_type == "NPZ (NumPy)":
                self.save_npz(filename)
            elif format_type == "AVI (Video)":
                self.save_avi(filename)
            elif format_type == "HDF5":
                self.save_hdf5(filename)
                
            self.log_message(f"✓ Data saved to {filename}")
            
        except Exception as e:
            self.log_message(f"❌ Save error: {e}")
            
    def get_file_filter(self, format_type):
        """Get file filter for format type"""
        filters = {
            "CSV": "CSV Files (*.csv)",
            "NPZ (NumPy)": "NumPy Files (*.npz)",
            "AVI (Video)": "AVI Files (*.avi)",
            "HDF5": "HDF5 Files (*.h5)"
        }
        return filters.get(format_type, "All Files (*)")
        
    def save_csv(self, filename):
        """Save data as CSV"""
        import csv
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'point_index', 'x', 'z', 'intensity'])
            
            for profile in self.recorded_data:
                timestamp = profile['timestamp']
                for i, (x, z, intensity) in enumerate(zip(profile['x'], profile['z'], profile['intensity'])):
                    writer.writerow([timestamp, i, x, z, intensity])
                    
    def save_npz(self, filename):
        """Save data as NumPy archive"""
        timestamps = [p['timestamp'] for p in self.recorded_data]
        x_data = np.array([p['x'] for p in self.recorded_data])
        z_data = np.array([p['z'] for p in self.recorded_data])
        intensity_data = np.array([p['intensity'] for p in self.recorded_data])
        
        np.savez_compressed(filename,
                          timestamps=timestamps,
                          x_data=x_data,
                          z_data=z_data,
                          intensity_data=intensity_data)
                          
    def save_avi(self, filename):
        """Save data as AVI video"""
        if not self.recorded_data:
            return
            
        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 10
        width, height = 800, 600
        
        out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
        
        # Create frames from profile data
        for profile in self.recorded_data:
            # Create a simple visualization frame
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
            
            x_data = profile['x']
            z_data = profile['z']
            intensity_data = profile['intensity']
            
            # Filter valid data
            valid = (z_data > -1000) & (z_data < 1000) & (x_data != 0)
            
            if np.any(valid):
                ax1.plot(x_data[valid], z_data[valid], 'g-')
                ax1.set_title('Height Profile')
                ax1.set_ylabel('Z (mm)')
                ax1.grid(True)
                
                ax2.plot(x_data[valid], intensity_data[valid], 'r-')
                ax2.set_title('Intensity Profile')
                ax2.set_xlabel('X (mm)')
                ax2.set_ylabel('Intensity')
                ax2.grid(True)
            
            plt.tight_layout()
            
            # Convert to OpenCV frame
            fig.canvas.draw()
            buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            frame = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
            frame = cv2.resize(frame, (width, height))
            
            out.write(frame)
            plt.close(fig)
            
        out.release()
        
    def save_hdf5(self, filename):
        """Save data as HDF5"""
        try:
            import h5py
            
            with h5py.File(filename, 'w') as f:
                # Create groups
                grp = f.create_group('scanner_data')
                
                # Save metadata
                grp.attrs['scanner_type'] = self.scanner.scanner_type.value
                grp.attrs['resolution'] = self.scanner.resolution
                grp.attrs['num_profiles'] = len(self.recorded_data)
                grp.attrs['timestamp'] = datetime.now().isoformat()
                
                # Save data arrays
                timestamps = [p['timestamp'] for p in self.recorded_data]
                x_data = np.array([p['x'] for p in self.recorded_data])
                z_data = np.array([p['z'] for p in self.recorded_data])
                intensity_data = np.array([p['intensity'] for p in self.recorded_data])
                
                grp.create_dataset('timestamps', data=timestamps)
                grp.create_dataset('x_profiles', data=x_data)
                grp.create_dataset('z_profiles', data=z_data)
                grp.create_dataset('intensity_profiles', data=intensity_data)
                
        except ImportError:
            raise ImportError("h5py package required for HDF5 format")
            
    def update_plot(self, x_data, z_data, intensity_data):
        """Update plot with new data"""
        self.plot_widget.update_plot(x_data, z_data, intensity_data)
        
    def handle_data_error(self, error_msg):
        """Handle data acquisition errors"""
        self.log_message(f"⚠️ Data error: {error_msg}")
        
    def log_message(self, message):
        """Log message to message panel"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.message_log.append(formatted_message)
        print(formatted_message)  # Also print to console
        
    def closeEvent(self, event):
        """Handle window close event"""
        if self.data_thread:
            self.data_thread.stop()
            
        if self.scanner.is_connected:
            self.scanner.disconnect()
            
        event.accept()


if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    window = ScannerConfigWindow()
    window.show()
    sys.exit(app.exec_())
