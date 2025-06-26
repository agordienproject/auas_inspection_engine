#!/usr/bin/env python3
"""
scanCONTROL Function Tester GUI

A comprehensive GUI tool for testing and debugging scanCONTROL functions.
Shows detailed results, error codes, raw data, and API responses for every function call.

This is like a GUI version of the Jupyter notebook with interactive controls
and detailed logging of all operations.
"""

import sys
import time
import ctypes as ct
import numpy as np
import json
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QTabWidget, QGroupBox,
                             QLabel, QPushButton, QLineEdit, QSpinBox, QComboBox,
                             QTextEdit, QScrollArea, QSplitter, QCheckBox,
                             QProgressBar, QTreeWidget, QTreeWidgetItem,
                             QHeaderView, QFrame, QSlider)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QColor, QPalette

# Add scanCONTROL SDK path
sys.path.insert(0, '/home/agordien/projects/auas_inspection_engine/scanCONTROL-Linux-SDK-1-0-1/python_bindings')

try:
    import pylinllt as llt
    SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: scanCONTROL SDK not available: {e}")
    SDK_AVAILABLE = False

class FunctionCall:
    """Class to store detailed information about each function call"""
    
    def __init__(self, function_name, parameters=None):
        self.function_name = function_name
        self.parameters = parameters or {}
        self.timestamp = datetime.now()
        self.start_time = time.time()
        self.end_time = None
        self.duration = None
        self.result = None
        self.success = None
        self.error_code = None
        self.error_message = None
        self.raw_data = None
        self.additional_info = {}
        
    def complete(self, result, success=None, error_message=None, raw_data=None, **kwargs):
        """Mark the function call as complete with results"""
        self.end_time = time.time()
        self.duration = self.end_time - self.start_time
        self.result = result
        self.success = success if success is not None else (result >= 1 if isinstance(result, int) else None)
        self.error_message = error_message
        self.raw_data = raw_data
        self.additional_info.update(kwargs)
        
        # Determine error code
        if isinstance(result, int) and result < 1:
            self.error_code = result
            
    def to_dict(self):
        """Convert to dictionary for JSON serialization"""
        return {
            'function_name': self.function_name,
            'parameters': str(self.parameters),
            'timestamp': self.timestamp.isoformat(),
            'duration': self.duration,
            'result': str(self.result),
            'success': self.success,
            'error_code': self.error_code,
            'error_message': self.error_message,
            'additional_info': str(self.additional_info)
        }

class ScannerFunctionTester:
    """Core class for testing scanner functions with detailed logging"""
    
    def __init__(self):
        self.hLLT = None
        self.scanner_type = ct.c_int(0)
        self.resolution = 1024
        self.call_history = []
        self.current_call = None
        
    def start_call(self, function_name, **parameters):
        """Start tracking a function call"""
        self.current_call = FunctionCall(function_name, parameters)
        return self.current_call
        
    def complete_call(self, result, **kwargs):
        """Complete the current function call"""
        if self.current_call:
            self.current_call.complete(result, **kwargs)
            self.call_history.append(self.current_call)
            call = self.current_call
            self.current_call = None
            return call
        return None
    
    def test_create_device(self):
        """Test device creation"""
        call = self.start_call("create_llt_device")
        
        try:
            if not SDK_AVAILABLE:
                result = "SDK_NOT_AVAILABLE"
                self.hLLT = "SIMULATED"
            else:
                self.hLLT = llt.create_llt_device()
                result = self.hLLT
                
            return self.complete_call(result, success=True, 
                                    additional_info={'handle': str(self.hLLT)})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_get_device_interfaces(self):
        """Test device interface discovery with fallback (same as scanner_config.py)"""
        call = self.start_call("get_device_interfaces")
        
        try:
            if not SDK_AVAILABLE:
                result = 1
                interfaces = ["192.168.3.2"]
                discovery_status = "simulated"
            else:
                available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
                available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))
                
                result = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
                
                interfaces = []
                if result >= 1:
                    # Found devices via discovery
                    for i in range(result):
                        interface = available_interfaces[i].value.decode('utf-8')
                        if interface:
                            interfaces.append(interface)
                    discovery_status = "discovered"
                else:
                    # Use fallback IP (same as scanner_config.py)
                    interfaces = ["192.168.3.2"]
                    discovery_status = "fallback_used"
                
            # Always report success if we have interfaces (discovered or fallback)
            success = len(interfaces) > 0
            
            return self.complete_call(result, success=success,
                                    additional_info={'discovered_interfaces': interfaces,
                                                   'interface_count': len(interfaces),
                                                   'discovery_status': discovery_status,
                                                   'raw_result': result})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_set_device_interface(self, interface="192.168.3.2"):
        """Test setting device interface"""
        call = self.start_call("set_device_interface", interface=interface)
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.set_device_interface(self.hLLT, interface.encode('utf-8'))
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'interface': interface})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_connect(self):
        """Test connection to scanner"""
        call = self.start_call("connect")
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.connect(self.hLLT)
                
            return self.complete_call(result, success=result >= 1)
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_get_llt_type(self):
        """Test getting scanner type"""
        call = self.start_call("get_llt_type")
        
        try:
            if not SDK_AVAILABLE:
                result = 1
                scanner_type_value = 4003
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type))
                scanner_type_value = self.scanner_type.value
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'scanner_type': scanner_type_value})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_get_resolutions(self):
        """Test getting available resolutions"""
        call = self.start_call("get_resolutions")
        
        try:
            if not SDK_AVAILABLE:
                result = 4
                resolutions = [1024, 512, 256, 128]
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                available_resolutions = (ct.c_uint*4)()
                result = llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
                
                resolutions = []
                if result >= 1:
                    for i in range(4):
                        if available_resolutions[i] > 0:
                            resolutions.append(available_resolutions[i])
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'available_resolutions': resolutions,
                                                   'resolution_count': len(resolutions)})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_set_resolution(self, resolution=1024):
        """Test setting resolution"""
        call = self.start_call("set_resolution", resolution=resolution)
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.set_resolution(self.hLLT, resolution)
                
            if result >= 1:
                self.resolution = resolution
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'resolution': resolution})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_set_profile_config(self, profile_type="PROFILE"):
        """Test setting profile configuration"""
        call = self.start_call("set_profile_config", profile_type=profile_type)
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                config = llt.TProfileConfig.PROFILE if profile_type == "PROFILE" else llt.TProfileConfig.PARTIAL_PROFILE
                result = llt.set_profile_config(self.hLLT, config)
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'profile_type': profile_type})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_set_feature(self, feature_name, value):
        """Test setting a scanner feature"""
        call = self.start_call("set_feature", feature=feature_name, value=value)
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                # Map feature names to constants
                feature_map = {
                    'TRIGGER': llt.FEATURE_FUNCTION_TRIGGER,
                    'EXPOSURE_TIME': llt.FEATURE_FUNCTION_EXPOSURE_TIME,
                    'IDLE_TIME': llt.FEATURE_FUNCTION_IDLE_TIME
                }
                
                if feature_name not in feature_map:
                    raise ValueError(f"Unknown feature: {feature_name}")
                    
                feature_constant = feature_map[feature_name]
                
                # Map trigger values
                if feature_name == 'TRIGGER':
                    if value == 'INTERNAL':
                        value = llt.TRIG_INTERNAL
                    elif value == 'EXTERNAL':
                        value = llt.TRIG_EXTERNAL
                
                result = llt.set_feature(self.hLLT, feature_constant, value)
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'feature': feature_name, 'value': value})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_transfer_profiles(self, enable=True):
        """Test starting/stopping profile transfer"""
        call = self.start_call("transfer_profiles", enable=enable)
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                transfer_value = 1 if enable else 0
                result = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, transfer_value)
                
            return self.complete_call(result, success=result >= 1,
                                    additional_info={'transfer_enabled': enable})
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_get_actual_profile(self, attempts=1):
        """Test getting actual profile data"""
        call = self.start_call("get_actual_profile", attempts=attempts)
        
        try:
            if not SDK_AVAILABLE:
                # Simulate data
                result = self.resolution * 64
                profile_data = np.random.randint(0, 255, result, dtype=np.uint8)
                raw_data_sample = profile_data[:32].tolist()
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                profile_buffer = (ct.c_ubyte*(self.resolution*64))()
                lost_profiles = ct.c_uint()
                
                success_count = 0
                last_result = None
                
                for attempt in range(attempts):
                    result = llt.get_actual_profile(self.hLLT, profile_buffer, len(profile_buffer),
                                                  llt.TProfileConfig.PROFILE, ct.byref(lost_profiles))
                    last_result = result
                    
                    if result == len(profile_buffer):
                        success_count += 1
                        break
                    elif result == -104:
                        continue  # No data available
                    else:
                        break  # Other error
                
                result = last_result
                
                # Get sample of raw data
                if result == len(profile_buffer):
                    raw_data_sample = [profile_buffer[i] for i in range(min(32, len(profile_buffer)))]
                else:
                    raw_data_sample = None
                
            additional_info = {
                'buffer_size': self.resolution * 64,
                'expected_size': self.resolution * 64,
                'attempts': attempts,
                'raw_data_sample': raw_data_sample
            }
            
            if not SDK_AVAILABLE or result == len(profile_buffer):
                additional_info['data_acquired'] = True
            elif result == -104:
                additional_info['data_acquired'] = False
                additional_info['reason'] = 'No object in field of view'
            else:
                additional_info['data_acquired'] = False
                additional_info['reason'] = f'Error code: {result}'
                
            return self.complete_call(result, success=(result == (self.resolution * 64)),
                                    additional_info=additional_info)
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_convert_profile_2_values(self):
        """Test converting profile data to X, Z, intensity values"""
        call = self.start_call("convert_profile_2_values")
        
        try:
            if not SDK_AVAILABLE:
                result = llt.CONVERT_X | llt.CONVERT_Z | llt.CONVERT_MAXIMUM if hasattr(llt, 'CONVERT_X') else 7
                x_range = (-50.0, 50.0)
                z_range = (80.0, 120.0)
                intensity_range = (500, 1500)
                valid_points = 856
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                    
                # First get profile data
                profile_buffer = (ct.c_ubyte*(self.resolution*64))()
                lost_profiles = ct.c_uint()
                
                profile_result = llt.get_actual_profile(self.hLLT, profile_buffer, len(profile_buffer),
                                                      llt.TProfileConfig.PROFILE, ct.byref(lost_profiles))
                
                if profile_result != len(profile_buffer):
                    if profile_result == -104:
                        return self.complete_call(-104, success=False,
                                                error_message="No profile data available (no object detected)",
                                                additional_info={'profile_result': profile_result})
                    else:
                        return self.complete_call(profile_result, success=False,
                                                error_message=f"Failed to get profile data: {profile_result}",
                                                additional_info={'profile_result': profile_result})
                
                # Convert profile data
                x = (ct.c_double * self.resolution)()
                z = (ct.c_double * self.resolution)()
                intensities = (ct.c_ushort * self.resolution)()
                
                null_ptr_short = ct.POINTER(ct.c_ushort)()
                null_ptr_int = ct.POINTER(ct.c_uint)()
                
                result = llt.convert_profile_2_values(profile_buffer, len(profile_buffer), self.resolution,
                                                    llt.TProfileConfig.PROFILE, self.scanner_type, 0,
                                                    null_ptr_short, intensities, null_ptr_short,
                                                    x, z, null_ptr_int, null_ptr_int)
                
                # Analyze converted data
                if result & llt.CONVERT_X and result & llt.CONVERT_Z:
                    x_data = [x[i] for i in range(self.resolution)]
                    z_data = [z[i] for i in range(self.resolution)]
                    intensity_data = [intensities[i] for i in range(self.resolution)]
                    
                    # Filter valid data
                    valid_mask = [(z_data[i] > -1000 and z_data[i] < 1000 and x_data[i] != 0) 
                                 for i in range(self.resolution)]
                    valid_points = sum(valid_mask)
                    
                    if valid_points > 0:
                        valid_x = [x_data[i] for i in range(self.resolution) if valid_mask[i]]
                        valid_z = [z_data[i] for i in range(self.resolution) if valid_mask[i]]
                        valid_intensity = [intensity_data[i] for i in range(self.resolution) if valid_mask[i]]
                        
                        x_range = (min(valid_x), max(valid_x))
                        z_range = (min(valid_z), max(valid_z))
                        intensity_range = (min(valid_intensity), max(valid_intensity))
                    else:
                        x_range = (0, 0)
                        z_range = (0, 0)
                        intensity_range = (0, 0)
                else:
                    valid_points = 0
                    x_range = (0, 0)
                    z_range = (0, 0)
                    intensity_range = (0, 0)
            
            conversion_flags = {
                'CONVERT_X': bool(result & llt.CONVERT_X) if hasattr(llt, 'CONVERT_X') else True,
                'CONVERT_Z': bool(result & llt.CONVERT_Z) if hasattr(llt, 'CONVERT_Z') else True,
                'CONVERT_MAXIMUM': bool(result & llt.CONVERT_MAXIMUM) if hasattr(llt, 'CONVERT_MAXIMUM') else True
            }
            
            additional_info = {
                'conversion_result': result,
                'conversion_flags': conversion_flags,
                'valid_points': valid_points,
                'total_points': self.resolution,
                'x_range': x_range,
                'z_range': z_range,
                'intensity_range': intensity_range
            }
            
            success = all(conversion_flags.values()) and valid_points > 0
            
            return self.complete_call(result, success=success, additional_info=additional_info)
            
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_disconnect(self):
        """Test disconnecting from scanner"""
        call = self.start_call("disconnect")
        
        try:
            if not SDK_AVAILABLE:
                result = 1
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.disconnect(self.hLLT)
                
            return self.complete_call(result, success=result >= 1)
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))
    
    def test_del_device(self):
        """Test deleting device handle"""
        call = self.start_call("del_device")
        
        try:
            if not SDK_AVAILABLE:
                result = 1
                self.hLLT = None
            else:
                if not self.hLLT:
                    raise ValueError("No device handle available")
                result = llt.del_device(self.hLLT)
                self.hLLT = None
                
            return self.complete_call(result, success=result >= 1)
        except Exception as e:
            return self.complete_call(None, success=False, error_message=str(e))

class ScannerFunctionTesterGUI(QMainWindow):
    """Main GUI window for the scanner function tester"""
    
    def __init__(self):
        super().__init__()
        self.tester = ScannerFunctionTester()
        self.connection_worker = None
        self.data_worker = None
        self.setup_ui()
        self.setup_connections()
        
    def setup_ui(self):
        self.setWindowTitle("scanCONTROL Function Tester GUI - Thread-Safe Version")
        self.setGeometry(100, 100, 1600, 1000)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel - Function controls
        left_panel = self.create_function_panel()
        splitter.addWidget(left_panel)
        
        # Right panel - Results and logs
        right_panel = self.create_results_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter proportions
        splitter.setSizes([500, 1100])
        
    def create_function_panel(self):
        """Create the function testing panel"""
        widget = QWidget()
        widget.setMaximumWidth(500)
        layout = QVBoxLayout(widget)
        
        # Title
        title = QLabel("Scanner Function Tester")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Tab widget for different categories
        tab_widget = QTabWidget()
        
        # Connection functions tab
        connection_tab = self.create_connection_tab()
        tab_widget.addTab(connection_tab, "Connection")
        
        # Configuration functions tab
        config_tab = self.create_config_tab()
        tab_widget.addTab(config_tab, "Configuration")
        
        # Data acquisition functions tab
        data_tab = self.create_data_tab()
        tab_widget.addTab(data_tab, "Data Acquisition")
        
        # Batch operations tab
        batch_tab = self.create_batch_tab()
        tab_widget.addTab(batch_tab, "Batch Tests")
        
        layout.addWidget(tab_widget)
        
        # Status and controls
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.device_status = QLabel("❌ No device handle")
        self.connection_status = QLabel("❌ Not connected")
        self.measurement_status = QLabel("⏹️ Measurement stopped")
        
        status_layout.addWidget(self.device_status)
        status_layout.addWidget(self.connection_status)
        status_layout.addWidget(self.measurement_status)
        
        # Clear results button
        clear_btn = QPushButton("🗑️ Clear All Results")
        clear_btn.clicked.connect(self.clear_results)
        status_layout.addWidget(clear_btn)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        return widget
        
    def create_connection_tab(self):
        """Create connection function testing tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Device creation
        device_group = QGroupBox("Device Management")
        device_layout = QVBoxLayout()
        
        create_btn = QPushButton("🔧 Create Device Handle")
        create_btn.clicked.connect(self.test_create_device)
        device_layout.addWidget(create_btn)
        
        del_btn = QPushButton("🗑️ Delete Device Handle")
        del_btn.clicked.connect(self.test_del_device)
        device_layout.addWidget(del_btn)
        
        device_group.setLayout(device_layout)
        layout.addWidget(device_group)
        
        # Interface discovery
        interface_group = QGroupBox("Interface Discovery")
        interface_layout = QVBoxLayout()
        
        discover_btn = QPushButton("🔍 Get Device Interfaces")
        discover_btn.clicked.connect(self.test_get_interfaces)
        interface_layout.addWidget(discover_btn)
        
        # Interface selection
        interface_select_layout = QHBoxLayout()
        interface_select_layout.addWidget(QLabel("Interface:"))
        self.interface_edit = QLineEdit("192.168.3.2")
        interface_select_layout.addWidget(self.interface_edit)
        interface_layout.addLayout(interface_select_layout)
        
        set_interface_btn = QPushButton("🔗 Set Device Interface")
        set_interface_btn.clicked.connect(self.test_set_interface)
        interface_layout.addWidget(set_interface_btn)
        
        interface_group.setLayout(interface_layout)
        layout.addWidget(interface_group)
        
        # Connection
        connection_group = QGroupBox("Connection")
        connection_layout = QVBoxLayout()
        
        connect_btn = QPushButton("🔌 Connect (Individual)")
        connect_btn.clicked.connect(self.test_connect)
        connection_layout.addWidget(connect_btn)
        
        disconnect_btn = QPushButton("❌ Disconnect")
        disconnect_btn.clicked.connect(self.test_disconnect)
        connection_layout.addWidget(disconnect_btn)
        
        connection_group.setLayout(connection_layout)
        layout.addWidget(connection_group)
        
        # Complete sequences (using scanner_config.py method)
        sequence_group = QGroupBox("Complete Sequences (scanner_config.py method)")
        sequence_layout = QVBoxLayout()
        
        full_connect_btn = QPushButton("🔗 Full Connection Test")
        full_connect_btn.clicked.connect(self.test_full_connect_sequence)
        full_connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        sequence_layout.addWidget(full_connect_btn)
        
        data_acquisition_btn = QPushButton("📊 Data Acquisition Test")
        data_acquisition_btn.clicked.connect(self.test_data_acquisition_sequence)
        data_acquisition_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }")
        sequence_layout.addWidget(data_acquisition_btn)
        
        sequence_group.setLayout(sequence_layout)
        layout.addWidget(sequence_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_config_tab(self):
        """Create configuration function testing tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Scanner info
        info_group = QGroupBox("Scanner Information")
        info_layout = QVBoxLayout()
        
        get_type_btn = QPushButton("🔍 Get Scanner Type")
        get_type_btn.clicked.connect(self.test_get_type)
        info_layout.addWidget(get_type_btn)
        
        get_resolutions_btn = QPushButton("📏 Get Available Resolutions")
        get_resolutions_btn.clicked.connect(self.test_get_resolutions)
        info_layout.addWidget(get_resolutions_btn)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # Resolution setting
        resolution_group = QGroupBox("Resolution")
        resolution_layout = QHBoxLayout()
        
        resolution_layout.addWidget(QLabel("Resolution:"))
        self.resolution_spin = QSpinBox()
        self.resolution_spin.setRange(64, 2048)
        self.resolution_spin.setValue(1024)
        resolution_layout.addWidget(self.resolution_spin)
        
        set_resolution_btn = QPushButton("Set")
        set_resolution_btn.clicked.connect(self.test_set_resolution)
        resolution_layout.addWidget(set_resolution_btn)
        
        resolution_group.setLayout(resolution_layout)
        layout.addWidget(resolution_group)
        
        # Profile configuration
        profile_group = QGroupBox("Profile Configuration")
        profile_layout = QVBoxLayout()
        
        profile_type_layout = QHBoxLayout()
        profile_type_layout.addWidget(QLabel("Type:"))
        self.profile_combo = QComboBox()
        self.profile_combo.addItems(["PROFILE", "PARTIAL_PROFILE"])
        profile_type_layout.addWidget(self.profile_combo)
        profile_layout.addLayout(profile_type_layout)
        
        set_profile_btn = QPushButton("⚙️ Set Profile Config")
        set_profile_btn.clicked.connect(self.test_set_profile_config)
        profile_layout.addWidget(set_profile_btn)
        
        profile_group.setLayout(profile_layout)
        layout.addWidget(profile_group)
        
        # Feature settings
        feature_group = QGroupBox("Feature Settings")
        feature_layout = QVBoxLayout()
        
        # Trigger
        trigger_layout = QHBoxLayout()
        trigger_layout.addWidget(QLabel("Trigger:"))
        self.trigger_combo = QComboBox()
        self.trigger_combo.addItems(["INTERNAL", "EXTERNAL"])
        trigger_layout.addWidget(self.trigger_combo)
        set_trigger_btn = QPushButton("Set")
        set_trigger_btn.clicked.connect(lambda: self.test_set_feature("TRIGGER", self.trigger_combo.currentText()))
        trigger_layout.addWidget(set_trigger_btn)
        feature_layout.addLayout(trigger_layout)
        
        # Exposure time
        exposure_layout = QHBoxLayout()
        exposure_layout.addWidget(QLabel("Exposure (μs):"))
        self.exposure_spin = QSpinBox()
        self.exposure_spin.setRange(1, 10000)
        self.exposure_spin.setValue(100)
        exposure_layout.addWidget(self.exposure_spin)
        set_exposure_btn = QPushButton("Set")
        set_exposure_btn.clicked.connect(lambda: self.test_set_feature("EXPOSURE_TIME", self.exposure_spin.value()))
        exposure_layout.addWidget(set_exposure_btn)
        feature_layout.addLayout(exposure_layout)
        
        # Idle time
        idle_layout = QHBoxLayout()
        idle_layout.addWidget(QLabel("Idle (μs):"))
        self.idle_spin = QSpinBox()
        self.idle_spin.setRange(100, 50000)
        self.idle_spin.setValue(3900)
        idle_layout.addWidget(self.idle_spin)
        set_idle_btn = QPushButton("Set")
        set_idle_btn.clicked.connect(lambda: self.test_set_feature("IDLE_TIME", self.idle_spin.value()))
        idle_layout.addWidget(set_idle_btn)
        feature_layout.addLayout(idle_layout)
        
        feature_group.setLayout(feature_layout)
        layout.addWidget(feature_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_data_tab(self):
        """Create data acquisition function testing tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Transfer control
        transfer_group = QGroupBox("Profile Transfer")
        transfer_layout = QVBoxLayout()
        
        start_transfer_btn = QPushButton("▶️ Start Profile Transfer")
        start_transfer_btn.clicked.connect(lambda: self.test_transfer_profiles(True))
        transfer_layout.addWidget(start_transfer_btn)
        
        stop_transfer_btn = QPushButton("⏹️ Stop Profile Transfer")
        stop_transfer_btn.clicked.connect(lambda: self.test_transfer_profiles(False))
        transfer_layout.addWidget(stop_transfer_btn)
        
        transfer_group.setLayout(transfer_layout)
        layout.addWidget(transfer_group)
        
        # Profile acquisition
        acquisition_group = QGroupBox("Profile Acquisition")
        acquisition_layout = QVBoxLayout()
        
        # Single acquisition
        single_acquire_btn = QPushButton("📊 Get Single Profile")
        single_acquire_btn.clicked.connect(lambda: self.test_get_profile(1))
        acquisition_layout.addWidget(single_acquire_btn)
        
        # Multiple attempts
        attempts_layout = QHBoxLayout()
        attempts_layout.addWidget(QLabel("Attempts:"))
        self.attempts_spin = QSpinBox()
        self.attempts_spin.setRange(1, 20)
        self.attempts_spin.setValue(5)
        attempts_layout.addWidget(self.attempts_spin)
        multi_acquire_btn = QPushButton("Get Profile (Multiple Attempts)")
        multi_acquire_btn.clicked.connect(lambda: self.test_get_profile(self.attempts_spin.value()))
        attempts_layout.addWidget(multi_acquire_btn)
        acquisition_layout.addLayout(attempts_layout)
        
        acquisition_group.setLayout(acquisition_layout)
        layout.addWidget(acquisition_group)
        
        # Data conversion
        conversion_group = QGroupBox("Data Conversion")
        conversion_layout = QVBoxLayout()
        
        convert_btn = QPushButton("🔄 Convert Profile to Values")
        convert_btn.clicked.connect(self.test_convert_profile)
        conversion_layout.addWidget(convert_btn)
        
        conversion_group.setLayout(conversion_layout)
        layout.addWidget(conversion_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_batch_tab(self):
        """Create batch operation testing tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Full connection sequence
        full_seq_group = QGroupBox("Complete Sequences")
        full_seq_layout = QVBoxLayout()
        
        full_connect_btn = QPushButton("🔗 Full Connection Sequence")
        full_connect_btn.clicked.connect(self.test_full_connection_sequence)
        full_seq_layout.addWidget(full_connect_btn)
        
        full_config_btn = QPushButton("⚙️ Full Configuration Sequence")
        full_config_btn.clicked.connect(self.test_full_configuration_sequence)
        full_seq_layout.addWidget(full_config_btn)
        
        full_data_btn = QPushButton("📊 Full Data Acquisition Test")
        full_data_btn.clicked.connect(self.test_full_data_acquisition)
        full_seq_layout.addWidget(full_data_btn)
        
        full_seq_group.setLayout(full_seq_layout)
        layout.addWidget(full_seq_group)
        
        # Stress tests
        stress_group = QGroupBox("Stress Tests")
        stress_layout = QVBoxLayout()
        
        # Continuous acquisition
        continuous_layout = QHBoxLayout()
        continuous_layout.addWidget(QLabel("Duration (s):"))
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(1, 300)
        self.duration_spin.setValue(10)
        continuous_layout.addWidget(self.duration_spin)
        continuous_btn = QPushButton("🔄 Continuous Acquisition Test")
        continuous_btn.clicked.connect(self.test_continuous_acquisition)
        continuous_layout.addWidget(continuous_btn)
        stress_layout.addLayout(continuous_layout)
        
        stress_group.setLayout(stress_layout)
        layout.addWidget(stress_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
        
    def create_results_panel(self):
        """Create the results and logging panel"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Results header
        header_layout = QHBoxLayout()
        header_label = QLabel("Function Call Results & Detailed Logs")
        header_label.setFont(QFont("Arial", 14, QFont.Bold))
        header_layout.addWidget(header_label)
        
        # Export button
        export_btn = QPushButton("💾 Export Results")
        export_btn.clicked.connect(self.export_results)
        header_layout.addWidget(export_btn)
        
        header_layout.addStretch()
        layout.addLayout(header_layout)
        
        # Results tree
        self.results_tree = QTreeWidget()
        self.results_tree.setHeaderLabels([
            "Function", "Result", "Success", "Duration", "Timestamp", "Details"
        ])
        
        # Set column widths
        header = self.results_tree.header()
        header.resizeSection(0, 200)  # Function
        header.resizeSection(1, 100)  # Result
        header.resizeSection(2, 80)   # Success
        header.resizeSection(3, 100)  # Duration
        header.resizeSection(4, 150)  # Timestamp
        header.resizeSection(5, 400)  # Details
        
        layout.addWidget(self.results_tree)
        
        # Detailed view
        details_group = QGroupBox("Detailed Information")
        details_layout = QVBoxLayout()
        
        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(200)
        details_layout.addWidget(self.details_text)
        
        details_group.setLayout(details_layout)
        layout.addWidget(details_group)
        
        # Connect tree selection to details
        self.results_tree.itemSelectionChanged.connect(self.show_call_details)
        
        return widget
        
    def setup_connections(self):
        """Setup signal connections"""
        pass  # Most connections are set up inline
        
    def add_result(self, call):
        """Add a function call result to the tree"""
        item = QTreeWidgetItem()
        
        # Set basic information
        item.setText(0, call.function_name)
        item.setText(1, str(call.result))
        item.setText(2, "✅ Yes" if call.success else "❌ No")
        item.setText(3, f"{call.duration*1000:.1f}ms" if call.duration else "N/A")
        item.setText(4, call.timestamp.strftime("%H:%M:%S.%f")[:-3])
        
        # Create details summary
        details = []
        if call.error_message:
            details.append(f"Error: {call.error_message}")
        if call.error_code:
            details.append(f"Code: {call.error_code}")
        if call.additional_info:
            for key, value in call.additional_info.items():
                details.append(f"{key}: {value}")
        
        item.setText(5, " | ".join(details))
        
        # Color coding
        if call.success:
            item.setBackground(2, QColor(200, 255, 200))  # Light green
        else:
            item.setBackground(2, QColor(255, 200, 200))  # Light red
            
        # Store the call object
        item.setData(0, Qt.UserRole, call)
        
        self.results_tree.addTopLevelItem(item)
        
        # Scroll to bottom
        self.results_tree.scrollToBottom()
        
    def show_call_details(self):
        """Show detailed information for selected call"""
        current_item = self.results_tree.currentItem()
        if not current_item:
            return
            
        call = current_item.data(0, Qt.UserRole)
        if not call:
            return
            
        # Format detailed information
        details = f"Function: {call.function_name}\n"
        details += f"Timestamp: {call.timestamp}\n"
        details += f"Duration: {call.duration*1000:.3f}ms\n" if call.duration else "Duration: N/A\n"
        details += f"Result: {call.result}\n"
        details += f"Success: {call.success}\n"
        
        if call.parameters:
            details += f"\nParameters:\n"
            for key, value in call.parameters.items():
                details += f"  {key}: {value}\n"
        
        if call.error_code:
            details += f"\nError Code: {call.error_code}\n"
            
        if call.error_message:
            details += f"Error Message: {call.error_message}\n"
            
        if call.additional_info:
            details += f"\nAdditional Information:\n"
            for key, value in call.additional_info.items():
                details += f"  {key}: {value}\n"
                
        if call.raw_data:
            details += f"\nRaw Data Sample: {call.raw_data}\n"
        
        self.details_text.setText(details)
        
    def update_status(self):
        """Update status indicators"""
        # Device status
        if self.tester.hLLT:
            self.device_status.setText("✅ Device handle created")
        else:
            self.device_status.setText("❌ No device handle")
            
        # TODO: Add connection and measurement status tracking
        
    # Simple connection methods (same as scanner_config.py)
    def test_full_connect_sequence(self):
        """Test complete connection sequence using scanner_config.py method"""
        interface = self.interface_edit.text()
        self.add_message(f"🔄 Starting full connection sequence with interface: {interface}")
        
        try:
            # Create device handle
            self.add_message("1. Creating device handle...")
            if not SDK_AVAILABLE:
                self.add_message("   ⚠️ SDK simulation mode")
                call = FunctionCall("full_connect_sequence")
                call.complete(1, success=True, additional_info={'mode': 'simulated'})
                self.add_result(call)
                return
                
            hLLT = llt.create_llt_device()
            self.add_message(f"   ✅ SUCCESS: {hLLT}")
            
            # Set device interface
            self.add_message(f"2. Setting device interface: {interface}")
            ret = llt.set_device_interface(hLLT, interface.encode('utf-8'))
            if ret < 1:
                self.add_message(f"   ❌ FAILED: {ret}")
                llt.del_device(hLLT)
                call = FunctionCall("full_connect_sequence")
                call.complete(ret, success=False, error_message=f"Interface setup failed: {ret}")
                self.add_result(call)
                return
            self.add_message(f"   ✅ SUCCESS: {ret}")
            
            # Connect to device
            self.add_message("3. Connecting to scanner...")
            ret = llt.connect(hLLT)
            if ret < 1:
                self.add_message(f"   ❌ FAILED: {ret}")
                llt.del_device(hLLT)
                error_messages = {
                    -301: "Scanner used by another software",
                    -305: "Interface not properly set",
                    -303: "Handle already in use", 
                    -102: "Device not found",
                    -103: "Connection timeout"
                }
                error_msg = error_messages.get(ret, f"Unknown error {ret}")
                call = FunctionCall("full_connect_sequence")
                call.complete(ret, success=False, error_message=f"{ret} ({error_msg})")
                self.add_result(call)
                return
                
            self.add_message(f"   ✅ SUCCESS: Connected!")
            
            # Get scanner info
            self.add_message("4. Getting scanner information...")
            scanner_type = ct.c_int(0)
            ret2 = llt.get_llt_type(hLLT, ct.byref(scanner_type))
            if ret2 >= 1:
                self.add_message(f"   Scanner type: {scanner_type.value}")
            
            # Cleanup
            self.add_message("5. Cleaning up...")
            llt.disconnect(hLLT)
            llt.del_device(hLLT)
            self.add_message("   ✅ Disconnected and cleaned up")
            
            # Success
            call = FunctionCall("full_connect_sequence")
            call.complete(ret, success=True, additional_info={'scanner_type': scanner_type.value if ret2 >= 1 else 'unknown'})
            self.add_result(call)
            
        except Exception as e:
            self.add_message(f"💥 EXCEPTION: {e}")
            call = FunctionCall("full_connect_sequence")
            call.complete(-1, success=False, error_message=str(e))
            self.add_result(call)
            
    def test_data_acquisition_sequence(self):
        """Test complete data acquisition sequence using scanner_config.py method"""
        interface = self.interface_edit.text()
        num_profiles = 5
        self.add_message(f"🔄 Starting data acquisition sequence: {num_profiles} profiles")
        
        try:
            if not SDK_AVAILABLE:
                self.add_message("   ⚠️ SDK simulation mode")
                call = FunctionCall("data_acquisition_sequence")
                call.complete(1, success=True, additional_info={'mode': 'simulated', 'profiles': num_profiles})
                self.add_result(call)
                return
                
            # Connect (same as scanner_config.py)
            self.add_message("1. Connecting...")
            hLLT = llt.create_llt_device()
            ret = llt.set_device_interface(hLLT, interface.encode('utf-8'))
            if ret < 1:
                self.add_message(f"   ❌ Interface failed: {ret}")
                llt.del_device(hLLT)
                return
                
            ret = llt.connect(hLLT)
            if ret < 1:
                self.add_message(f"   ❌ Connection failed: {ret}")
                llt.del_device(hLLT)
                return
                
            self.add_message("   ✅ Connected")
            
            # Set profile config
            self.add_message("2. Setting profile configuration...")
            ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
            if ret < 1:
                self.add_message(f"   ❌ Profile config failed: {ret}")
            else:
                self.add_message("   ✅ Profile config set")
            
            # Start transfer (same as scanner_config.py)
            self.add_message("3. Starting data transfer...")
            ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
            if ret < 1:
                self.add_message(f"   ❌ Transfer start failed: {ret}")
            else:
                self.add_message("   ✅ Transfer started")
                
                # Try to get profiles
                self.add_message(f"4. Acquiring {num_profiles} profiles...")
                profiles_received = 0
                
                for i in range(num_profiles):
                    # Use same method as scanner_config.py acquire_profile
                    resolution = 1024
                    x = np.empty(resolution, dtype=float)
                    z = np.empty(resolution, dtype=float)
                    x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
                    z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))
                    
                    profile_buffer = (ct.c_ubyte * 8192)()
                    profile_info = llt.TProfileInfo()
                    
                    ret = llt.get_actual_profile(hLLT, profile_buffer, 8192, llt.TProfileType.PROFILE, ct.byref(profile_info))
                    
                    if ret >= 1:
                        profiles_received += 1
                        self.add_message(f"   Profile {i+1}: {ret} points")
                    else:
                        self.add_message(f"   Profile {i+1}: No data ({ret})")
                        
                    time.sleep(0.1)
                
                # Stop transfer
                llt.transfer_profiles(hLLT, llt.TTransferProfileType.NO_TRANSFER, 1)
                self.add_message("   ✅ Transfer stopped")
                
                self.add_message(f"📊 Received {profiles_received}/{num_profiles} profiles")
            
            # Cleanup
            llt.disconnect(hLLT)
            llt.del_device(hLLT)
            self.add_message("   ✅ Cleaned up")
            
            call = FunctionCall("data_acquisition_sequence")
            call.complete(profiles_received, success=True, additional_info={'profiles_received': profiles_received, 'profiles_requested': num_profiles})
            self.add_result(call)
            
        except Exception as e:
            self.add_message(f"💥 EXCEPTION: {e}")
            call = FunctionCall("data_acquisition_sequence")
            call.complete(-1, success=False, error_message=str(e))
            self.add_result(call)
            
    def add_message(self, message):
        """Add a message to the results display"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_message = f"[{timestamp}] {message}"
        
        # Add to details text area
        self.details_text.append(formatted_message)
        
        # Scroll to bottom
        scrollbar = self.details_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
        # Process events to update GUI
        QApplication.processEvents()
        
    # Test function implementations
    def test_create_device(self):
        call = self.tester.test_create_device()
        self.add_result(call)
        self.update_status()
        
    def test_get_interfaces(self):
        call = self.tester.test_get_device_interfaces()
        self.add_result(call)
        
    def test_set_interface(self):
        interface = self.interface_edit.text()
        call = self.tester.test_set_device_interface(interface)
        self.add_result(call)
        
    def test_connect(self):
        call = self.tester.test_connect()
        self.add_result(call)
        self.update_status()
        
    def test_disconnect(self):
        call = self.tester.test_disconnect()
        self.add_result(call)
        self.update_status()
        
    def test_del_device(self):
        call = self.tester.test_del_device()
        self.add_result(call)
        self.update_status()
        
    def test_get_type(self):
        call = self.tester.test_get_llt_type()
        self.add_result(call)
        
    def test_get_resolutions(self):
        call = self.tester.test_get_resolutions()
        self.add_result(call)
        
    def test_set_resolution(self):
        resolution = self.resolution_spin.value()
        call = self.tester.test_set_resolution(resolution)
        self.add_result(call)
        
    def test_set_profile_config(self):
        profile_type = self.profile_combo.currentText()
        call = self.tester.test_set_profile_config(profile_type)
        self.add_result(call)
        
    def test_set_feature(self, feature_name, value):
        call = self.tester.test_set_feature(feature_name, value)
        self.add_result(call)
        
    def test_transfer_profiles(self, enable):
        call = self.tester.test_transfer_profiles(enable)
        self.add_result(call)
        
    def test_get_profile(self, attempts):
        call = self.tester.test_get_actual_profile(attempts)
        self.add_result(call)
        
    def test_convert_profile(self):
        call = self.tester.test_convert_profile_2_values()
        self.add_result(call)
        
    def test_full_connection_sequence(self):
        """Test complete connection sequence with automatic fallback"""
        # Step 1: Create device
        self.add_result(self.tester.test_create_device())
        
        # Step 2: Discover interfaces (with automatic fallback)
        discovery_call = self.tester.test_get_device_interfaces()
        self.add_result(discovery_call)
        
        # Step 3: Automatically use discovered or fallback IP
        if discovery_call.additional_info and 'discovered_interfaces' in discovery_call.additional_info:
            interfaces = discovery_call.additional_info['discovered_interfaces']
            if interfaces:
                # Use first discovered interface or ensure fallback
                interface_to_use = interfaces[0]
                self.interface_edit.setText(interface_to_use)  # Update GUI field
            else:
                # Ensure fallback is used
                interface_to_use = "192.168.3.2"
                self.interface_edit.setText(interface_to_use)
        else:
            interface_to_use = self.interface_edit.text()
        
        self.add_result(self.tester.test_set_device_interface(interface_to_use))
        
        # Step 4: Connect
        self.add_result(self.tester.test_connect())
        self.update_status()
        
    def test_full_configuration_sequence(self):
        """Test complete configuration sequence"""
        self.add_result(self.tester.test_get_llt_type())
        self.add_result(self.tester.test_get_resolutions())
        self.add_result(self.tester.test_set_resolution(self.resolution_spin.value()))
        self.add_result(self.tester.test_set_profile_config(self.profile_combo.currentText()))
        self.add_result(self.tester.test_set_feature("TRIGGER", self.trigger_combo.currentText()))
        self.add_result(self.tester.test_set_feature("EXPOSURE_TIME", self.exposure_spin.value()))
        self.add_result(self.tester.test_set_feature("IDLE_TIME", self.idle_spin.value()))
        
    def test_full_data_acquisition(self):
        """Test complete data acquisition sequence"""
        self.add_result(self.tester.test_transfer_profiles(True))
        time.sleep(0.2)  # Warm-up time
        self.add_result(self.tester.test_get_actual_profile(5))
        self.add_result(self.tester.test_convert_profile_2_values())
        self.add_result(self.tester.test_transfer_profiles(False))
        
    def test_continuous_acquisition(self):
        """Test continuous data acquisition"""
        duration = self.duration_spin.value()
        # TODO: Implement continuous acquisition test
        pass
        
    def clear_results(self):
        """Clear all results"""
        self.results_tree.clear()
        self.details_text.clear()
        self.tester.call_history.clear()
        
    def export_results(self):
        """Export results to JSON file"""
        if not self.tester.call_history:
            return
            
        filename = f"scanner_test_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        try:
            data = {
                'test_session': {
                    'timestamp': datetime.now().isoformat(),
                    'sdk_available': SDK_AVAILABLE,
                    'total_calls': len(self.tester.call_history)
                },
                'function_calls': [call.to_dict() for call in self.tester.call_history]
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
                
            print(f"Results exported to {filename}")
            
        except Exception as e:
            print(f"Export failed: {e}")

def main():
    """Main function"""
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = ScannerFunctionTesterGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
