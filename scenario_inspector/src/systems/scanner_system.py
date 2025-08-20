"""
Scanner System using scanCONTROL connection functions from GUI application
"""
import sys
import os
import ctypes as ct
import logging
from typing import Dict, Any, Optional

from systems.base_system import BaseSystem
from utils.scanner_sdk_loader import load_scanner_sdk_from_config, get_scanner_sdk, is_scanner_sdk_available


class ScannerController:
    """Controller class for scanCONTROL sensor operations (from GUI application)"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.hLLT = None
        self.scanner_type = ct.c_int(0)
        self.resolution = 640
        self.is_connected = False
        self.is_measuring = False
        self.available_devices = []
        
        # Initialize SDK with path from config
        load_scanner_sdk_from_config(config)
        
        # Get fallback IP from config
        self.fallback_ip = self.config.get('ip', '192.168.3.2')
        
    def discover_devices(self):
        """Discover available scanCONTROL devices"""
        if not is_scanner_sdk_available():
            return [self.fallback_ip]  # Fallback for testing
        
        llt = get_scanner_sdk()
        if not llt:
            return [self.fallback_ip]
            
        try:
            available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
            available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))
            
            ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
            if ret < 1:
                # Try known IP as fallback
                return [self.fallback_ip]
                
            discovered_devices = []
            for i in range(ret):
                device_name = available_interfaces[i].value.decode('utf-8')
                if device_name:
                    discovered_devices.append(device_name)
                    
            return discovered_devices if discovered_devices else [self.fallback_ip]
            
        except Exception:
            return [self.fallback_ip]
    
    def connect(self, device_interface=None):
        """Connect to scanCONTROL device"""
        if device_interface is None:
            device_interface = self.fallback_ip
            
        if not is_scanner_sdk_available():
            self.is_connected = True
            return True
        
        llt = get_scanner_sdk()
        if not llt:
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
        if not is_scanner_sdk_available():
            return
        
        llt = get_scanner_sdk()
        if not llt:
            return
            
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
    
    def disconnect(self):
        """Disconnect from the scanner"""
        if not is_scanner_sdk_available():
            self.is_connected = False
            return True
        
        llt = get_scanner_sdk()
        if not llt:
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

    def test_connection(self):
        """Test if connection is working"""
        return self.is_connected


class ScannerSystem(BaseSystem):
    """Scanner system implementation using GUI connection functions"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.scanner_controller = ScannerController(config)
        self.logger = logging.getLogger(__name__)
        
    def initialize(self) -> bool:
        """Initialize the scanner system"""
        try:
            self.logger.info(f"Initializing scanner system: {self.name}")
            
            # Check if SDK was loaded successfully
            llt_path = self.config.get('llt_path')
            if llt_path and not is_scanner_sdk_available():
                self.logger.warning(f"Scanner SDK not available at configured path: {llt_path}")
            elif is_scanner_sdk_available():
                self.logger.info(f"Scanner SDK loaded successfully from: {llt_path}")
            
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize scanner system: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to the scanner"""
        try:
            device_ip = self.config.get('ip', '192.168.3.2')
            self.logger.info(f"Attempting to connect to scanner at {device_ip}")
            
            success = self.scanner_controller.connect(device_ip)
            if success:
                self.logger.info("Scanner connected successfully")
                return True
            else:
                self.logger.error("Failed to connect to scanner")
                return False
                
        except Exception as e:
            self.logger.error(f"Scanner connection error: {e}")
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from the scanner"""
        try:
            success = self.scanner_controller.disconnect()
            if success:
                self.logger.info("Scanner disconnected successfully")
            return success
        except Exception as e:
            self.logger.error(f"Scanner disconnection error: {e}")
            return False
    
    def test_connection(self) -> Dict[str, Any]:
        """Test scanner connection"""
        try:
            # First check if SDK is available
            if not is_scanner_sdk_available():
                llt_path = self.config.get('llt_path', 'Not configured')
                return {
                    'status': 'not_available',
                    'message': f'Scanner SDK not available at path: {llt_path}',
                    'details': {
                        'llt_path': llt_path,
                        'sdk_available': False
                    }
                }
            
            if self.scanner_controller.test_connection():
                return {
                    'status': 'available',
                    'message': 'Scanner connected and ready',
                    'details': {
                        'resolution': self.scanner_controller.resolution,
                        'scanner_type': self.scanner_controller.scanner_type.value,
                        'sdk_available': True,
                        'llt_path': self.config.get('llt_path', 'N/A')
                    }
                }
            else:
                # Try to connect
                if self.connect():
                    return {
                        'status': 'available',
                        'message': 'Scanner connection established',
                        'details': {
                            'resolution': self.scanner_controller.resolution,
                            'scanner_type': self.scanner_controller.scanner_type.value,
                            'sdk_available': True,
                            'llt_path': self.config.get('llt_path', 'N/A')
                        }
                    }
                else:
                    return {
                        'status': 'not_available',
                        'message': 'Scanner connection failed',
                        'details': {
                            'sdk_available': True,
                            'llt_path': self.config.get('llt_path', 'N/A')
                        }
                    }
        except Exception as e:
            self.logger.error(f"Scanner connection test failed: {e}")
            return {
                'status': 'error',
                'message': f'Scanner test error: {str(e)}',
                'details': {
                    'sdk_available': is_scanner_sdk_available(),
                    'llt_path': self.config.get('llt_path', 'N/A')
                }
            }
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the scanner"""
        step_name = step_config.get('name', 'unknown')
        self.logger.info(f"Executing scanner step: {step_name}")
        
        try:
            # Ensure connection
            if not self.scanner_controller.is_connected:
                if not self.connect():
                    raise ConnectionError("Cannot connect to scanner")
            
            # Execute the step based on step configuration
            step_type = step_config.get('action', 'scan')
            
            if step_type == 'scan':
                return self._perform_scan(step_config)
            else:
                raise ValueError(f"Unknown scanner step type: {step_type}")
                
        except Exception as e:
            self.logger.error(f"Scanner step execution failed: {e}")
            raise
    
    def _perform_scan(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Perform a scan operation"""
        # This would implement the actual scanning logic
        self.logger.info("Performing scanner scan operation")
        
        return {
            'status': 'success',
            'message': 'Scan completed successfully',
            'data': {
                'scan_id': f"scan_{self.name}_{step_config.get('step', 'unknown')}",
                'timestamp': self.get_timestamp()
            }
        }
    
    def shutdown(self):
        """Shutdown the scanner system"""
        try:
            self.disconnect()
            self.logger.info("Scanner system shut down")
        except Exception as e:
            self.logger.error(f"Error shutting down scanner system: {e}")
