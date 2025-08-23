"""
Scan Control System for laser scanning operations
"""

import time
import os
from typing import Dict, Any
from systems.base_system import BaseSystem
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../python_bindings')))
import sys
import os
import importlib.util
# Robust import for pyllt (dev and PyInstaller-frozen)
def _import_pyllt():
    try:
        import pyllt
        return pyllt
    except ImportError:
        # Try to find libs/python_bindings up to 3 levels up from exe or script
        if hasattr(sys, '_MEIPASS'):
            base_dir = os.path.dirname(sys.executable)
        else:
            base_dir = os.path.dirname(getattr(sys.modules['__main__'], '__file__', os.path.abspath(__file__)))
        found = False
        for _ in range(4):  # Check current + 3 levels up
            candidate = os.path.join(base_dir, 'libs', 'python_bindings')
            if os.path.isdir(candidate):
                if candidate not in sys.path:
                    sys.path.insert(0, candidate)
                found = True
                break
            parent = os.path.dirname(base_dir)
            if parent == base_dir:
                break
            base_dir = parent
        if not found:
            raise ImportError('Cannot find libs/python_bindings directory in any of 3 parent folders')
        try:
            import pyllt
            return pyllt
        except ImportError:
            raise ImportError('Cannot find pyllt after adding libs/python_bindings to sys.path')
llt = _import_pyllt()
import ctypes as ct

class ScanControlSystem(BaseSystem):
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.hLLT = None
        self.resolution = None
        self.scanner_type = None
        self.connected = False

    def test_connection(self) -> Dict[str, Any]:
        try:
            self.logger.info("Testing connection to ScanControl system...")
            # Try to initialize and connect, then disconnect
            if not self.initialize():
                return {'status': 'error', 'message': 'Initialization failed'}
            self.cleanup()
            return {'status': 'success', 'message': 'ScanControl system connection test passed.'}
        except Exception as e:
            self.logger.error(f"ScanControl connection test failed: {e}")
            return {'status': 'error', 'message': str(e)}

    def initialize(self) -> bool:
        try:
            self.logger.info(f"Initializing ScanControl at {self.config.get('ip', 'unknown')}")
            # 1. Create device handle for Ethernet
            self.hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
            # 2. Find available interfaces
            available_interfaces = (ct.c_uint * 6)()
            llt.get_device_interfaces_fast(self.hLLT, available_interfaces, len(available_interfaces))
            # 3. Set first found interface
            llt.set_device_interface(self.hLLT, available_interfaces[0])
            # 4. Connect
            if llt.connect(self.hLLT) != 0:
                self.logger.error("Failed to connect to ScanControl device")
                self.is_initialized = False
                return False
            # 5. Get scanner type
            scanner_type = ct.c_int(0)
            llt.get_llt_type(self.hLLT, ct.byref(scanner_type))
            self.scanner_type = scanner_type.value
            # 6. Get and set resolution (use highest by default)
            available_resolutions = (ct.c_uint * 4)()
            llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
            self.resolution = available_resolutions[0]
            llt.set_resolution(self.hLLT, self.resolution)
            # 7. Set profile config to PROFILE
            llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
            self.is_initialized = True
            self.logger.info("ScanControl system initialized successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize ScanControl: {e}")
            self.is_initialized = False
            return False
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_initialized:
            raise RuntimeError("ScanControl system not initialized")
        step_name = step_config.get('name', 'unknown')
        action = step_config.get('action', 'recording_data')
        parameters = step_config.get('parameters', {})
        recording_time = step_config.get('recording_time', 10)
        saving_file = step_config.get('saving_file', False)
        output_path = step_config.get('path', 'scan_data')
        self.logger.info(f"Executing ScanControl step: {step_name}")
        try:
            if action == "recording_data":
                return self._execute_data_recording(parameters, recording_time, saving_file, output_path)
            else:
                raise ValueError(f"Unsupported scan action: {action}")
        except Exception as e:
            self.logger.error(f"ScanControl step execution failed: {e}")
            return {'success': False, 'error': str(e), 'step_name': step_name}
    
    def _execute_data_recording(self, parameters: Dict[str, Any], recording_time: int, saving_file: bool, output_path: str) -> Dict[str, Any]:
        self.logger.info("Starting data recording scan (real scanner)")
        start_time = time.time()
        # Start profile transfer
        llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        lost_profiles = ct.c_int()
        profiles = []
        for i in range(recording_time):
            # Wait for profile (simulate 1Hz, real scanner may be faster)
            time.sleep(1)
            llt.get_actual_profile(self.hLLT, profile_buffer, len(profile_buffer), llt.TProfileConfig.PROFILE, ct.byref(lost_profiles))
            # Convert profile to x, z, intensities
            x = (ct.c_double * self.resolution)()
            z = (ct.c_double * self.resolution)()
            intensities = (ct.c_ushort * self.resolution)()
            snull = ct.POINTER(ct.c_ushort)()
            inull = ct.POINTER(ct.c_uint)()
            llt.convert_profile_2_values(self.hLLT, profile_buffer, self.resolution, llt.TProfileConfig.PROFILE, self.scanner_type, 0, 1, snull, intensities, snull, x, z, inull, inull)
            # Store profile data
            profiles.append({
                'x': [x[j] for j in range(self.resolution)],
                'z': [z[j] for j in range(self.resolution)],
                'intensities': [intensities[j] for j in range(self.resolution)]
            })
            self.logger.debug(f"Profile {i+1}/{recording_time} captured")
        # Stop profile transfer
        llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        end_time = time.time()
        actual_duration = end_time - start_time
        result = {
            'success': True,
            'step_name': 'laser_scan',
            'mode': 'recording_data',
            'duration': actual_duration,
            'parameters_used': parameters,
            'profiles_captured': len(profiles),
            'file_saved': False,
            'output_path': None
        }
        # Save data if requested
        if saving_file:
            try:
                os.makedirs(output_path, exist_ok=True)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"scan_data_{timestamp}.csv"
                file_path = os.path.join(output_path, filename)
                with open(file_path, 'w') as f:
                    f.write("# x,z,intensity\n")
                    for profile in profiles:
                        for j in range(self.resolution):
                            f.write(f"{profile['x'][j]},{profile['z'][j]},{profile['intensities'][j]}\n")
                result['file_saved'] = True
                result['output_path'] = file_path
                self.logger.info(f"Scan data saved to: {file_path}")
            except Exception as e:
                self.logger.error(f"Failed to save scan data: {e}")
                result['save_error'] = str(e)
        self.logger.info("Data recording scan completed successfully")
        return result
    
    def _execute_calibration(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute calibration scan"""
        self.logger.info("Starting calibration scan")
        
        # Simulate calibration process
        time.sleep(5)
        
        return {
            'success': True,
            'step_name': 'laser_calibration',
            'mode': 'calibration',
            'calibration_status': 'completed',
            'accuracy': 99.5  # Simulate calibration accuracy
        }
    
    def _execute_test_scan(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute test scan"""
        self.logger.info("Starting test scan")
        
        # Simulate test scan
        time.sleep(2)
        
        return {
            'success': True,
            'step_name': 'laser_test',
            'mode': 'test_scan',
            'test_results': 'passed',
            'signal_quality': 'good'
        }
    
    def validate_scan_parameters(self, parameters: Dict[str, Any]) -> bool:
        # For now, always return True (use defaults)
        return True
    
    def cleanup(self) -> None:
        self.logger.info("Cleaning up ScanControl system")
        try:
            if self.hLLT is not None:
                llt.disconnect(self.hLLT)
                llt.del_device(self.hLLT)
                self.hLLT = None
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        self.is_initialized = False
        self.logger.info("ScanControl system cleanup completed")
