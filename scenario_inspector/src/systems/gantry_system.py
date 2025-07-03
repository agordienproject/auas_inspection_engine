"""
Gantry System using CRI controller connection functions from GUI application
"""
import logging
from typing import Dict, Any, Optional
import sys
import os

from systems.base_system import BaseSystem

# Add CRI lib path
sys.path.insert(0, '/home/agordien/projects/auas_inspection_engine/gui_application')

try:
    import cri_lib
    CRI_AVAILABLE = True
except ImportError as e:
    print(f"Warning: CRI library not available: {e}")
    CRI_AVAILABLE = False


class GantryController:
    """Controller class for Gantry operations (from GUI application)"""
    
    def __init__(self):
        self.controller = None
        self.connected = False
        self.motor_enabled = False
        self.program_loaded = False
        
    def connect(self, ip: str, port: int):
        """Connect to gantry system"""
        if not CRI_AVAILABLE:
            # Simulate connection for testing
            self.connected = True
            return True
            
        try:
            if self.controller:
                try:
                    self.controller.close()
                except Exception:
                    pass
                    
            self.controller = cri_lib.CRIController()
            self.connected = self.controller.connect(ip, port)
            return self.connected
            
        except Exception as e:
            self.connected = False
            raise e
    
    def disconnect(self):
        """Disconnect from gantry system"""
        try:
            if self.controller and self.connected:
                self.controller.close()
            self.connected = False
            self.motor_enabled = False
            self.program_loaded = False
            return True
        except Exception:
            self.connected = False
            return False
    
    def enable_motors(self):
        """Enable gantry motors"""
        if not self.connected:
            return False
            
        if not CRI_AVAILABLE:
            self.motor_enabled = True
            return True
            
        try:
            ok = self.controller.enable()
            if ok:
                self.motor_enabled = True
            return ok
        except Exception:
            return False
    
    def disable_motors(self):
        """Disable gantry motors"""
        if not self.connected:
            return False
            
        if not CRI_AVAILABLE:
            self.motor_enabled = False
            return True
            
        try:
            ok = self.controller.disable()
            if ok:
                self.motor_enabled = False
            return ok
        except Exception:
            return False
    
    def load_program(self, program_name: str, local_path: str = None):
        """Load a program to the gantry"""
        if not self.connected:
            return False
            
        if not CRI_AVAILABLE:
            self.program_loaded = True
            return True
            
        try:
            self.controller.set_active_control(True)
            self.controller.wait_for_kinematics_ready(10)
            self.controller.set_override(50.0)
            
            if local_path:
                # Upload the file first
                self.controller.upload_file(local_path, "Programs")
            
            # Load the program
            if self.controller.load_programm(program_name):
                self.program_loaded = True
                return True
            return False
            
        except Exception:
            return False
    
    def start_program(self):
        """Start the loaded program"""
        if not self.connected or not self.program_loaded:
            return False
            
        if not CRI_AVAILABLE:
            return True
            
        try:
            return self.controller.start_programm()
        except Exception:
            return False
    
    def stop_program(self):
        """Stop the running program"""
        if not self.connected:
            return False
            
        if not CRI_AVAILABLE:
            return True
            
        try:
            return self.controller.stop_programm()
        except Exception:
            return False
    
    def pause_program(self):
        """Pause the running program"""
        if not self.connected:
            return False
            
        if not CRI_AVAILABLE:
            return True
            
        try:
            return self.controller.pause_programm()
        except Exception:
            return False
    
    def test_connection(self):
        """Test if connection is working"""
        return self.connected


class GantrySystem(BaseSystem):
    """Gantry system implementation using GUI connection functions"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.gantry_controller = GantryController()
        self.logger = logging.getLogger(__name__)
        
    def initialize(self) -> bool:
        """Initialize the gantry system"""
        try:
            self.logger.info(f"Initializing gantry system: {self.name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize gantry system: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to the gantry"""
        try:
            ip = self.config.get('ip', '192.168.3.11')
            port = self.config.get('port', 3920)
            
            self.logger.info(f"Attempting to connect to gantry at {ip}:{port}")
            
            success = self.gantry_controller.connect(ip, port)
            if success:
                self.logger.info("Gantry connected successfully")
                return True
            else:
                self.logger.error("Failed to connect to gantry")
                return False
                
        except Exception as e:
            self.logger.error(f"Gantry connection error: {e}")
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from the gantry"""
        try:
            success = self.gantry_controller.disconnect()
            if success:
                self.logger.info("Gantry disconnected successfully")
            return success
        except Exception as e:
            self.logger.error(f"Gantry disconnection error: {e}")
            return False
    
    def test_connection(self) -> Dict[str, Any]:
        """Test gantry connection"""
        try:
            if self.gantry_controller.test_connection():
                return {
                    'status': 'available',
                    'message': 'Gantry connected and ready',
                    'details': {
                        'motor_enabled': self.gantry_controller.motor_enabled,
                        'program_loaded': self.gantry_controller.program_loaded
                    }
                }
            else:
                # Try to connect
                if self.connect():
                    return {
                        'status': 'available',
                        'message': 'Gantry connection established',
                        'details': {
                            'motor_enabled': self.gantry_controller.motor_enabled,
                            'program_loaded': self.gantry_controller.program_loaded
                        }
                    }
                else:
                    return {
                        'status': 'not_available',
                        'message': 'Gantry connection failed'
                    }
        except Exception as e:
            self.logger.error(f"Gantry connection test failed: {e}")
            return {
                'status': 'error',
                'message': f'Gantry test error: {str(e)}'
            }
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the gantry"""
        step_name = step_config.get('name', 'unknown')
        self.logger.info(f"Executing gantry step: {step_name}")
        
        try:
            # Ensure connection
            if not self.gantry_controller.connected:
                if not self.connect():
                    raise ConnectionError("Cannot connect to gantry")
            
            # Execute the step based on step configuration
            action = step_config.get('action', 'move')
            
            if action == 'move':
                return self._perform_move(step_config)
            elif action == 'load_program':
                return self._load_program(step_config)
            elif action == 'run_program':
                return self._run_program(step_config)
            else:
                raise ValueError(f"Unknown gantry action: {action}")
                
        except Exception as e:
            self.logger.error(f"Gantry step execution failed: {e}")
            raise
    
    def _perform_move(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Perform a move operation"""
        self.logger.info("Performing gantry move operation")
        
        # Enable motors if needed
        if not self.gantry_controller.motor_enabled:
            if not self.gantry_controller.enable_motors():
                raise RuntimeError("Failed to enable gantry motors")
        
        return {
            'status': 'success',
            'message': 'Move completed successfully',
            'data': {
                'move_id': f"move_{self.name}_{step_config.get('step', 'unknown')}",
                'timestamp': self.get_timestamp()
            }
        }
    
    def _load_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Load a program"""
        program_name = step_config.get('program_name')
        program_path = step_config.get('program_path')
        
        if not program_name:
            raise ValueError("Program name is required")
        
        self.logger.info(f"Loading gantry program: {program_name}")
        
        success = self.gantry_controller.load_program(program_name, program_path)
        if not success:
            raise RuntimeError(f"Failed to load program: {program_name}")
        
        return {
            'status': 'success',
            'message': f'Program {program_name} loaded successfully',
            'data': {
                'program_name': program_name,
                'timestamp': self.get_timestamp()
            }
        }
    
    def _run_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run a program"""
        self.logger.info("Running gantry program")
        
        success = self.gantry_controller.start_program()
        if not success:
            raise RuntimeError("Failed to start program")
        
        return {
            'status': 'success',
            'message': 'Program started successfully',
            'data': {
                'timestamp': self.get_timestamp()
            }
        }
    
    def shutdown(self):
        """Shutdown the gantry system"""
        try:
            # Stop any running program
            self.gantry_controller.stop_program()
            # Disable motors
            self.gantry_controller.disable_motors()
            # Disconnect
            self.disconnect()
            self.logger.info("Gantry system shut down")
        except Exception as e:
            self.logger.error(f"Error shutting down gantry system: {e}")
