"""
Gantry System using CRI controller connection functions from GUI application
"""
import logging
from typing import Dict, Any, Optional
import sys
import os

from systems.base_system import BaseSystem

# Add CRI lib path dynamically based on project structure
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, project_root)

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
            self.connected = False
            return False
            
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
    
    def load_program(self, program_name: str, local_path: str):
        """Load a program to the gantry"""
        logger = logging.getLogger(__name__)
        logger.info(f"Attempting to load program '{program_name}' with local path: {local_path}")
        
        if not self.connected:
            logger.error("Cannot load program: Gantry not connected")
            return False
            
        if not CRI_AVAILABLE:
            logger.info("CRI library not available - simulating program load success")
            self.program_loaded = True
            return True
            
        try:
            logger.debug("Setting active control to True")
            self.controller.set_active_control(True)
            
            logger.debug("Waiting for kinematics to be ready (timeout: 10s)")
            self.controller.wait_for_kinematics_ready(10)
            
            logger.debug("Setting override to 50%")
            self.controller.set_override(50.0)
            
            if local_path:
                logger.info(f"Loading program from local path: {local_path}")
                # Merge the local_path and the program_name (keep Linux paths)
                full_local_path = os.path.join(local_path, program_name)
                logger.debug(f"Full local path resolved to: {full_local_path}")

                # Check if file exists before upload
                if not os.path.exists(full_local_path):
                    logger.error(f"Program file not found at path: {full_local_path}")
                    return False

                # Upload the file first
                logger.info(f"Uploading program file '{full_local_path}' to 'Programs' directory")
                upload_result = self.controller.upload_file(full_local_path, "Programs")
                logger.debug(f"Upload result: {upload_result}")
            else:
                logger.info(f"Loading program '{program_name}' directly (no local path specified)")
            
            # Load the program (use program_name only, not the local path)
            logger.info(f"Loading program '{program_name}' into controller")
            load_result = self.controller.load_programm(program_name)
            logger.debug(f"Program load result: {load_result}")
            
            if load_result:
                self.program_loaded = True
                logger.info(f"Successfully loaded program '{program_name}'")
                return True
            else:
                logger.warning(f"Failed to load program '{program_name}' - controller returned False")
                return False
            
        except FileNotFoundError as e:
            logger.error(f"File not found while loading program '{program_name}': {e}")
            return False
        except PermissionError as e:
            logger.error(f"Permission error while loading program '{program_name}': {e}")
            return False
        except Exception as e:
            logger.error(f"Unexpected error while loading program '{program_name}': {e}")
            return False
    
    def start_program(self, wait_for_completion: bool = True, timeout: float = 300.0):
        """Start the loaded program
        
        Parameters
        ----------
        wait_for_completion : bool
            If True, wait until the program finishes execution
        timeout : float
            Maximum time to wait for program completion in seconds
        """
        logger = logging.getLogger(__name__)

        if not self.connected or not self.program_loaded:
            logger.error("Cannot start program: not connected or no program loaded")
            return False
            
        if not CRI_AVAILABLE:
            logger.info("CRI library not available - simulating program start")
            return True
            
        try:
            logger.info(f"Starting program on gantry (wait_for_completion={wait_for_completion})")
            
            # Register for EXECEND if we want to wait for completion
            if wait_for_completion:
                logger.debug("Registering for EXECEND message to wait for program completion")
                self.controller._register_answer("EXECEND")
            
            # Start the program
            command = "CMD StartProgram"
            msg_id = self.controller._send_command(command, True)
            
            if msg_id is None:
                logger.error("Failed to send start program command")
                return False
            
            # Wait for command acknowledgment
            error_msg = self.controller._wait_for_answer(f"{msg_id}", timeout=10.0)
            if error_msg is not None:
                logger.error(f"Error starting program: {error_msg}")
                return False
            
            logger.info("Program start command acknowledged")
            
            # If we should wait for completion, wait for EXECEND
            if wait_for_completion:
                logger.info(f"Waiting for program completion (timeout: {timeout}s)")
                
                # Wait for EXECEND message
                error_msg = self.controller._wait_for_answer("EXECEND", timeout=timeout)
                if error_msg is not None:
                    logger.warning(f"Program execution error or timeout: {error_msg}")
                    return False
                
                logger.info("Program completed successfully")
            
            return True
        
        except Exception as e:
            logger.error(f"Exception while starting program: {e}")
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
            programs_path = self.config.get('programs_path', 'Not configured')
            
            if self.gantry_controller.test_connection():
                return {
                    'status': 'available',
                    'message': 'Gantry connected and ready',
                    'details': {
                        'motor_enabled': self.gantry_controller.motor_enabled,
                        'program_loaded': self.gantry_controller.program_loaded,
                        'programs_path': programs_path
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
                            'program_loaded': self.gantry_controller.program_loaded,
                            'programs_path': programs_path
                        }
                    }
                else:
                    return {
                        'status': 'not_available',
                        'message': 'Gantry connection failed',
                        'details': {
                            'programs_path': programs_path
                        }
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
                result = self._perform_move(step_config)
            elif action == 'load_program':
                result = self._load_program(step_config)
            elif action == 'run_program':
                result = self._run_program(step_config)
            elif action == 'load_and_run':
                result = self._load_and_run_program(step_config)
            else:
                raise ValueError(f"Unknown gantry action: {action}")
            
            # Always disconnect after completing the step to free resources
            self.logger.info(f"Step '{step_name}' completed successfully, disconnecting gantry")
            self.disconnect()
            
            return result
                
        except Exception as e:
            self.logger.error(f"Gantry step execution failed: {e}")
            # Ensure we disconnect even if there's an error
            try:
                self.disconnect()
                self.logger.info("Disconnected gantry after error")
            except Exception as disconnect_error:
                self.logger.warning(f"Failed to disconnect gantry after error: {disconnect_error}")
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
        
        if not program_name:
            raise ValueError("Program name is required")
        
        # Determine the program path to use
        # Priority: step-specific program_path > default programs_path from config
        step_program_path = step_config.get('program_path')
        default_programs_path = self.config.get('programs_path')
        
        if step_program_path:
            # Use the path specified in the step
            programs_path = step_program_path
            self.logger.info(f"Using step-specific program path: {programs_path}")
        elif default_programs_path:
            # Use the default path from configuration
            programs_path = default_programs_path
            self.logger.info(f"Using default program path from config: {programs_path}")
        else:
            # No path specified, load program without path
            programs_path = None
            self.logger.info("No program path specified, loading program without path")
        
        self.logger.info(f"Loading gantry program: {program_name} from path: {programs_path}")
        
        success = self.gantry_controller.load_program(program_name, programs_path)
        if not success:
            raise RuntimeError(f"Failed to load program: {program_name}")
        
        return {
            'status': 'success',
            'message': f'Program {program_name} loaded successfully from {programs_path or "default location"}',
            'data': {
                'program_name': program_name,
                'program_path': programs_path,
                'timestamp': self.get_timestamp()
            }
        }
    
    def _run_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run a program"""
        self.logger.info("Running gantry program")
        
        # Ensure motors are enabled before starting program
        if not self.gantry_controller.motor_enabled:
            if not self.gantry_controller.enable_motors():
                raise RuntimeError("Failed to enable gantry motors before starting program")
        
        # Get configuration for program execution
        wait_for_completion = step_config.get('wait_for_completion', True)
        timeout = step_config.get('timeout', 300.0)  # Default 5 minutes
        
        self.logger.info(f"Starting program with wait_for_completion={wait_for_completion}, timeout={timeout}s")
        
        success = self.gantry_controller.start_program(
            wait_for_completion=wait_for_completion,
            timeout=timeout
        )
        
        if not success:
            raise RuntimeError("Failed to start program or program execution failed")
        
        completion_message = "Program completed successfully" if wait_for_completion else "Program started successfully"
        
        return {
            'status': 'success',
            'message': completion_message,
            'data': {
                'wait_for_completion': wait_for_completion,
                'timeout': timeout,
                'timestamp': self.get_timestamp()
            }
        }
    
    def _load_and_run_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Load and run a program in one atomic operation"""
        program_name = step_config.get('program_name')
        
        if not program_name:
            raise ValueError("Program name is required for load_and_run action")
        
        # Determine the program path to use
        step_program_path = step_config.get('program_path')
        default_programs_path = self.config.get('programs_path')
        
        if step_program_path:
            programs_path = step_program_path
            self.logger.info(f"Using step-specific program path: {programs_path}")
        elif default_programs_path:
            programs_path = default_programs_path
            self.logger.info(f"Using default program path from config: {programs_path}")
        else:
            programs_path = None
            self.logger.info("No program path specified, loading program without path")
        
        # Get execution configuration
        wait_for_completion = step_config.get('wait_for_completion', True)
        timeout = step_config.get('timeout', 300.0)  # Default 5 minutes
        
        self.logger.info(f"Loading and running gantry program: {program_name} from path: {programs_path}")
        self.logger.info(f"Execution settings: wait_for_completion={wait_for_completion}, timeout={timeout}s")
        
        try:
            # Step 1: Load the program
            self.logger.info(f"Step 1: Loading program {program_name}")
            load_success = self.gantry_controller.load_program(program_name, programs_path)
            if not load_success:
                raise RuntimeError(f"Failed to load program: {program_name}")
            
            self.logger.info(f"Program {program_name} loaded successfully")
            
            # Step 2: Ensure motors are enabled
            if not self.gantry_controller.motor_enabled:
                self.logger.info("Step 2: Enabling gantry motors")
                if not self.gantry_controller.enable_motors():
                    raise RuntimeError("Failed to enable gantry motors before starting program")
            else:
                self.logger.info("Step 2: Motors already enabled")
            
            # Step 3: Start the program
            self.logger.info(f"Step 3: Starting program {program_name}")
            run_success = self.gantry_controller.start_program(
                wait_for_completion=wait_for_completion,
                timeout=timeout
            )
            
            if not run_success:
                raise RuntimeError("Failed to start program or program execution failed")
            
            completion_message = f"Program {program_name} loaded and completed successfully" if wait_for_completion else f"Program {program_name} loaded and started successfully"
            
            return {
                'status': 'success',
                'message': completion_message,
                'data': {
                    'program_name': program_name,
                    'program_path': programs_path,
                    'wait_for_completion': wait_for_completion,
                    'timeout': timeout,
                    'action': 'load_and_run',
                    'timestamp': self.get_timestamp()
                }
            }
            
        except Exception as e:
            error_msg = f"Failed to load and run program {program_name}: {str(e)}"
            self.logger.error(error_msg)
            raise RuntimeError(error_msg)
    
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
