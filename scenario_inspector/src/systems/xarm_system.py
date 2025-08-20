"""
xArm Robot System using Python SDK (No ROS2 Required)

This system provides xArm robot control functionality for the scenario inspector,
including move to position, load program, start program, and stop program operations.
"""
import logging
import time
import os
import subprocess
import sys
from typing import Dict, Any, Optional
from systems.base_system import BaseSystem

# Try to import xArm Python SDK
try:
    from xarm.wrapper import XArmAPI
    XARM_SDK_AVAILABLE = True
except ImportError as e:
    XARM_SDK_AVAILABLE = False
    print(f"[WARN] xArm Python SDK not found: {e}")
    print("[WARN] Install with: pip install xarm-python-sdk")
    
    # Create dummy class for development/testing
    class XArmAPI:
        def __init__(self, ip):
            self.ip = ip
            
        def motion_enable(self, enable): 
            return 0
            
        def set_mode(self, mode): 
            return 0
            
        def set_state(self, state): 
            return 0
            
        def get_version(self): 
            return (0, "Mock xArm v1.0.0")
            
        def get_state(self): 
            return (0, 2)
            
        def set_position(self, *args, **kwargs): 
            return 0
            
        def move_gohome(self, wait=True): 
            return 0
            
        def load_program(self, filename): 
            return 0
            
        def run_program(self): 
            return 0
            
        def stop_program(self): 
            return 0
            
        def emergency_stop(self): 
            return 0
            
        def disconnect(self): 
            return 0


class XarmSystem(BaseSystem):
    """xArm Robot System for scenario inspector"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.arm = None
        self.connected = False
        self.enabled = False
        self.ip = config.get('ip', '192.168.1.222')
        self.timeout = config.get('timeout', 30)
        self.programs_path = config.get('programs_path', os.path.join(os.path.dirname(__file__), 'xarm_programs'))
        self.current_program = None
        self.current_program_name = None
        
    def initialize(self) -> bool:
        """Initialize the xArm system"""
        try:
            self.logger.info(f"Initializing xArm system at {self.ip}")
            
            if not XARM_SDK_AVAILABLE:
                self.logger.warning("xArm SDK not available, using mock mode")
                self.is_initialized = True
                return True
            
            # Test connection
            result = self.test_connection()
            if result.get('status') == 'available':
                self.is_initialized = True
                self.logger.info("xArm system initialized successfully")
                return True
            else:
                self.logger.error(f"Failed to initialize xArm: {result.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error initializing xArm system: {e}")
            return False
    
    def test_connection(self) -> Dict[str, Any]:
        """Test connection to the xArm robot"""
        try:
            self.logger.info(f"Testing connection to xArm at {self.ip}")
            
            # If SDK is not available, return not_available status
            if not XARM_SDK_AVAILABLE:
                self.logger.warning("xArm SDK not available, using mock mode")
                return {
                    "status": "not_available",
                    "message": "xArm SDK not available - install with: pip install xarm-python-sdk",
                    "details": {
                        "ip": self.ip,
                        "sdk_available": False,
                        "mock_mode": True
                    }
                }
            
            # Create xArm API instance
            test_arm = XArmAPI(self.ip)
            
            # Initialize robot (following working example pattern)
            result1 = test_arm.motion_enable(enable=True)
            result2 = test_arm.set_mode(0)
            result3 = test_arm.set_state(state=0)
            
            # Test by getting version info
            version = test_arm.get_version()
            state = test_arm.get_state()
            
            # Clean up test connection
            try:
                test_arm.disconnect()
            except:
                pass  # Some versions may not have disconnect
            
            if result1 == 0 and result2 == 0 and result3 == 0:
                return {
                    "status": "available",
                    "message": f"Successfully connected to xArm at {self.ip}",
                    "details": {
                        "ip": self.ip,
                        "version": version,
                        "state": state,
                        "sdk_available": XARM_SDK_AVAILABLE
                    }
                }
            else:
                return {
                    "status": "not_available",
                    "message": f"Connection failed with results: {result1}, {result2}, {result3}",
                    "details": {
                        "ip": self.ip,
                        "sdk_available": XARM_SDK_AVAILABLE
                    }
                }
                
        except Exception as e:
            return {
                "status": "error",
                "message": f"Connection test failed: {str(e)}",
                "details": {
                    "ip": self.ip,
                    "sdk_available": XARM_SDK_AVAILABLE
                }
            }
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step with given configuration"""
        action = step_config.get('action', '').lower()
        
        self.logger.info(f"Executing xArm step: {action}")
        
        try:
            if action == 'move_to_position':
                return self._move_to_position(step_config)
            elif action == 'load_program':
                return self._load_program(step_config)
            elif action == 'start_program':
                return self._start_program(step_config)
            elif action == 'stop_program':
                return self._stop_program(step_config)
            elif action == 'connect':
                return self._connect_robot()
            elif action == 'disconnect':
                return self._disconnect_robot()
            elif action == 'enable':
                return self._enable_robot()
            elif action == 'disable':
                return self._disable_robot()
            elif action == 'home':
                return self._move_home()
            elif action == 'emergency_stop':
                return self._emergency_stop()
            else:
                return {
                    "success": False,
                    "error": f"Unknown step type: {action}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Error executing step {action}: {e}")
            return {
                "success": False,
                "error": f"Execution failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _connect_robot(self) -> Dict[str, Any]:
        """Connect to the xArm robot"""
        try:
            if self.connected:
                return {
                    "success": True,
                    "message": "Robot already connected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info(f"Connecting to xArm at {self.ip}")
            
            # Create xArm API instance
            self.arm = XArmAPI(self.ip)
            
            # Initialize robot (following working example pattern)
            result1 = self.arm.motion_enable(enable=True)
            result2 = self.arm.set_mode(0)
            result3 = self.arm.set_state(state=0)
            
            if result1 == 0 and result2 == 0 and result3 == 0:
                self.connected = True
                self.enabled = True
                
                # Get robot info
                version = self.arm.get_version()
                state = self.arm.get_state()
                
                self.logger.info(f"Successfully connected to xArm. Version: {version}, State: {state}")
                
                return {
                    "success": True,
                    "message": f"Connected to xArm at {self.ip}",
                    "details": {
                        "version": version,
                        "state": state
                    },
                    "timestamp": self.get_timestamp()
                }
            else:
                return {
                    "success": False,
                    "error": f"Connection failed with results: {result1}, {result2}, {result3}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Connection error: {e}")
            return {
                "success": False,
                "error": f"Connection failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _disconnect_robot(self) -> Dict[str, Any]:
        """Disconnect from the xArm robot"""
        try:
            if not self.connected:
                return {
                    "success": True,
                    "message": "Robot already disconnected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info("Disconnecting from xArm")
            
            if self.arm:
                try:
                    self.arm.motion_enable(enable=False)
                    self.arm.disconnect()
                except:
                    pass  # Some versions may not have these methods
            
            self.arm = None
            self.connected = False
            self.enabled = False
            self.current_program = None
            self.current_program_name = None
            
            return {
                "success": True,
                "message": "Disconnected from xArm",
                "timestamp": self.get_timestamp()
            }
            
        except Exception as e:
            self.logger.error(f"Disconnect error: {e}")
            return {
                "success": False,
                "error": f"Disconnect failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _enable_robot(self) -> Dict[str, Any]:
        """Enable robot motion"""
        try:
            if not self.connected:
                return {
                    "success": False,
                    "error": "Robot not connected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info("Enabling xArm motion")
            
            result1 = self.arm.motion_enable(enable=True)
            result2 = self.arm.set_mode(0)  # Position mode
            result3 = self.arm.set_state(state=0)  # Ready state
            
            if result1 == 0 and result2 == 0 and result3 == 0:
                self.enabled = True
                return {
                    "success": True,
                    "message": "Robot enabled successfully",
                    "timestamp": self.get_timestamp()
                }
            else:
                return {
                    "success": False,
                    "error": f"Enable failed with results: {result1}, {result2}, {result3}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Enable error: {e}")
            return {
                "success": False,
                "error": f"Enable failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _disable_robot(self) -> Dict[str, Any]:
        """Disable robot motion"""
        try:
            if not self.connected:
                return {
                    "success": False,
                    "error": "Robot not connected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info("Disabling xArm motion")
            
            result = self.arm.motion_enable(enable=False)
            
            if result == 0:
                self.enabled = False
                return {
                    "success": True,
                    "message": "Robot disabled successfully",
                    "timestamp": self.get_timestamp()
                }
            else:
                return {
                    "success": False,
                    "error": f"Disable failed with result: {result}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Disable error: {e}")
            return {
                "success": False,
                "error": f"Disable failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _move_to_position(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Move robot to specified position"""
        try:
            # Ensure we're connected (auto-connect if needed)
            if not self.connected:
                connect_result = self._connect_robot()
                if not connect_result.get('success', False):
                    return connect_result
            
            # Extract position parameters
            x = step_config.get('x', 0)
            y = step_config.get('y', 0)
            z = step_config.get('z', 0)
            roll = step_config.get('roll', 0)
            pitch = step_config.get('pitch', 0)
            yaw = step_config.get('yaw', 0)
            speed = step_config.get('speed', 50)
            wait = step_config.get('wait', True)
            
            self.logger.info(f"Moving xArm to position: ({x}, {y}, {z}, {roll}, {pitch}, {yaw})")
            
            result = self.arm.set_position(x, y, z, roll, pitch, yaw, speed=speed, wait=wait)
            
            if result == 0:
                return {
                    "success": True,
                    "message": f"Moved to position ({x}, {y}, {z})",
                    "details": {
                        "position": [x, y, z, roll, pitch, yaw],
                        "speed": speed
                    },
                    "timestamp": self.get_timestamp()
                }
            else:
                return {
                    "success": False,
                    "error": f"Move failed with error code: {result}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Move error: {e}")
            return {
                "success": False,
                "error": f"Move failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _move_home(self) -> Dict[str, Any]:
        """Move robot to home position"""
        try:
            # Ensure we're connected (auto-connect if needed)
            if not self.connected:
                connect_result = self._connect_robot()
                if not connect_result.get('success', False):
                    return connect_result
            
            self.logger.info("Moving xArm to home position")
            
            result = self.arm.move_gohome(wait=True)
            
            if result == 0:
                return {
                    "success": True,
                    "message": "Moved to home position",
                    "timestamp": self.get_timestamp()
                }
            else:
                return {
                    "success": False,
                    "error": f"Move home failed with error code: {result}",
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Move home error: {e}")
            return {
                "success": False,
                "error": f"Move home failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _load_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Load a Python program for xArm robot"""
        try:
            program_name = step_config.get('program_name', '')
            if not program_name:
                return {
                    "success": False,
                    "error": "No program name specified",
                    "timestamp": self.get_timestamp()
                }
            
            # Construct full program path
            if not os.path.isabs(program_name):
                program_path = os.path.join(self.programs_path, program_name)
            else:
                program_path = program_name
            
            # Check if program file exists
            if not os.path.exists(program_path):
                return {
                    "success": False,
                    "error": f"Program file not found: {program_path}",
                    "timestamp": self.get_timestamp()
                }
            
            # Check if it's a Python file
            if not program_path.endswith('.py'):
                return {
                    "success": False,
                    "error": f"Program must be a Python file (.py): {program_path}",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info(f"Loading xArm Python program: {program_path}")
            
            # Store the program path for execution
            self.current_program = program_path
            self.current_program_name = program_name
            
            return {
                "success": True,
                "message": f"Program loaded: {program_name}",
                "details": {
                    "program_name": program_name,
                    "program_path": program_path
                },
                "timestamp": self.get_timestamp()
            }
                
        except Exception as e:
            self.logger.error(f"Load program error: {e}")
            return {
                "success": False,
                "error": f"Load program failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _start_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Start/execute a Python program for xArm robot"""
        try:
            # First ensure we're connected
            if not self.connected:
                connect_result = self._connect_robot()
                if not connect_result.get('success', False):
                    return connect_result
            
            # Get program name and handle loading if not already loaded
            program_name = step_config.get('program_name', '')
            if program_name:
                # Load the program first
                load_result = self._load_program(step_config)
                if not load_result.get('success', False):
                    return load_result
            elif not self.current_program:
                return {
                    "success": False,
                    "error": "No program loaded or specified",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info(f"Starting xArm Python program: {self.current_program_name or self.current_program}")
            
            # Execute the Python program with the robot IP as argument
            try:
                result = subprocess.run([
                    sys.executable, self.current_program, self.ip
                ], 
                capture_output=True, 
                text=True, 
                timeout=self.timeout,
                cwd=os.path.dirname(self.current_program)
                )
                
                if result.returncode == 0:
                    self.logger.info(f"Program executed successfully: {self.current_program}")
                    return {
                        "success": True,
                        "message": f"Program executed successfully: {os.path.basename(self.current_program)}",
                        "details": {
                            "program_path": self.current_program,
                            "stdout": result.stdout if result.stdout else "No output",
                            "execution_time": "Completed"
                        },
                        "timestamp": self.get_timestamp()
                    }
                else:
                    error_msg = result.stderr if result.stderr else f"Program exited with code {result.returncode}"
                    self.logger.error(f"Program execution failed: {error_msg}")
                    return {
                        "success": False,
                        "error": f"Program execution failed: {error_msg}",
                        "details": {
                            "program_path": self.current_program,
                            "stdout": result.stdout,
                            "stderr": result.stderr,
                            "return_code": result.returncode
                        },
                        "timestamp": self.get_timestamp()
                    }
                    
            except subprocess.TimeoutExpired:
                return {
                    "success": False,
                    "error": f"Program execution timed out after {self.timeout} seconds",
                    "details": {
                        "program_path": self.current_program,
                        "timeout": self.timeout
                    },
                    "timestamp": self.get_timestamp()
                }
            except subprocess.SubprocessError as e:
                return {
                    "success": False,
                    "error": f"Failed to execute program: {str(e)}",
                    "details": {
                        "program_path": self.current_program
                    },
                    "timestamp": self.get_timestamp()
                }
                
        except Exception as e:
            self.logger.error(f"Start program error: {e}")
            return {
                "success": False,
                "error": f"Start program failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _stop_program(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Stop the currently running program"""
        try:
            if not self.connected:
                return {
                    "success": False,
                    "error": "Robot not connected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.info("Stopping xArm program")
            
            # Since programs are executed as subprocess, we'll use emergency stop
            # to halt any ongoing robot movement
            if self.arm and hasattr(self.arm, 'emergency_stop'):
                self.arm.emergency_stop()
                # Re-enable after emergency stop if needed
                time.sleep(1)
                if self.enabled:
                    self.arm.motion_enable(enable=True)
                    self.arm.set_mode(0)
                    self.arm.set_state(state=0)
            
            return {
                "success": True,
                "message": "Program stop signal sent (emergency stop used)",
                "details": {
                    "note": "Python programs run as subprocess - emergency stop used to halt robot movement"
                },
                "timestamp": self.get_timestamp()
            }
                
        except Exception as e:
            self.logger.error(f"Stop program error: {e}")
            return {
                "success": False,
                "error": f"Stop program failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def _emergency_stop(self) -> Dict[str, Any]:
        """Emergency stop the robot"""
        try:
            if not self.connected:
                return {
                    "success": False,
                    "error": "Robot not connected",
                    "timestamp": self.get_timestamp()
                }
            
            self.logger.warning("Emergency stop activated")
            
            self.arm.emergency_stop()
            self.enabled = False
            
            return {
                "success": True,
                "message": "Emergency stop activated",
                "timestamp": self.get_timestamp()
            }
            
        except Exception as e:
            self.logger.error(f"Emergency stop error: {e}")
            return {
                "success": False,
                "error": f"Emergency stop failed: {str(e)}",
                "timestamp": self.get_timestamp()
            }
    
    def cleanup(self) -> None:
        """Cleanup xArm system resources"""
        try:
            if self.connected and self.arm:
                self.logger.info("Cleaning up xArm system")
                self.arm.motion_enable(enable=False)
                self.arm.disconnect()
                
            self.arm = None
            self.connected = False
            self.enabled = False
            self.current_program = None
            self.current_program_name = None
            
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current xArm system status"""
        base_status = super().get_status()
        base_status.update({
            "connected": self.connected,
            "enabled": self.enabled,
            "ip": self.ip,
            "current_program": self.current_program_name,
            "current_program_path": self.current_program,
            "sdk_available": XARM_SDK_AVAILABLE
        })
        return base_status
    
    def validate_parameters(self, parameters: Dict[str, Any]) -> bool:
        """Validate step parameters"""
        action = parameters.get('type', '').lower()
        
        if action == 'move_to_position':
            # Check if position parameters are provided and valid
            required_params = ['x', 'y', 'z']
            for param in required_params:
                if param not in parameters:
                    self.logger.error(f"Missing required parameter: {param}")
                    return False
                try:
                    float(parameters[param])
                except (ValueError, TypeError):
                    self.logger.error(f"Invalid value for parameter {param}: {parameters[param]}")
                    return False
            return True
            
        elif action == 'load_program':
            if 'program_name' not in parameters:
                self.logger.error("Missing required parameter: program_name")
                return False
            return True
            
        elif action in ['start_program', 'stop_program', 'connect', 'disconnect', 
                          'enable', 'disable', 'home', 'emergency_stop']:
            return True
            
        else:
            self.logger.error(f"Unknown step type: {action}")
            return False
