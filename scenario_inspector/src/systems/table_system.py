"""
Table System using serial connection functions from GUI application
"""
import logging
import serial
import serial.tools.list_ports
import time
from typing import Dict, Any, Optional

from systems.base_system import BaseSystem


class TableController:
    """Controller class for rotating table operations (from GUI application)"""
    
    def __init__(self):
        self.serial_connection = None
        self.connected = False
        self.port = None
        self.baudrate = None
        
    def connect(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        """Connect to table via serial"""
        try:
            # Check if the port exists first
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            if port not in available_ports:
                raise ConnectionError(f"Port {port} not found. Available ports: {available_ports}")
            
            self.port = port
            self.baudrate = baudrate
            
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout
            )
            
            if self.serial_connection.is_open:
                self.connected = True
                # Wait a bit for Arduino to initialize (like the GUI does)
                time.sleep(2)
                return True
            else:
                return False
                
        except Exception as e:
            self.connected = False
            raise e
    
    def disconnect(self):
        """Disconnect from table"""
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
            self.connected = False
            return True
        except Exception:
            self.connected = False
            return False
    
    def send_command(self, command: str):
        """Send command to table"""
        if not self.connected or not self.serial_connection:
            raise ConnectionError("Not connected to table")
        
        try:
            self.serial_connection.write((command + '\n').encode())
            # Add a small delay to ensure command is processed
            time.sleep(0.1)
            return True
        except Exception as e:
            raise RuntimeError(f"Failed to send command: {e}")
    
    def start_rotation(self):
        """Start table rotation"""
        return self.send_command("rotate_table")
    
    def stop_rotation(self):
        """Stop table rotation"""
        return self.send_command("stop_table")
    
    def move_by_degree(self, degrees: int):
        """Move table by specific degrees"""
        return self.send_command(f"move_table_{degrees}")
    
    def test_connection(self):
        """Test if connection is working"""
        if not self.connected:
            return False
        
        try:
            # Try sending a simple status command
            self.send_command("status")
            return True
        except Exception:
            return False


class TableSystem(BaseSystem):
    """Table system implementation using GUI connection functions"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.table_controller = TableController()
        self.logger = logging.getLogger(__name__)
        
    def initialize(self) -> bool:
        """Initialize the table system"""
        try:
            self.logger.info(f"Initializing table system: {self.name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize table system: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to the table"""
        try:
            port = self.config.get('port', '/dev/ttyACM0')
            baudrate = self.config.get('baudrate', 9600)
            timeout = self.config.get('timeout', 1.0)  # Use shorter timeout like GUI
            
            self.logger.info(f"Attempting to connect to table at {port} (baudrate: {baudrate})")
            
            # Check if Arduino is connected (like GUI does)
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            if port not in available_ports:
                self.logger.error(f"Arduino not detected at {port}. Available ports: {available_ports}")
                return False
            
            success = self.table_controller.connect(port, baudrate, timeout)
            if success:
                self.logger.info("Table connected successfully")
                return True
            else:
                self.logger.error("Failed to connect to table")
                return False
                
        except Exception as e:
            self.logger.error(f"Table connection error: {e}")
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from the table"""
        try:
            success = self.table_controller.disconnect()
            if success:
                self.logger.info("Table disconnected successfully")
            return success
        except Exception as e:
            self.logger.error(f"Table disconnection error: {e}")
            return False
    
    def test_connection(self) -> Dict[str, Any]:
        """Test table connection"""
        try:
            port = self.config.get('port', '/dev/ttyACM0')
            
            # First check if Arduino is detected (like GUI does)
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            if port not in available_ports:
                return {
                    'status': 'not_available',
                    'message': f'Arduino not detected at {port}',
                    'details': {
                        'available_ports': available_ports,
                        'expected_port': port
                    }
                }
            
            # Try to connect if not already connected
            if not self.table_controller.connected:
                if not self.connect():
                    return {
                        'status': 'not_available',
                        'message': 'Table connection failed - check serial port'
                    }
            
            # Test if the connection is working
            if self.table_controller.test_connection():
                return {
                    'status': 'available',
                    'message': 'Table connected and ready',
                    'details': {
                        'port': self.table_controller.port,
                        'baudrate': self.table_controller.baudrate
                    }
                }
            else:
                return {
                    'status': 'not_available',
                    'message': 'Table connection test failed'
                }
                
        except Exception as e:
            self.logger.error(f"Table connection test failed: {e}")
            return {
                'status': 'error',
                'message': f'Table test error: {str(e)}'
            }
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the table"""
        step_name = step_config.get('name', 'unknown')
        self.logger.info(f"Executing table step: {step_name}")
        
        try:
            # Ensure connection
            if not self.table_controller.connected:
                if not self.connect():
                    raise ConnectionError("Cannot connect to table")
            
            # Execute the step based on step configuration
            action = step_config.get('action', 'rotate')
            
            if action == 'rotate':
                return self._start_rotation(step_config)
            elif action == 'stop':
                return self._stop_rotation(step_config)
            elif action == 'move':
                return self._move_by_degree(step_config)
            else:
                raise ValueError(f"Unknown table action: {action}")
                
        except Exception as e:
            self.logger.error(f"Table step execution failed: {e}")
            raise
    
    def _start_rotation(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Start table rotation"""
        duration = step_config.get('duration', 0)
        
        if duration > 0:
            self.logger.info(f"Starting table rotation for {duration} seconds")
        else:
            self.logger.info("Starting table rotation (continuous)")
        
        self.logger.info("Sending rotate_table command...")
        success = self.table_controller.start_rotation()
        if not success:
            raise RuntimeError("Failed to start table rotation")
        
        self.logger.info("Rotate command sent successfully")
        
        # If duration is specified, wait for that time then stop
        if duration > 0:
            self.logger.info(f"Waiting {duration} seconds...")
            time.sleep(duration)
            
            self.logger.info("Duration completed, stopping rotation")
            stop_success = self.table_controller.stop_rotation()
            if not stop_success:
                self.logger.warning("Failed to stop rotation after duration")
            else:
                self.logger.info("Table rotation stopped successfully")
        
        return {
            'status': 'success',
            'message': f'Table rotation {"completed" if duration > 0 else "started"}',
            'data': {
                'action': 'rotate',
                'duration': duration,
                'timestamp': self.get_timestamp()
            }
        }
    
    def _stop_rotation(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Stop table rotation"""
        self.logger.info("Stopping table rotation")
        
        success = self.table_controller.stop_rotation()
        if not success:
            raise RuntimeError("Failed to stop table rotation")
        
        return {
            'status': 'success',
            'message': 'Table rotation stopped',
            'data': {
                'action': 'stop',
                'timestamp': self.get_timestamp()
            }
        }
    
    def _move_by_degree(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Move table by specific degrees"""
        degrees = step_config.get('degrees', 90)
        
        self.logger.info(f"Moving table by {degrees} degrees")
        
        success = self.table_controller.move_by_degree(degrees)
        if not success:
            raise RuntimeError(f"Failed to move table by {degrees} degrees")
        
        return {
            'status': 'success',
            'message': f'Table moved by {degrees} degrees',
            'data': {
                'action': 'move',
                'degrees': degrees,
                'timestamp': self.get_timestamp()
            }
        }
    
    def shutdown(self):
        """Shutdown the table system"""
        try:
            # Stop rotation if running
            try:
                self.table_controller.stop_rotation()
            except Exception:
                pass  # Ignore errors if not rotating
            
            # Disconnect
            self.disconnect()
            self.logger.info("Table system shut down")
        except Exception as e:
            self.logger.error(f"Error shutting down table system: {e}")
