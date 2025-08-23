"""
Table System using serial connection functions from GUI application
"""
import logging
import time
from typing import Dict, Any
import serial
import serial.tools.list_ports

from systems.base_system import BaseSystem


class TableController:
    """Controller class for rotating table operations (from GUI application)"""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.serial_connection = None
        self.connected = False
        self.port = None
        self.baudrate = None

    def connect(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        """Connect to table via serial"""
        self.logger.debug("[DEBUG] TableController.connect() called with port={port}, baudrate={baudrate}, timeout={timeout}")
        try:
            # Check if the port exists first
            self.logger.debug("[DEBUG] Checking available ports...")
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            self.logger.debug("[DEBUG] Available ports: %s", available_ports)
            if port not in available_ports:
                self.logger.debug("[DEBUG] Port %s not found in available ports", port)
                raise ConnectionError(f"Port {port} not found. Available ports: {available_ports}")

            self.logger.debug("[DEBUG] Port {port} found, proceeding with connection")
            self.port = port
            self.baudrate = baudrate

            self.logger.debug("[DEBUG] Creating serial connection...")
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout
            )

            self.logger.debug("[DEBUG] Serial connection created, checking if open...")
            if self.serial_connection.is_open:
                self.logger.debug("[DEBUG] Serial connection is open, setting connected=True")
                self.connected = True
                # Wait a bit for Arduino to initialize (like the GUI does)
                self.logger.debug("[DEBUG] Waiting 2 seconds for Arduino to initialize...")
                time.sleep(2)
                self.logger.debug("[DEBUG] Connection successful")
                return True
            else:
                self.logger.debug("[DEBUG] Serial connection is not open")
                return False

        except Exception as e:
            self.logger.debug("[DEBUG] Exception in TableController.connect(): {e}")
            self.connected = False
            raise e

    def disconnect(self):
        """Disconnect from table"""
        self.logger.debug("[DEBUG] TableController.disconnect() called")
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.logger.debug("[DEBUG] Closing serial connection...")
                self.serial_connection.close()
                self.logger.debug("[DEBUG] Serial connection closed")
            self.connected = False
            self.logger.debug("[DEBUG] Disconnect successful")
            return True
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in disconnect(): {e}")
            self.connected = False
            return False

    def send_command(self, command: str):
        """Send command to table"""
        self.logger.debug("[DEBUG] TableController.send_command() called with command='{command}'")
        if not self.connected or not self.serial_connection:
            self.logger.debug("[DEBUG] Not connected to table (connected={self.connected}, serial_connection={self.serial_connection is not None})")
            raise ConnectionError("Not connected to table")

        try:
            self.logger.debug("[DEBUG] Sending command to serial port...")
            self.serial_connection.write((command + '\n').encode())
            # Add a small delay to ensure command is processed
            time.sleep(0.1)
            self.logger.debug("[DEBUG] Command sent successfully")
            return True
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in send_command(): {e}")
            raise RuntimeError(f"Failed to send command: {e}")

    def start_rotation(self):
        """Start table rotation"""
        self.logger.debug("[DEBUG] TableController.start_rotation() called")
        return self.send_command("rotate_table")

    def stop_rotation(self):
        """Stop table rotation"""
        self.logger.debug("[DEBUG] TableController.stop_rotation() called")
        return self.send_command("stop_table")

    def move_by_degree(self, degrees: int):
        """Move table by specific degrees"""
        self.logger.debug("[DEBUG] TableController.move_by_degree() called with degrees={degrees}")
        return self.send_command(f"move_table_{degrees}")

    def test_connection(self):
        """Test if connection is working"""
        self.logger.debug("[DEBUG] TableController.test_connection() called")
        if not self.connected:
            self.logger.debug("[DEBUG] Not connected, returning False")
            return False

        try:
            # Try sending a simple status command
            self.logger.debug("[DEBUG] Sending status command...")
            self.send_command("status")
            self.logger.debug("[DEBUG] Status command sent successfully")
            return True
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in test_connection(): {e}")
            return False


class TableSystem(BaseSystem):
    """Table system implementation using GUI connection functions"""

    def __init__(self, name: str, config: Dict[str, Any]):
        self.logger.debug("[DEBUG] TableSystem.__init__() called with name='{name}', config={config}")
        super().__init__(name, config)
        self.table_controller = TableController()
        self.logger = logging.getLogger(__name__)
        self.logger.debug("[DEBUG] TableSystem initialized")

    def initialize(self) -> bool:
        """Initialize the table system"""
        self.logger.debug("[DEBUG] TableSystem.initialize() called")
        try:
            self.logger.info("Initializing table system: %s", self.name)
            self.logger.debug("[DEBUG] Table system initialized successfully")
            return True
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in initialize(): {e}")
            self.logger.error("Failed to initialize table system: %s", e)
            return False

    def connect(self) -> bool:
        """Connect to the table"""
        self.logger.debug("[DEBUG] TableSystem.connect() called")
        try:
            port = self.config.get('port', '/dev/ttyACM0')
            baudrate = self.config.get('baudrate', 9600)
            timeout = self.config.get('timeout', 1.0)  # Use shorter timeout like GUI

            self.logger.debug("[DEBUG] Connection parameters - port: {port}, baudrate: {baudrate}, timeout: {timeout}")
            self.logger.info("Attempting to connect to table at %s (baudrate: %s)", port, baudrate)

            # Check if Arduino is connected (like GUI does)
            self.logger.debug("[DEBUG] Checking if Arduino is connected...")
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            self.logger.debug("[DEBUG] Available ports: {available_ports}")
            if port not in available_ports:
                self.logger.debug("[DEBUG] Arduino not detected at {port}")
                self.logger.error("Arduino not detected at %s. Available ports: %s", port, available_ports)
                return False

            self.logger.debug("[DEBUG] Arduino detected, attempting connection...")
            success = self.table_controller.connect(port, baudrate, timeout)
            if success:
                self.logger.debug("[DEBUG] Table connection successful")
                self.logger.info("Table connected successfully")
                return True
            else:
                self.logger.debug("[DEBUG] Table connection failed")
                self.logger.error("Failed to connect to table")
                return False

        except Exception as e:
            self.logger.debug("[DEBUG] Exception in TableSystem.connect(): {e}")
            self.logger.error("Table connection error: %s", e)
            return False

    def disconnect(self) -> bool:
        """Disconnect from the table"""
        self.logger.debug("[DEBUG] TableSystem.disconnect() called")
        try:
            success = self.table_controller.disconnect()
            if success:
                self.logger.debug("[DEBUG] Table disconnection successful")
                self.logger.info("Table disconnected successfully")
            else:
                self.logger.debug("[DEBUG] Table disconnection failed")
            return success
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in TableSystem.disconnect(): {e}")
            self.logger.error("Table disconnection error: %s", e)
            return False

    def test_connection(self) -> Dict[str, Any]:
        """Test table connection"""
        self.logger.debug("[DEBUG] TableSystem.test_connection() called")
        try:
            port = self.config.get('port', '/dev/ttyACM0')
            self.logger.debug("[DEBUG] Testing connection for port: {port}")

            # First check if Arduino is detected (like GUI does)
            self.logger.debug("[DEBUG] Checking if Arduino is detected...")
            available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
            self.logger.debug("[DEBUG] Available ports: {available_ports}")
            if port not in available_ports:
                self.logger.debug("[DEBUG] Arduino not detected at {port}")
                return {
                    'status': 'not_available',
                    'message': f'Arduino not detected at {port}',
                    'details': {
                        'available_ports': available_ports,
                        'expected_port': port
                    }
                }

            self.logger.debug("[DEBUG] Arduino detected, checking connection status...")
            # Try to connect if not already connected
            if not self.table_controller.connected:
                self.logger.debug("[DEBUG] Not connected, attempting to connect...")
                if not self.connect():
                    self.logger.debug("[DEBUG] Connection attempt failed")
                    return {
                        'status': 'not_available',
                        'message': 'Table connection failed - check serial port'
                    }

            self.logger.debug("[DEBUG] Testing the connection...")
            # Test if the connection is working
            if self.table_controller.test_connection():
                self.logger.debug("[DEBUG] Connection test successful")
                return {
                    'status': 'available',
                    'message': 'Table connected and ready',
                    'details': {
                        'port': self.table_controller.port,
                        'baudrate': self.table_controller.baudrate
                    }
                }
            else:
                self.logger.debug("[DEBUG] Connection test failed")
                return {
                    'status': 'not_available',
                    'message': 'Table connection test failed'
                }

        except Exception as e:
            self.logger.debug("[DEBUG] Exception in TableSystem.test_connection(): {e}")
            self.logger.error("Table connection test failed: %s", e)
            return {
                'status': 'error',
                'message': f'Table test error: {str(e)}'
            }

    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the table"""
        step_name = step_config.get('name', 'unknown')
        self.logger.debug("[DEBUG] TableSystem.execute_step() called with step_name='{step_name}', step_config={step_config}")
        self.logger.info("Executing table step: %s", step_name)

        try:
            # Ensure connection
            self.logger.debug("[DEBUG] Checking connection status...")
            if not self.table_controller.connected:
                self.logger.debug("[DEBUG] Not connected, attempting to connect...")
                if not self.connect():
                    self.logger.debug("[DEBUG] Failed to connect to table")
                    raise ConnectionError("Cannot connect to table")

            # Execute the step based on step configuration
            action = step_config.get('action', 'rotate')
            self.logger.debug("[DEBUG] Executing action: {action}")

            if action == 'rotate':
                return self._start_rotation(step_config)
            elif action == 'stop':
                return self._stop_rotation(step_config)
            elif action == 'move':
                return self._move_by_degree(step_config)
            else:
                self.logger.debug("[DEBUG] Unknown action: {action}")
                raise ValueError(f"Unknown table action: {action}")

        except Exception as e:
            self.logger.debug("[DEBUG] Exception in execute_step(): {e}")
            self.logger.error("Table step execution failed: %s", e)
            raise

    def _start_rotation(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Start table rotation"""
        duration = step_config.get('duration', 0)
        self.logger.debug("[DEBUG] TableSystem._start_rotation() called with duration={duration}")

        if duration > 0:
            self.logger.debug("[DEBUG] Starting timed rotation for {duration} seconds")
            self.logger.info("Starting table rotation for %d seconds", duration)
        else:
            self.logger.debug("[DEBUG] Starting continuous rotation")
            self.logger.info("Starting table rotation (continuous)")

        self.logger.info("Sending rotate_table command...")
        success = self.table_controller.start_rotation()
        if not success:
            self.logger.debug("[DEBUG] Failed to start table rotation")
            raise RuntimeError("Failed to start table rotation")

        self.logger.debug("[DEBUG] Rotation command sent successfully")
        self.logger.info("Rotate command sent successfully")

        # If duration is specified, wait for that time then stop
        if duration > 0:
            self.logger.debug("[DEBUG] Waiting {duration} seconds...")
            self.logger.info("Waiting %d seconds...", duration)
            time.sleep(duration)

            self.logger.debug("[DEBUG] Duration completed, attempting to stop rotation")
            self.logger.info("Duration completed, stopping rotation")
            stop_success = self.table_controller.stop_rotation()
            if not stop_success:
                self.logger.debug("[DEBUG] Failed to stop rotation after duration")
                self.logger.warning("Failed to stop rotation after duration")
            else:
                self.logger.debug("[DEBUG] Table rotation stopped successfully after duration")
                self.logger.info("Table rotation stopped successfully")

        self.logger.debug("[DEBUG] Returning success result for rotation")
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
        self.logger.debug("[DEBUG] TableSystem._stop_rotation() called")
        self.logger.info("Stopping table rotation")

        success = self.table_controller.stop_rotation()
        if not success:
            self.logger.debug("[DEBUG] Failed to stop table rotation")
            raise RuntimeError("Failed to stop table rotation")

        self.logger.debug("[DEBUG] Table rotation stopped successfully")
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

        self.logger.debug("[DEBUG] TableSystem._move_by_degree() called with degrees={degrees}")
        self.logger.info("Moving table by %d degrees", degrees)

        success = self.table_controller.move_by_degree(degrees)
        if not success:
            self.logger.debug("[DEBUG] Failed to move table by {degrees} degrees")
            raise RuntimeError(f"Failed to move table by {degrees} degrees")

        self.logger.debug("[DEBUG] Table moved by {degrees} degrees successfully")
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
        self.logger.debug("[DEBUG] TableSystem.shutdown() called")
        try:
            # Stop rotation if running
            try:
                self.logger.debug("[DEBUG] Attempting to stop rotation before shutdown...")
                self.table_controller.stop_rotation()
                self.logger.debug("[DEBUG] Rotation stopped")
            except Exception as e:
                self.logger.debug("[DEBUG] Error stopping rotation during shutdown: {e}")
                pass  # Ignore errors if not rotating

            # Disconnect
            self.logger.debug("[DEBUG] Disconnecting...")
            self.disconnect()
            self.logger.debug("[DEBUG] Table system shut down successfully")
            self.logger.info("Table system shut down")
        except Exception as e:
            self.logger.debug("[DEBUG] Error shutting down table system: {e}")
            self.logger.error("Error shutting down table system: %s", e)
