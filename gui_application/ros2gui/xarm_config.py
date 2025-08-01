"""
Simple xArm Configuration Window using Python SDK (No ROS2 Required)

This is a complete replacement for the complex ROS2-based xArm control.
It uses the official xArm Python SDK for direct TCP/IP communication.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QLineEdit, QHBoxLayout, QTextEdit, QGroupBox
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
import threading
import time

# Try to import xArm Python SDK (much simpler than ROS2)
try:
    from xarm.wrapper import XArmAPI
    XARM_SDK_AVAILABLE = True
    print("✅ xArm Python SDK available")
except ImportError as e:
    print(f"Warning: xArm Python SDK not found: {e}")
    print("Install with: pip install xarm-python-sdk")
    XARM_SDK_AVAILABLE = False
    
    # Create dummy class for development
    class XArmAPI:
        def __init__(self, ip):
            self.ip = ip
        def connect(self): return 0
        def disconnect(self): return 0
        def clean_error(self): return 0
        def clean_warn(self): return 0
        def set_state(self, state): return 0
        def motion_enable(self, enable): return 0
        def set_mode(self, mode): return 0
        def move_gohome(self, wait=True): return 0
        def emergency_stop(self): return 0
        def get_position(self): return [0, [300, 0, 400, 0, 0, 0]]
        def set_position(self, *args, **kwargs): return 0


class XarmConfigWindow(QWidget):
    """Simple xArm configuration window using Python SDK"""
    
    # Signals for thread-safe GUI updates
    connection_success = pyqtSignal(str)  # IP address
    connection_failed = pyqtSignal(str)   # Error message
    status_update = pyqtSignal(str)       # Status message
    
    def __init__(self, parent=None):
        super().__init__()
        self.setWindowTitle("xArm Control (Python SDK)")
        self.setGeometry(300, 300, 600, 800)
        self.parent = parent
        
        # Robot connection
        self.arm = None
        self.connected = False
        self.enabled = False
        
        # Timer for position updates
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.update_position)
        
        # Connect signals
        self.connection_success.connect(self._on_connection_success)
        self.connection_failed.connect(self._on_connection_failed)
        self.status_update.connect(self._on_status_update)
        
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        
        # Header
        header_label = QLabel("🎯 Simple xArm Control (Python SDK)")
        header_label.setStyleSheet("QLabel { font-size: 16px; font-weight: bold; color: #155724; margin: 10px; }")
        layout.addWidget(header_label)
        
        # SDK Status
        if not XARM_SDK_AVAILABLE:
            warning_label = QLabel("⚠️ xArm Python SDK not installed. Install with: pip install xarm-python-sdk")
            warning_label.setStyleSheet("QLabel { background-color: #fff3cd; color: #856404; padding: 8px; border: 1px solid #ffeaa7; border-radius: 4px; }")
            layout.addWidget(warning_label)
        else:
            status_label = QLabel("✅ xArm Python SDK is available")
            status_label.setStyleSheet("QLabel { background-color: #d4edda; color: #155724; padding: 8px; border: 1px solid #c3e6cb; border-radius: 4px; }")
            layout.addWidget(status_label)
        
        # Connection section
        connection_group = QGroupBox("Robot Connection")
        connection_layout = QVBoxLayout()
        
        # Status display
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; }")
        connection_layout.addWidget(self.status_label)
        
        # IP Address input
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("Robot IP:"))
        self.ip_input = QLineEdit("192.168.1.222")
        self.ip_input.setPlaceholderText("Enter xArm IP address")
        ip_layout.addWidget(self.ip_input)
        connection_layout.addLayout(ip_layout)
        
        # Connection buttons
        conn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect to Robot")
        self.connect_btn.clicked.connect(self.connect_robot)
        self.connect_btn.setStyleSheet("QPushButton { background-color: #007bff; color: white; padding: 8px; border-radius: 4px; }")
        conn_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("QPushButton { background-color: #6c757d; color: white; padding: 8px; border-radius: 4px; }")
        conn_layout.addWidget(self.disconnect_btn)
        connection_layout.addLayout(conn_layout)
        
        connection_group.setLayout(connection_layout)
        layout.addWidget(connection_group)
        
        # Robot control section
        control_group = QGroupBox("Robot Control")
        control_layout = QVBoxLayout()
        
        # Enable/Disable buttons
        enable_layout = QHBoxLayout()
        self.enable_btn = QPushButton("Enable Robot")
        self.enable_btn.clicked.connect(self.enable_robot)
        self.enable_btn.setEnabled(False)
        self.enable_btn.setStyleSheet("QPushButton { background-color: #28a745; color: white; padding: 8px; border-radius: 4px; }")
        enable_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("Disable Robot")
        self.disable_btn.clicked.connect(self.disable_robot)
        self.disable_btn.setEnabled(False)
        self.disable_btn.setStyleSheet("QPushButton { background-color: #ffc107; color: black; padding: 8px; border-radius: 4px; }")
        enable_layout.addWidget(self.disable_btn)
        control_layout.addLayout(enable_layout)
        
        # Movement buttons
        move_layout = QHBoxLayout()
        self.home_btn = QPushButton("Move to Home")
        self.home_btn.clicked.connect(self.move_home)
        self.home_btn.setEnabled(False)
        self.home_btn.setStyleSheet("QPushButton { background-color: #17a2b8; color: white; padding: 8px; border-radius: 4px; }")
        move_layout.addWidget(self.home_btn)
        
        self.stop_btn = QPushButton("Emergency Stop")
        self.stop_btn.clicked.connect(self.emergency_stop)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("QPushButton { background-color: #dc3545; color: white; font-weight: bold; padding: 8px; border-radius: 4px; }")
        move_layout.addWidget(self.stop_btn)
        control_layout.addLayout(move_layout)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Example movements section
        example_group = QGroupBox("Example Movements")
        example_layout = QVBoxLayout()
        
        # Position input
        pos_layout = QVBoxLayout()
        pos_layout.addWidget(QLabel("Move to Custom Position (X, Y, Z in mm):"))
        
        pos_input_layout = QHBoxLayout()
        self.x_input = QLineEdit("300")
        self.x_input.setPlaceholderText("X")
        pos_input_layout.addWidget(QLabel("X:"))
        pos_input_layout.addWidget(self.x_input)
        
        self.y_input = QLineEdit("0")
        self.y_input.setPlaceholderText("Y")
        pos_input_layout.addWidget(QLabel("Y:"))
        pos_input_layout.addWidget(self.y_input)
        
        self.z_input = QLineEdit("400")
        self.z_input.setPlaceholderText("Z")
        pos_input_layout.addWidget(QLabel("Z:"))
        pos_input_layout.addWidget(self.z_input)
        pos_layout.addLayout(pos_input_layout)
        
        self.move_to_btn = QPushButton("Move to Position")
        self.move_to_btn.clicked.connect(self.move_to_position)
        self.move_to_btn.setEnabled(False)
        self.move_to_btn.setStyleSheet("QPushButton { background-color: #fd7e14; color: white; padding: 8px; border-radius: 4px; }")
        pos_layout.addWidget(self.move_to_btn)
        example_layout.addLayout(pos_layout)
        
        # Quick movement buttons
        quick_label = QLabel("Quick Movements:")
        quick_label.setStyleSheet("QLabel { font-weight: bold; margin-top: 10px; }")
        example_layout.addWidget(quick_label)
        
        quick_layout1 = QHBoxLayout()
        self.move_up_btn = QPushButton("Move Up (+50mm)")
        self.move_up_btn.clicked.connect(lambda: self.quick_move(0, 0, 50))
        self.move_up_btn.setEnabled(False)
        quick_layout1.addWidget(self.move_up_btn)
        
        self.move_down_btn = QPushButton("Move Down (-50mm)")
        self.move_down_btn.clicked.connect(lambda: self.quick_move(0, 0, -50))
        self.move_down_btn.setEnabled(False)
        quick_layout1.addWidget(self.move_down_btn)
        example_layout.addLayout(quick_layout1)
        
        quick_layout2 = QHBoxLayout()
        self.move_forward_btn = QPushButton("Move Forward (+50mm)")
        self.move_forward_btn.clicked.connect(lambda: self.quick_move(50, 0, 0))
        self.move_forward_btn.setEnabled(False)
        quick_layout2.addWidget(self.move_forward_btn)
        
        self.move_back_btn = QPushButton("Move Back (-50mm)")
        self.move_back_btn.clicked.connect(lambda: self.quick_move(-50, 0, 0))
        self.move_back_btn.setEnabled(False)
        quick_layout2.addWidget(self.move_back_btn)
        example_layout.addLayout(quick_layout2)
        
        example_group.setLayout(example_layout)
        layout.addWidget(example_group)
        
        # Status display section
        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()
        
        self.position_text = QTextEdit()
        self.position_text.setMaximumHeight(120)
        self.position_text.setReadOnly(True)
        self.position_text.setStyleSheet("QTextEdit { background-color: #f8f9fa; font-family: monospace; }")
        status_layout.addWidget(self.position_text)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        self.setLayout(layout)

    def connect_robot(self):
        """Connect to xArm robot using Python SDK (following working example pattern)"""
        ip = self.ip_input.text().strip()
        if not ip:
            self.status_label.setText("❌ Please enter IP address")
            return
            
        def do_connect():
            try:
                self.status_update.emit("🔄 Connecting...")
                print(f"🔄 Attempting to connect to xArm at {ip}...")
                
                # Create xArm API instance (like working example)
                self.arm = XArmAPI(ip)
                print(f"📦 XArmAPI instance created for {ip}")
                
                # Initialize robot (following working example pattern)
                # The working example calls these FIRST, which establishes connection
                print("� Initializing robot (motion_enable, set_mode, set_state)...")
                
                # These calls will establish the connection automatically
                result1 = self.arm.motion_enable(enable=True)
                print(f"📋 motion_enable result: {result1}")
                
                result2 = self.arm.set_mode(0)
                print(f"📋 set_mode result: {result2}")
                
                result3 = self.arm.set_state(state=0)
                print(f"📋 set_state result: {result3}")
                
                # Test connection by getting version (like working example)
                try:
                    version = self.arm.get_version()
                    print(f"🤖 Robot version: {version}")
                    
                    state = self.arm.get_state()
                    print(f"🤖 Robot state: {state}")
                    
                    # If we got here, connection is successful
                    print(f"✅ Successfully connected and initialized xArm at {ip}")
                    self.connection_success.emit(ip)  # Use signal instead of direct GUI update
                    
                    # Start position updates
                    # Note: Timer will be started by signal handler in main thread
                    
                except Exception as e:
                    print(f"❌ Failed to get robot info: {e}")
                    self.connection_failed.emit(str(e))  # Use signal instead of direct GUI update
                    
            except Exception as e:
                print(f"❌ Exception during connection: {e}")
                print(f"❌ Exception type: {type(e)}")
                import traceback
                traceback.print_exc()
                self.connection_failed.emit(str(e))  # Use signal instead of direct GUI update
                
        threading.Thread(target=do_connect, daemon=True).start()

    def disconnect_robot(self):
        """Disconnect from xArm robot"""
        try:
            self.position_timer.stop()
            
            if self.arm and self.connected:
                self.arm.disconnect()
                
            self.connected = False
            self.enabled = False
            self.status_label.setText("Status: Disconnected")
            
            # Reset UI
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.enable_btn.setEnabled(False)
            self.disable_btn.setEnabled(False)
            self.home_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.move_to_btn.setEnabled(False)
            self.move_up_btn.setEnabled(False)
            self.move_down_btn.setEnabled(False)
            self.move_forward_btn.setEnabled(False)
            self.move_back_btn.setEnabled(False)
            
            self.position_text.clear()
            
        except Exception as e:
            self.status_label.setText(f"❌ Disconnect error: {str(e)}")

    def enable_robot(self):
        """Enable robot motion"""
        if not self.connected:
            return
            
        def do_enable():
            try:
                self.status_label.setText("🔄 Enabling robot...")
                
                # Clear any errors
                self.arm.clean_error()
                self.arm.clean_warn()
                time.sleep(0.1)
                
                # Set robot to normal state
                result = self.arm.set_state(0)  # 0 = normal state
                if result != 0:
                    self.status_label.setText(f"❌ Failed to set state: {result}")
                    return
                time.sleep(0.1)
                
                # Enable motion
                result = self.arm.motion_enable(True)
                if result != 0:
                    self.status_label.setText(f"❌ Failed to enable motion: {result}")
                    return
                time.sleep(0.1)
                
                # Set position mode
                result = self.arm.set_mode(0)  # 0 = position mode
                if result != 0:
                    self.status_label.setText(f"❌ Failed to set mode: {result}")
                    return
                
                self.enabled = True
                self.status_label.setText("✅ Robot enabled and ready")
                
                # Enable movement buttons
                self.enable_btn.setEnabled(False)
                self.disable_btn.setEnabled(True)
                self.home_btn.setEnabled(True)
                self.move_to_btn.setEnabled(True)
                self.move_up_btn.setEnabled(True)
                self.move_down_btn.setEnabled(True)
                self.move_forward_btn.setEnabled(True)
                self.move_back_btn.setEnabled(True)
                
            except Exception as e:
                self.status_label.setText(f"❌ Enable error: {str(e)}")
                
        threading.Thread(target=do_enable, daemon=True).start()

    def disable_robot(self):
        """Disable robot motion"""
        if not self.connected:
            return
            
        def do_disable():
            try:
                self.status_label.setText("🔄 Disabling robot...")
                
                # Disable motion
                result = self.arm.motion_enable(False)
                if result != 0:
                    self.status_label.setText(f"❌ Failed to disable motion: {result}")
                    return
                
                # Set to stopped state
                result = self.arm.set_state(4)  # 4 = stopped state
                if result != 0:
                    self.status_label.setText(f"❌ Failed to set stopped state: {result}")
                    return
                
                self.enabled = False
                self.status_label.setText("🔴 Robot disabled")
                
                # Disable movement buttons
                self.enable_btn.setEnabled(True)
                self.disable_btn.setEnabled(False)
                self.home_btn.setEnabled(False)
                self.move_to_btn.setEnabled(False)
                self.move_up_btn.setEnabled(False)
                self.move_down_btn.setEnabled(False)
                self.move_forward_btn.setEnabled(False)
                self.move_back_btn.setEnabled(False)
                
            except Exception as e:
                self.status_label.setText(f"❌ Disable error: {str(e)}")
                
        threading.Thread(target=do_disable, daemon=True).start()

    def move_home(self):
        """Move robot to home position"""
        if not self.connected or not self.enabled:
            self.status_label.setText("❌ Robot not connected or enabled")
            return
            
        def do_move_home():
            try:
                self.status_label.setText("🔄 Moving to home position...")
                result = self.arm.move_gohome(wait=True)
                
                if result == 0:
                    self.status_label.setText("🏠 Moved to home position")
                else:
                    self.status_label.setText(f"❌ Move home failed: Error code {result}")
                    
            except Exception as e:
                self.status_label.setText(f"❌ Move home error: {str(e)}")
                
        threading.Thread(target=do_move_home, daemon=True).start()

    def emergency_stop(self):
        """Emergency stop the robot"""
        if not self.connected:
            return
            
        try:
            self.arm.emergency_stop()
            self.enabled = False
            self.status_label.setText("🛑 Emergency stop activated")
            
            # Reset movement buttons
            self.enable_btn.setEnabled(True)
            self.disable_btn.setEnabled(False)
            self.home_btn.setEnabled(False)
            self.move_to_btn.setEnabled(False)
            self.move_up_btn.setEnabled(False)
            self.move_down_btn.setEnabled(False)
            self.move_forward_btn.setEnabled(False)
            self.move_back_btn.setEnabled(False)
            
        except Exception as e:
            self.status_label.setText(f"❌ Emergency stop error: {str(e)}")

    def move_to_position(self):
        """Move to custom position"""
        if not self.connected or not self.enabled:
            self.status_label.setText("❌ Robot not connected or enabled")
            return
            
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except ValueError:
            self.status_label.setText("❌ Invalid position values")
            return
            
        def do_move():
            try:
                self.status_label.setText(f"🔄 Moving to position ({x}, {y}, {z})...")
                result = self.arm.set_position(x, y, z, 0, 0, 0, speed=50, wait=True)
                
                if result == 0:
                    self.status_label.setText(f"✅ Moved to position ({x}, {y}, {z})")
                else:
                    self.status_label.setText(f"❌ Move failed: Error code {result}")
                    
            except Exception as e:
                self.status_label.setText(f"❌ Move error: {str(e)}")
                
        threading.Thread(target=do_move, daemon=True).start()

    def quick_move(self, dx, dy, dz):
        """Quick relative movement"""
        if not self.connected or not self.enabled:
            self.status_label.setText("❌ Robot not connected or enabled")
            return
            
        def do_move():
            try:
                # Get current position
                code, current_pos = self.arm.get_position()
                if code != 0:
                    self.status_label.setText("❌ Could not get current position")
                    return
                
                # Calculate new position
                new_x = current_pos[0] + dx
                new_y = current_pos[1] + dy
                new_z = current_pos[2] + dz
                
                self.status_label.setText(f"🔄 Moving by ({dx}, {dy}, {dz})...")
                result = self.arm.set_position(new_x, new_y, new_z, 
                                              current_pos[3], current_pos[4], current_pos[5], 
                                              speed=50, wait=True)
                
                if result == 0:
                    self.status_label.setText(f"✅ Moved by ({dx}, {dy}, {dz})")
                else:
                    self.status_label.setText(f"❌ Move failed: Error code {result}")
                    
            except Exception as e:
                self.status_label.setText(f"❌ Move error: {str(e)}")
                
        threading.Thread(target=do_move, daemon=True).start()

    def update_position(self):
        """Update position display"""
        if not self.connected:
            return
            
        try:
            # Get position
            code, position = self.arm.get_position()
            
            status_text = ""
            if code == 0:
                status_text += f"Position (mm):\n"
                status_text += f"  X: {position[0]:.1f}\n"
                status_text += f"  Y: {position[1]:.1f}\n"
                status_text += f"  Z: {position[2]:.1f}\n"
                status_text += f"  Roll: {position[3]:.1f}°\n"
                status_text += f"  Pitch: {position[4]:.1f}°\n"
                status_text += f"  Yaw: {position[5]:.1f}°\n"
            else:
                status_text += f"Position: Error code {code}\n"
            
            status_text += f"\nStatus:\n"
            status_text += f"  Connected: {self.connected}\n"
            status_text += f"  Enabled: {self.enabled}\n"
            
            self.position_text.setText(status_text)
            
        except Exception as e:
            self.position_text.setText(f"Error updating position: {e}")
            
    def _on_connection_success(self, ip):
        """Handle successful connection (called in main thread)"""
        self.connected = True
        self.enabled = True  # Robot is already enabled
        self.status_label.setText(f"✅ Connected and ready at {ip}")
        
        # Update UI
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.enable_btn.setEnabled(False)  # Already enabled
        self.disable_btn.setEnabled(True)
        self.home_btn.setEnabled(True)
        self.stop_btn.setEnabled(True)
        self.move_to_btn.setEnabled(True)
        self.move_up_btn.setEnabled(True)
        self.move_down_btn.setEnabled(True)
        self.move_forward_btn.setEnabled(True)
        self.move_back_btn.setEnabled(True)
        
        # Start position updates (safe to do in main thread)
        self.position_timer.start(2000)  # Update every 2 seconds
        
    def _on_connection_failed(self, error_msg):
        """Handle connection failure (called in main thread)"""
        self.status_label.setText(f"❌ Connection failed: {error_msg}")
        
    def _on_status_update(self, message):
        """Handle status updates (called in main thread)"""
        self.status_label.setText(message)

# Test function
if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    window = XarmConfigWindow()
    window.show()
    sys.exit(app.exec_())
