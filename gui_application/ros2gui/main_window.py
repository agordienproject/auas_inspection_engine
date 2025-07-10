import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import QTimer
from . import utils
from .gantry_config import GantryConfigWindow
# from .camera_config import CameraConfigWindow  # Disabled for now - requires ROS2
from .table_config import TableConfigWindow
from .xarm_config import XarmConfigWindow  # Use simple SDK version instead
from .scanner_config import ScannerConfigWindow

# No ROS2 required for xArm SDK version
print("🎯 Running in standalone mode (xArm SDK, no ROS2 required)")
ROS2_AVAILABLE = False

# Create dummy classes for compatibility
class Node:
    def __init__(self, name): pass
    def get_topic_names_and_types(self): return []
    def create_client(self, *args): return None

class CvBridge:
    def imgmsg_to_cv2(self, *args): return None


class ROS2GuiApp(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.setWindowTitle("Inspection System GUI")
        self.setGeometry(100, 100, 500, 400)
        
        # Initialize ROS2 components only if available
        if ROS2_AVAILABLE:
            try:
                import rclpy
                rclpy.init()
                self.node = Node('inspection_gui_node')
                self.bridge = CvBridge()
                print("✅ ROS2 node initialized")
            except Exception as e:
                print(f"⚠️ ROS2 initialization failed: {e}")
                self.node = None
                self.bridge = None
        else:
            self.node = None
            self.bridge = None
            
        self.image_data = None
        self.timer = QTimer()
        self.processes = {}
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()
        self.control_buttons = {}
        layout.addLayout(self.make_button_pair("Talker", self.launch_talker, self.stop_talker))
        layout.addLayout(self.make_button_pair("Listener", self.launch_listener, self.stop_listener))
        layout.addLayout(self.make_config_button("Configure Rotating Table", self.open_table_config, key="rotate_table"))
        layout.addLayout(self.make_config_button("Configurer la caméra", self.open_camera_config, key="camera_config"))
        layout.addLayout(self.make_config_button("Configurer le Gantry", self.open_gantry_config, key="gantry_config"))
        layout.addLayout(self.make_config_button("Configurer le Xarm", self.open_xarm_config, key="xarm_config"))
        layout.addLayout(self.make_config_button("Configure scanCONTROL Scanner", self.open_scanner_config, key="scanner_config"))
        self.setLayout(layout)
        QTimer.singleShot(1000, self.check_hardware_status)

    def make_button_pair(self, label, start_func, stop_func, key=None):
        box = QHBoxLayout()
        box.addWidget(QLabel(label))
        start_btn = QPushButton("Start")
        stop_btn = QPushButton("Stop")
        start_btn.clicked.connect(start_func)
        stop_btn.clicked.connect(stop_func)
        box.addWidget(start_btn)
        box.addWidget(stop_btn)
        if key:
            self.control_buttons[key] = {"start": start_btn, "stop": stop_btn}
        return box

    def make_config_button(self, label, click_func, key=None):
        box = QHBoxLayout()
        box.addWidget(QLabel(label))
        button = QPushButton("Open")
        button.clicked.connect(click_func)
        box.addWidget(button)
        if key:
            self.control_buttons[key] = {"start": button}
        return box

    def start_process(self, key, command):
        utils.start_process(self.processes, key, command)

    def stop_process(self, key, match=None):
        utils.stop_process(self.processes, key, match)

    def check_hardware_status(self):
        arduino_ok = utils.check_arduino_connection()
        cam_ok = utils.check_camera_connection()
        scanner_ok = self.check_scanner_connection()
        if "rotate_table" in self.control_buttons:
            self.set_button_state(self.control_buttons["rotate_table"]["start"], arduino_ok, "green" if arduino_ok else "red")
        if "camera_config" in self.control_buttons:
            self.set_button_state(self.control_buttons["camera_config"]["start"], cam_ok, "green" if cam_ok else "red")
        if "gantry_config" in self.control_buttons:
            self.set_button_state(self.control_buttons["gantry_config"]["start"], True, "gray")
        if "scanner_config" in self.control_buttons:
            self.set_button_state(self.control_buttons["scanner_config"]["start"], scanner_ok, "green" if scanner_ok else "orange")

    def set_button_state(self, button, enabled, color):
        button.setEnabled(enabled)
        if enabled:
            button.setStyleSheet(f"background-color: {color}; color: white;")
        else:
            button.setStyleSheet("background-color: gray; color: white;")

    def launch_talker(self):
        self.start_process("talker", [
            "gnome-terminal", "--", "bash", "-c",
            "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py talker"
        ])

    def stop_talker(self):
        self.stop_process("talker", match="ros2 run demo_nodes_py talker")

    def launch_listener(self):
        self.start_process("listener", [
            "gnome-terminal", "--", "bash", "-c",
            "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py listener"
        ])

    def stop_listener(self):
        self.stop_process("listener", match="ros2 run demo_nodes_py listener")

    def open_table_config(self):
        if not utils.check_arduino_connection():
            print("[WARN] Arduino not detected.")
            return
        import serial
        try:
            self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            print("[INFO] Connected to Arduino.")
            self.table_window = TableConfigWindow(self.serial)
            self.table_window.show()
        except Exception as e:
            print(f"[ERROR] Couldn't open serial port: {e}")

    def stop_table(self):
        self.stop_process("rotate_table", match="python3 rotate_table.py")

    def open_camera_config(self):
        # Camera disabled for now - requires ROS2
        print("⚠️ Camera config requires ROS2 - currently disabled")
        return
        # if not utils.check_camera_connection():
        #     print("[WARN] Camera not detected.")
        #     return
        # self.camera_window = CameraConfigWindow(self, self.bridge)
        # self.camera_window.show()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data = cv_image
        except Exception as e:
            print(f"[ERROR] Failed to convert image: {e}")
    
    def open_gantry_config(self):
        self.gantry_window = GantryConfigWindow()
        self.gantry_window.show()

    def open_xarm_config(self):
        # Pass the node only if ROS2 is available (for compatibility)
        parent_node = self.node if hasattr(self, 'node') and self.node else None
        self.xarm_window = XarmConfigWindow(parent_node)
        self.xarm_window.show()

    def open_scanner_config(self):
        """Open the scanCONTROL scanner configuration window"""
        self.scanner_window = ScannerConfigWindow()
        self.scanner_window.show()

    def check_scanner_connection(self):
        """Check if scanCONTROL scanner SDK is available"""
        try:
            import sys
            sys.path.insert(0, '/home/agordien/projects/auas_inspection_engine/scanCONTROL-Linux-SDK-1-0-1/python_bindings')
            # import pylinllt as llt  # Temporarily disabled
            return False  # Disabled for now
        except ImportError:
            return False
