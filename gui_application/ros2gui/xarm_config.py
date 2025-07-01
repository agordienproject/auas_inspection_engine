from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QLineEdit, QHBoxLayout, QComboBox
import rclpy
from rclpy.node import Node
import threading
import os

class XarmConfigWindow(QWidget):
    PROGRAMS_FOLDER = os.path.expanduser("~/xarm_ws/src/xarm_ros2/xarm_example/programs")
    def __init__(self, parent=None):
        super().__init__()
        self.setWindowTitle("Configuration Xarm")
        self.setGeometry(300, 300, 400, 400)
        self.parent = parent
        self.connected = False
        self.program_files = []
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.status = QLabel("Status: Disconnected")
        layout.addWidget(self.status)

        # IP input (not used for simulation, but kept for real robot)
        ip_layout = QHBoxLayout()
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("Xarm IP address (not needed for simulation)")
        ip_layout.addWidget(QLabel("IP:"))
        ip_layout.addWidget(self.ip_input)
        layout.addLayout(ip_layout)

        # Connect to simulation button
        self.sim_connect_btn = QPushButton("Connect to Simulated Xarm")
        self.sim_connect_btn.clicked.connect(self.connect_sim_xarm)
        layout.addWidget(self.sim_connect_btn)

        # Program selection
        self.program_combo = QComboBox()
        self.refresh_program_list()
        layout.addWidget(QLabel("Select a program:"))
        layout.addWidget(self.program_combo)

        # Program control buttons
        self.load_btn = QPushButton("Load Program")
        self.load_btn.setEnabled(False)
        self.load_btn.clicked.connect(self.load_program)
        layout.addWidget(self.load_btn)

        self.start_btn = QPushButton("Start Program")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_program)
        layout.addWidget(self.start_btn)

        self.pause_btn = QPushButton("Pause Program")
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self.pause_program)
        layout.addWidget(self.pause_btn)

        self.stop_btn = QPushButton("Stop Program")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_program)
        layout.addWidget(self.stop_btn)

        # Example: Add more controls here (move, start program, etc.)
        self.enable_btn = QPushButton("Enable Robot")
        self.enable_btn.setEnabled(False)
        self.enable_btn.clicked.connect(self.enable_robot)
        layout.addWidget(self.enable_btn)

        self.disable_btn = QPushButton("Disable Robot")
        self.disable_btn.setEnabled(False)
        self.disable_btn.clicked.connect(self.disable_robot)
        layout.addWidget(self.disable_btn)

        self.setLayout(layout)

    def refresh_program_list(self):
        self.program_combo.clear()
        if os.path.isdir(self.PROGRAMS_FOLDER):
            self.program_files = [f for f in os.listdir(self.PROGRAMS_FOLDER) if f.endswith('.xarmp')]  # or .py/.xml as needed
            self.program_combo.addItems(self.program_files)
        else:
            self.program_combo.addItem("No programs found")

    def connect_sim_xarm(self):
        # Use the parent node to check for /joint_states topic
        def check_topic():
            # Use the main window's ROS2 node
            node = self.parent  # ROS2GuiApp is a Node
            topics = node.get_topic_names_and_types()
            found = any(name == '/joint_states' for name, _ in topics)
            if found:
                self.status.setText("✅ Connected to Simulated Xarm (/joint_states found)")
                self.connected = True
                self.enable_btn.setEnabled(True)
                self.disable_btn.setEnabled(True)
                self.load_btn.setEnabled(True)
                self.start_btn.setEnabled(True)
                self.pause_btn.setEnabled(True)
                self.stop_btn.setEnabled(True)
            else:
                self.status.setText("❌ Simulated Xarm not found (no /joint_states topic)")
                self.connected = False
                self.enable_btn.setEnabled(False)
                self.disable_btn.setEnabled(False)
                self.load_btn.setEnabled(False)
                self.start_btn.setEnabled(False)
                self.pause_btn.setEnabled(False)
                self.stop_btn.setEnabled(False)
        threading.Thread(target=check_topic, daemon=True).start()

    def load_program(self):
        idx = self.program_combo.currentIndex()
        if idx < 0 or not self.program_files:
            self.status.setText("❌ No program selected")
            return
        program_name = self.program_files[idx]
        # Here you would call the ROS2 service/action to load the program
        self.status.setText(f"Loaded program: {program_name} (simulated)")

    def start_program(self):
        idx = self.program_combo.currentIndex()
        if idx < 0 or not self.program_files:
            self.status.setText("❌ No program selected")
            return
        program_name = self.program_files[idx]
        # Here you would call the ROS2 service/action to start the program
        self.status.setText(f"Started program: {program_name} (simulated)")

    def pause_program(self):
        idx = self.program_combo.currentIndex()
        if idx < 0 or not self.program_files:
            self.status.setText("❌ No program selected")
            return
        program_name = self.program_files[idx]
        # Here you would call the ROS2 service/action to pause the program
        self.status.setText(f"Paused program: {program_name} (simulated)")

    def stop_program(self):
        idx = self.program_combo.currentIndex()
        if idx < 0 or not self.program_files:
            self.status.setText("❌ No program selected")
            return
        program_name = self.program_files[idx]
        # Here you would call the ROS2 service/action to stop the program
        self.status.setText(f"Stopped program: {program_name} (simulated)")

    def enable_robot(self):
        self.status.setText("Robot enabled (simulated)")

    def disable_robot(self):
        self.status.setText("Robot disabled (simulated)")
