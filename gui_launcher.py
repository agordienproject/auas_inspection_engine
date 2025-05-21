import sys
import cv2
import os
import signal
import subprocess
import datetime

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout,
    QLabel, QHBoxLayout, QLineEdit, QGroupBox
)

from PyQt5.QtGui import QImage, QPixmap, QColor, QPalette
from PyQt5.QtCore import QTimer
import serial.tools.list_ports
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ROS2GuiApp(Node, QWidget):

    #----------- Constructor -----------#
    def __init__(self):
        Node.__init__(self, 'ros2_gui_launcher')
        QWidget.__init__(self)

        self.setWindowTitle("ROS 2 GUI Launcher")
        self.setGeometry(100, 100, 500, 400)

        self.bridge = CvBridge()
        self.image_data = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.processes = {}

        self.setup_ui()

    #----------- INIT -----------#
    def setup_ui(self):
        layout = QVBoxLayout()
        self.control_buttons = {}  # Nouveau dictionnaire pour stocker les boutons

        layout.addLayout(self.make_button_pair("Talker", self.launch_talker, self.stop_talker))
        layout.addLayout(self.make_button_pair("Listener", self.launch_listener, self.stop_listener))
        layout.addLayout(self.make_config_button("Configure Rotating Table", self.open_table_config, key="rotate_table"))
        layout.addLayout(self.make_button_pair("D435i Camera", self.launch_camera, self.stop_camera, key="camera"))
        layout.addLayout(self.make_button_pair("Capture Photo", self.capture_photo, lambda: None, key="capture_photo"))

        self.image_label = QLabel("📷 Camera not started")
        self.image_label.setFixedSize(640, 480)
        layout.addWidget(self.image_label)

        self.setLayout(layout)

        # Vérification initiale
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
            self.control_buttons[key] = {
                "start": start_btn,
                "stop": stop_btn
            }

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
        if key in self.processes and self.processes[key].poll() is None:
            print(f"[INFO] {key} is already running.")
            return
        print(f"[INFO] Starting {key}...")
        proc = subprocess.Popen(command, preexec_fn=os.setsid)
        self.processes[key] = proc

    def stop_process(self, key, match=None):
        proc = self.processes.get(key)
        if proc and proc.poll() is None:
            print(f"[INFO] Stopping {key} (from GUI)...")
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        else:
            print(f"[INFO] {key} not tracked or already stopped. Trying pattern match...")
            if match:
                subprocess.run(['pkill', '-f', match])
                print(f"[INFO] Killed matching process: {match}")
            else:
                print(f"[WARN] No pattern provided for {key}")


    #----------- CHECK CONNECTION -----------#
    def check_arduino_connection(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return "/dev/ttyACM0" in ports

    def check_camera_connection(self):
        try:
            output = subprocess.check_output(['lsusb']).decode()
            for line in output.splitlines():
                if "Intel Corp." in line or "RealSense" in line:
                    return True
            return False
        except Exception as e:
            print(f"[ERROR] USB check failed: {e}")
            return False


    def create_client_interface_future(self):
        future = Future()
        return future

    def set_button_state(self, button, enabled, color):
        button.setEnabled(enabled)
        if enabled:
            button.setStyleSheet(f"background-color: {color}; color: white;")
        else:
            button.setStyleSheet("background-color: gray; color: white;")


    def check_hardware_status(self):
        # Vérifie l'arduino
        arduino_ok = self.check_arduino_connection()
        cam_ok = self.check_camera_connection()

        if "rotate_table" in self.control_buttons:
            self.set_button_state(self.control_buttons["rotate_table"]["start"], arduino_ok, "green" if arduino_ok else "red")

        if "camera" in self.control_buttons:
            self.set_button_state(self.control_buttons["camera"]["start"], cam_ok, "green" if cam_ok else "red")
            self.set_button_state(self.control_buttons["camera"]["stop"], cam_ok, "green" if cam_ok else "red")
            self.set_button_state(self.control_buttons["capture_photo"]["start"], False, "red")
            self.set_button_state(self.control_buttons["capture_photo"]["stop"], False, "red")
            

    # -- PROCESS LAUNCHERS --
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
        if not self.check_arduino_connection():
            print("[WARN] Arduino not detected.")
            return

        try:
            self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            print("[INFO] Connected to Arduino.")
            self.table_window = TableConfigWindow(self.serial)
            self.table_window.show()
        except Exception as e:
            print(f"[ERROR] Couldn't open serial port: {e}")


    def stop_table(self):
        self.stop_process("rotate_table", match="python3 rotate_table.py")


    def launch_camera(self):
        cam_ok = self.check_camera_connection()
        if cam_ok:

            self.start_process("camera", [
                "gnome-terminal", "--", "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"
            ])
            print("[INFO] Camera launch command sent.")
            self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
            self.timer.start(100)

            self.set_button_state(self.control_buttons["capture_photo"]["start"], cam_ok, "green" if cam_ok else "red")
            self.set_button_state(self.control_buttons["capture_photo"]["stop"], cam_ok, "green" if cam_ok else "red")            

    def stop_camera(self):
        self.stop_process("camera", match="ros2 launch realsense2_camera rs_launch.py")
        self.timer.stop()
        self.image_label.setText("📷 Camera stopped")
        self.image_data = None
        self.set_button_state(self.control_buttons["capture_photo"]["start"], False, "red")
        self.set_button_state(self.control_buttons["capture_photo"]["stop"], False, "red")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data = cv_image
        except Exception as e:
            print("[ERROR] Failed to convert image:", e)

    def update_image(self):
        if self.image_data is not None:
            rgb = cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_img).scaled(self.image_label.width(), self.image_label.height())
            self.image_label.setPixmap(pixmap)

    def capture_photo(self):
        if self.image_data is not None:
            output_dir = os.path.expanduser("~/Documents/img")
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(output_dir, f"capture_{timestamp}.png")
            cv2.imwrite(filepath, self.image_data)
            print(f"[✓] Photo saved at: {filepath}")
        else:
            print("[!] No image available — is the camera running?")


from PyQt5.QtWidgets import QLineEdit, QGroupBox

class TableConfigWindow(QWidget):
    def __init__(self, serial_port):
        super().__init__()
        self.setWindowTitle("Configure Rotating Table")
        self.setGeometry(200, 200, 400, 200)
        self.serial = serial_port

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        self.status = QLabel("Ready")
        layout.addWidget(self.status)

        rotate_btn = QPushButton("Start Rotation")
        stop_btn = QPushButton("Stop Rotation")
        rotate_btn.clicked.connect(self.start_rotation)
        stop_btn.clicked.connect(self.stop_rotation)

        degree_box = QGroupBox("Move by Degree")
        degree_layout = QHBoxLayout()
        self.degree_input = QLineEdit()
        self.degree_input.setPlaceholderText("e.g. 90")
        degree_btn = QPushButton("Send")
        degree_btn.clicked.connect(self.move_by_degree)
        degree_layout.addWidget(self.degree_input)
        degree_layout.addWidget(degree_btn)
        degree_box.setLayout(degree_layout)

        layout.addWidget(rotate_btn)
        layout.addWidget(stop_btn)
        layout.addWidget(degree_box)
        layout.addWidget(self.status)

        self.setLayout(layout)

    def start_rotation(self):
        self.send_command("rotate_table")

    def stop_rotation(self):
        self.send_command("stop_table")

    def move_by_degree(self):
        deg = self.degree_input.text()
        if deg.isdigit():
            self.send_command(f"move_table_{deg}")
        else:
            self.status.setText("❌ Invalid input")

    def send_command(self, command):
        try:
            self.serial.write((command + '\n').encode())
            self.status.setText(f"✅ Sent: {command}")
        except Exception as e:
            self.status.setText(f"❌ Error: {e}")

    def closeEvent(self, event):
        try:
            self.serial.close()
            print("[INFO] Serial connection closed.")
        except:
            pass
        event.accept()



def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ROS2GuiApp()
    gui.show()

    # Start ROS 2 spinning in a background thread
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
