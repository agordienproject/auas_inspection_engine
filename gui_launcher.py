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


# GANTRY SYSTEM
import cri_lib


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
        self.processes = {}

        self.setup_ui()

    #----------- INIT -----------#
    def setup_ui(self):
        layout = QVBoxLayout()
        self.control_buttons = {}

        layout.addLayout(self.make_button_pair("Talker", self.launch_talker, self.stop_talker))
        layout.addLayout(self.make_button_pair("Listener", self.launch_listener, self.stop_listener))
        layout.addLayout(self.make_config_button("Configure Rotating Table", self.open_table_config, key="rotate_table"))
        layout.addLayout(self.make_config_button("Configurer la caméra", self.open_camera_config, key="camera_config"))
        layout.addLayout(self.make_config_button("Configurer le Gantry", self.open_gantry_config, key="gantry_config"))  # Ajout Gantry

        self.setLayout(layout)
        QTimer.singleShot(1000, self.check_hardware_status)

    def make_single_button(self, label, click_func, key=None):
        box = QHBoxLayout()
        box.addWidget(QLabel(label))
        button = QPushButton(label)
        button.clicked.connect(click_func)
        box.addWidget(button)
        if key:
            self.control_buttons[key] = {"start": button}
        return box

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

    def check_gantry_connection(self):
        try:
            controller = cri_lib.CRIController()
            connected = controller.connect("127.0.0.1", 3921)
            controller.close()
            return connected
        except Exception as e:
            print(f"[ERROR] Gantry CRI connection failed: {e}")
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
        arduino_ok = self.check_arduino_connection()
        cam_ok = self.check_camera_connection()
        gantry_ok = self.check_gantry_connection()

        if "rotate_table" in self.control_buttons:
            self.set_button_state(self.control_buttons["rotate_table"]["start"], arduino_ok, "green" if arduino_ok else "red")

        if "camera_config" in self.control_buttons:
            self.set_button_state(self.control_buttons["camera_config"]["start"], cam_ok, "green" if cam_ok else "red")

        if "gantry_config" in self.control_buttons:
            self.set_button_state(self.control_buttons["gantry_config"]["start"], gantry_ok, "green" if gantry_ok else "red")
    
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

    def open_camera_config(self):
        if not self.check_camera_connection():
            print("[WARN] Camera not detected.")
            return

        self.camera_window = CameraConfigWindow(self, self.bridge)
        self.camera_window.show()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data = cv_image
        except Exception as e:
            print(f"[ERROR] Failed to convert image: {e}")
    
    def open_gantry_config(self):
        if not self.check_gantry_connection():
            print("[WARN] Gantry CRI not detected.")
            return

        self.gantry_window = GantryConfigWindow()
        self.gantry_window.show()

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

class CameraConfigWindow(QWidget):
    def __init__(self, parent, bridge):
        super().__init__()
        self.setWindowTitle("Configuration Caméra")
        self.setGeometry(200, 200, 700, 600)
        self.parent = parent
        self.bridge = bridge
        self.image_data = None
        self.recording = False
        self.video_writer = None

        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)

    def init_ui(self):
        layout = QVBoxLayout()

        self.image_label = QLabel("📷 Camera not started")
        self.image_label.setFixedSize(640, 480)
        layout.addWidget(self.image_label)

        btn_layout = QHBoxLayout()

        self.launch_camera_btn = QPushButton("Démarrer la caméra")
        self.launch_camera_btn.clicked.connect(self.launch_camera)
        btn_layout.addWidget(self.launch_camera_btn)

        self.stop_camera_btn = QPushButton("Arrêter la caméra")
        self.stop_camera_btn.clicked.connect(self.stop_camera)
        self.stop_camera_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_camera_btn)

        self.capture_btn = QPushButton("Capture d'écran")
        self.capture_btn.clicked.connect(self.capture_photo)
        self.capture_btn.setEnabled(False)
        btn_layout.addWidget(self.capture_btn)

        self.start_video_btn = QPushButton("Démarrer vidéo")
        self.start_video_btn.clicked.connect(self.start_recording)
        self.start_video_btn.setEnabled(False)
        btn_layout.addWidget(self.start_video_btn)

        self.stop_video_btn = QPushButton("Arrêter vidéo")
        self.stop_video_btn.clicked.connect(self.stop_recording)
        self.stop_video_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_video_btn)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def launch_camera(self):
        if not hasattr(self.parent, "camera_subscription"):
            self.parent.start_process("camera", [
                "gnome-terminal", "--", "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"
            ])
            print("[INFO] Camera launch command sent.")
            self.parent.create_subscription(
                Image, '/camera/camera/color/image_raw', self.parent.image_callback, 10
            )
            self.parent.timer.start(100)
            self.launch_camera_btn.setEnabled(False)
            self.stop_camera_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)
            self.start_video_btn.setEnabled(True)

    def stop_camera(self):
        self.parent.stop_process("camera", match="ros2 launch realsense2_camera rs_launch.py")
        self.parent.timer.stop()
        self.image_label.setText("📷 Camera stopped")
        self.image_data = None
        self.launch_camera_btn.setEnabled(True)
        self.stop_camera_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        self.start_video_btn.setEnabled(False)
        self.stop_video_btn.setEnabled(False)

    def update_image(self):
        if self.parent.image_data is not None:
            rgb = cv2.cvtColor(self.parent.image_data, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_img).scaled(self.image_label.width(), self.image_label.height())
            self.image_label.setPixmap(pixmap)
            self.image_data = self.parent.image_data.copy()

            if self.recording and self.video_writer is not None:
                self.video_writer.write(self.parent.image_data)

    def capture_photo(self):
        if self.image_data is not None:
            output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "img")
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(output_dir, f"capture_{timestamp}.png")
            cv2.imwrite(filepath, self.image_data)
            print(f"[✓] Photo saved at: {filepath}")
        else:
            print("[!] No image available — is the camera running?")

    def start_recording(self):
        if self.image_data is not None and not self.recording:
            videos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "img", "videos")
            os.makedirs(videos_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(videos_dir, f"video_{timestamp}.avi")
            h, w, _ = self.image_data.shape
            self.video_writer = cv2.VideoWriter(filepath, cv2.VideoWriter_fourcc(*'XVID'), 20, (w, h))
            self.recording = True
            self.start_video_btn.setEnabled(False)
            self.stop_video_btn.setEnabled(True)
            print(f"[INFO] Recording started: {filepath}")
        else:
            print("[!] No image available or already recording.")

    def stop_recording(self):
        if self.recording and self.video_writer is not None:
            self.recording = False
            self.video_writer.release()
            self.video_writer = None
            self.start_video_btn.setEnabled(True)
            self.stop_video_btn.setEnabled(False)
            print("[INFO] Recording stopped.")

    def closeEvent(self, event):
        self.timer.stop()
        if self.recording and self.video_writer is not None:
            self.video_writer.release()
        event.accept()

class GantryConfigWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuration Gantry")
        self.setGeometry(250, 250, 400, 250)
        self.controller = cri_lib.CRIController()
        self.connected = self.controller.connect("127.0.0.1", 3921)
        self.program_loaded = False

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.status = QLabel("Status: Disconnected" if not self.connected else "Status: Connected")
        layout.addWidget(self.status)

        self.program_input = QLineEdit()
        self.program_input.setPlaceholderText("Chemin du programme XML")
        layout.addWidget(self.program_input)

        load_btn = QPushButton("Charger un programme")
        load_btn.clicked.connect(self.load_program)
        layout.addWidget(load_btn)

        start_btn = QPushButton("Lancer le programme")
        start_btn.clicked.connect(self.start_program)
        layout.addWidget(start_btn)

        stop_btn = QPushButton("Arrêter le programme")
        stop_btn.clicked.connect(self.stop_program)
        layout.addWidget(stop_btn)

        self.setLayout(layout)

    def load_program(self):
        path = self.program_input.text()
        if not path:
            self.status.setText("❌ Chemin du programme manquant")
            return
        if not self.connected:
            self.status.setText("❌ Non connecté au Gantry")
            return
        if self.controller.load_programm(path):
            self.status.setText(f"✅ Programme chargé: {path}")
            self.program_loaded = True
        else:
            self.status.setText("❌ Échec du chargement")

    def start_program(self):
        if not self.connected or not self.program_loaded:
            self.status.setText("❌ Charger un programme d'abord")
            return
        if self.controller.start_programm():
            self.status.setText("✅ Programme lancé")
        else:
            self.status.setText("❌ Échec du lancement")

    def stop_program(self):
        if not self.connected:
            self.status.setText("❌ Non connecté au Gantry")
            return
        if self.controller.stop_programm():
            self.status.setText("✅ Programme arrêté")
        else:
            self.status.setText("❌ Échec de l'arrêt")

    def closeEvent(self, event):
        try:
            if self.connected:
                self.controller.disable()
                self.controller.close()
        except Exception:
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
