import os
import datetime
import cv2
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image

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
