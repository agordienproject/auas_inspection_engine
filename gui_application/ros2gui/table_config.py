from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QGroupBox, QHBoxLayout, QLineEdit

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
