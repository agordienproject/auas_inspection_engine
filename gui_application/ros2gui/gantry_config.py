import os
import sys
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout, QLineEdit, QPushButton, QComboBox, QCheckBox
from . import utils

# Add the parent directory to sys.path to access cri_lib
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

class GantryConfigWindow(QWidget):
    PROGRAMS_FOLDER = "/home/agordien/Documents/gantry_programs"
    PROGRAMS_REMOTE_PATH = r"C:\\iRC-igusRobotControl-V14\Data\\Programs\\"
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuration Gantry")
        self.setGeometry(250, 250, 400, 300)
        self.controller = None
        self.connected = False
        self.program_loaded = False
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.status = QLabel("Status: Disconnected")
        layout.addWidget(self.status)
        ip_port_layout = QHBoxLayout()
        self.ip_input = QLineEdit("192.168.3.11")
        self.ip_input.setPlaceholderText("Adresse IP")
        self.port_input = QLineEdit("3920")
        self.port_input.setPlaceholderText("Port")
        ip_port_layout.addWidget(QLabel("IP:"))
        ip_port_layout.addWidget(self.ip_input)
        ip_port_layout.addWidget(QLabel("Port:"))
        ip_port_layout.addWidget(self.port_input)
        layout.addLayout(ip_port_layout)
        self.connect_btn = QPushButton("Connexion")
        self.connect_btn.clicked.connect(self.try_connect)
        layout.addWidget(self.connect_btn)

        # Add Disconnect button
        self.disconnect_btn = QPushButton("Déconnexion")
        self.disconnect_btn.clicked.connect(self.handle_disconnect)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn)

        self.motor_btn = QPushButton("disabled")
        self.motor_btn.setStyleSheet("background-color: red; color: white;")
        self.motor_btn.setEnabled(False)
        self.motor_btn.clicked.connect(self.toggle_motors)
        layout.addWidget(self.motor_btn)
        self.program_combo = QComboBox()
        self.program_files = []
        if os.path.isdir(self.PROGRAMS_FOLDER):
            self.program_files = [f for f in os.listdir(self.PROGRAMS_FOLDER) if f.endswith('.xml')]
            self.program_combo.addItems(self.program_files)
        else:
            self.program_combo.addItem("No programs found")
        layout.addWidget(QLabel("Choisir un programme:"))
        layout.addWidget(self.program_combo)
        # Add custom program checkbox
        self.custom_program_checkbox = QCheckBox("Choisir un programme personnalisé")
        self.custom_program_checkbox.stateChanged.connect(self.toggle_custom_program)
        layout.addWidget(self.custom_program_checkbox)
        self.program_input = QLineEdit(self.PROGRAMS_REMOTE_PATH)
        self.program_input.setPlaceholderText("Chemin du programme XML")
        self.program_input.setReadOnly(False)
        layout.addWidget(self.program_input)
        self.load_btn = QPushButton("Charger un programme")
        self.load_btn.clicked.connect(self.load_program)
        layout.addWidget(self.load_btn)
        self.start_btn = QPushButton("Lancer le programme")
        self.start_btn.clicked.connect(self.start_program)
        layout.addWidget(self.start_btn)
        self.resume_btn = QPushButton("Pauser le programme")
        self.resume_btn.clicked.connect(self.pause_program)
        layout.addWidget(self.resume_btn)
        self.stop_btn = QPushButton("Arrêter le programme")
        self.stop_btn.clicked.connect(self.stop_program)
        layout.addWidget(self.stop_btn)
        self.setLayout(layout)
        self.set_connected_state(False)
        self.try_connect(auto=True)
        self.program_combo.currentIndexChanged.connect(self.update_program_input)

    def toggle_custom_program(self, state):
        if self.custom_program_checkbox.isChecked():
            self.program_input.setReadOnly(False)
        else:
            self.program_input.setReadOnly(True)
            self.update_program_input()

    def update_program_input(self):
        if self.custom_program_checkbox.isChecked():
            # Do not overwrite if custom mode is enabled
            return
        idx = self.program_combo.currentIndex()
        if idx >= 0 and self.program_files:
            filename = self.program_files[idx]
            # Show only one backslash in the display
            display_path = (self.PROGRAMS_REMOTE_PATH + filename).replace('\\\\', '\\')
            self.program_input.setText(display_path)
        else:
            self.program_input.setText(self.PROGRAMS_REMOTE_PATH)

    def set_connected_state(self, connected):
        self.connected = connected
        self.load_btn.setEnabled(connected)
        self.start_btn.setEnabled(connected)
        self.stop_btn.setEnabled(connected)
        self.motor_btn.setEnabled(connected)
        self.disconnect_btn.setEnabled(connected)
        if connected:
            self.status.setText("✅ Connecté au Gantry")
            self.status.setStyleSheet("color: green;")
            self.connect_btn.setEnabled(False)
            self.motor_enabled = False
            self.update_motor_button()
        else:
            self.status.setText("❌ Non connecté au Gantry")
            self.status.setStyleSheet("color: red;")
            self.connect_btn.setEnabled(True)
            self.motor_btn.setText("disabled")
            self.motor_btn.setStyleSheet("background-color: red; color: white;")
            self.motor_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(False)

    def try_connect(self, auto=False):
        ip = self.ip_input.text().strip()
        try:
            port = int(self.port_input.text().strip())
        except ValueError:
            self.status.setText("❌ Port invalide")
            self.status.setStyleSheet("color: red;")
            return
        if self.controller:
            try:
                self.controller.close()
            except Exception:
                pass
        import sys
        import os
        sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
        import cri_lib
        self.controller = cri_lib.CRIController()
        self.connected = self.controller.connect(ip, port)
        self.set_connected_state(self.connected)
        if not self.connected and not auto:
            self.status.setText("❌ Connexion échouée")
            self.status.setStyleSheet("color: red;")
    
    def update_motor_button(self):
        if getattr(self, "motor_enabled", False):
            self.motor_btn.setText("enabled")
            self.motor_btn.setStyleSheet("background-color: green; color: white;")
        else:
            self.motor_btn.setText("disabled")
            self.motor_btn.setStyleSheet("background-color: red; color: white;")

    def toggle_motors(self):
        if not self.connected:
            return
        if getattr(self, "motor_enabled", False):
            ok = self.controller.disable()
            if ok:
                self.motor_enabled = False
        else:
            ok = self.controller.enable()
            if ok:
                self.motor_enabled = True
        self.update_motor_button()

    def load_program(self):
        if not self.connected:
            self.status.setText("❌ Non connecté au Gantry")
            return
        self.controller.set_active_control(True)
        self.controller.wait_for_kinematics_ready(10)
        self.controller.set_override(50.0)
        idx = self.program_combo.currentIndex()
        if idx < 0 or not self.program_files:
            self.status.setText("❌ Aucun programme sélectionné")
            return
        program_name = self.program_files[idx]
        local_path = os.path.join(self.PROGRAMS_FOLDER, program_name)
        print("local_path:", local_path)
        # Upload the file first
        self.controller.upload_file(local_path, "Programs")
        
        # Now load by filename only
        if self.controller.load_programm(program_name):
            self.status.setText(f"✅ Programme chargé: {program_name}")
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

    def pause_program(self):
        if not self.connected:
            self.status.setText("❌ Non connecté au Gantry")
            return
        try:
            if self.controller.pause_programm():
                self.status.setText("✅ Programme pausé")
            else:
                self.status.setText("❌ Échec de la pause")
        except Exception as e:
            self.status.setText(f"❌ Erreur: {e}")
    
    def stop_program(self):
        if not self.connected:
            self.status.setText("❌ Non connecté au Gantry")
            return
        if self.controller.stop_programm():
            self.status.setText("✅ Programme arrêté")
        else:
            self.status.setText("❌ Échec de l'arrêt")

    def handle_disconnect(self):
        if self.controller and self.connected:
            try:
                self.controller.disable()
                self.controller.close()
            except Exception:
                pass
        self.set_connected_state(False)

    def closeEvent(self, event):
        self.handle_disconnect()
        event.accept()
