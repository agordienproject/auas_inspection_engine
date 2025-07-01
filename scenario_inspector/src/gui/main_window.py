"""
Main window for Scenario Inspector application
"""
import os
import logging
from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QComboBox, QTextEdit, QProgressBar,
                             QGroupBox, QGridLayout, QMessageBox, QFileDialog,
                             QTableWidget, QTableWidgetItem, QTabWidget, QSplitter)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QTextCursor

from config.config_manager import ConfigManager
from database.models import User
from inspector.scenario_engine import ScenarioEngine

class ScenarioExecutionThread(QThread):
    """Thread for executing scenarios without blocking the UI"""
    
    progress_updated = pyqtSignal(str)  # Progress message
    execution_finished = pyqtSignal(bool)  # Success status
    
    def __init__(self, scenario_engine, scenario_file):
        super().__init__()
        self.scenario_engine = scenario_engine
        self.scenario_file = scenario_file
        
    def run(self):
        """Execute scenario in thread"""
        try:
            # Load scenario
            if not self.scenario_engine.load_scenario_from_file(self.scenario_file):
                self.execution_finished.emit(False)
                return
            
            self.progress_updated.emit("Scenario loaded successfully")
            
            # Execute scenario
            success = self.scenario_engine.execute_scenario()
            self.execution_finished.emit(success)
            
        except Exception as e:
            logging.getLogger(__name__).error(f"Execution thread error: {e}")
            self.execution_finished.emit(False)

class ScenarioInspectorWindow(QMainWindow):
    """Main window for the Scenario Inspector application"""
    
    def __init__(self, user: User):
        super().__init__()
        self.user = user
        self.logger = logging.getLogger(__name__)
        self.config_manager = ConfigManager()
        self.scenario_engine = ScenarioEngine(user)
        
        # UI state
        self.execution_thread = None
        self.is_executing = False
        
        self.setup_ui()
        self.setup_menu()
        self.load_available_programs()
        
        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(5000)  # Update every 5 seconds

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.label = QLabel("Welcome to the Scenario Inspector")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)

        self.start_inspection_button = QPushButton("Start Inspection")
        self.start_inspection_button.clicked.connect(self.start_inspection)
        self.layout.addWidget(self.start_inspection_button)

        self.exit_button = QPushButton("Exit")
        self.exit_button.clicked.connect(self.close)
        self.layout.addWidget(self.exit_button)

    def start_inspection(self):
        QMessageBox.information(self, "Info", "Starting the inspection process...")