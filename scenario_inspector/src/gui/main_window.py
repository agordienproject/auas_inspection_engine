"""
Main window for AUAS Inspection Engine
"""
import os
import logging
import yaml
import threading
from datetime import datetime
from typing import Dict, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QComboBox, QTextEdit, QProgressBar,
                             QGroupBox, QGridLayout, QMessageBox, QFileDialog, 
                             QLineEdit, QListWidget, QTabWidget, QFrame, QSpacerItem,
                             QSizePolicy, QFormLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPalette

from config.config_manager import ConfigManager
from database.models import User
from systems.system_manager import SystemManager


class ProgramExecutionThread(QThread):
    """Thread for executing inspection programs without blocking the UI"""
    
    progress_updated = pyqtSignal(str)  # Progress message
    execution_finished = pyqtSignal(bool)  # Success status
    step_completed = pyqtSignal(str)  # Step name
    progress_percentage = pyqtSignal(int)  # Progress percentage for progress bar
    
    def __init__(self, program_data, piece_info, config_manager):
        super().__init__()
        self.program_data = program_data
        self.piece_info = piece_info
        self.config_manager = config_manager
        self.system_manager = SystemManager(config_manager)
        self.logger = logging.getLogger(__name__)
        
    def run(self):
        """Execute program in thread"""
        try:
            self.progress_updated.emit("Starting program execution...")
            
            # Create inspection folder before starting execution
            program_data_for_folder = {
                'name': self.program_data.get('program', {}).get('name', 'unknown_program'),
                'piece_info': self.piece_info
            }
            
            inspection_folder = self.system_manager.create_inspection_folder(program_data_for_folder)
            self.progress_updated.emit(f"Created inspection folder: {os.path.basename(inspection_folder)}")
            
            # Get program stages
            stages = self.program_data.get('program', {}).get('stages', [])
            total_steps = sum(len(stage.get('steps', [])) for stage in stages)
            completed_steps = 0
            
            # Initialize progress
            self.progress_percentage.emit(0)
            
            for stage in stages:
                stage_name = stage.get('name', f"Stage {stage.get('stage', 'Unknown')}")
                together = stage.get('together', False)
                
                self.progress_updated.emit(f"Executing stage: {stage_name} {'(parallel)' if together else '(sequential)'}")
                
                # Execute steps in stage
                steps = stage.get('steps', [])
                
                if together and len(steps) > 1:
                    # Execute steps in parallel
                    completed_steps += self._execute_steps_parallel(steps, stage_name, completed_steps, total_steps)
                else:
                    # Execute steps sequentially 
                    completed_steps += self._execute_steps_sequential(steps, completed_steps, total_steps)
                
                # Update progress
                progress_percent = int((completed_steps / total_steps) * 100) if total_steps > 0 else 100
                self.progress_percentage.emit(progress_percent)
                self.progress_updated.emit(f"Stage '{stage_name}' completed. Progress: {progress_percent}% ({completed_steps}/{total_steps})")
            
            self.progress_updated.emit("Program execution completed successfully!")
            self.progress_percentage.emit(100)
            self.execution_finished.emit(True)
            
        except Exception as e:
            self.logger.error(f"Program execution error: {e}")
            self.progress_updated.emit(f"Execution failed: {str(e)}")
            self.execution_finished.emit(False)
        finally:
            # Cleanup systems
            self.system_manager.shutdown_all_systems()
    
    def _execute_steps_sequential(self, steps, current_completed, total_steps):
        """Execute steps one after another"""
        completed = 0
        for step in steps:
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            
            self.progress_updated.emit(f"Executing step: {step_name} on system: {system}")
            
            try:
                result = self.system_manager.execute_step(step)
                self.step_completed.emit(step_name)
                completed += 1
                
                # Update progress bar after each step
                new_completed = current_completed + completed
                progress_percent = int((new_completed / total_steps) * 100) if total_steps > 0 else 0
                self.progress_percentage.emit(progress_percent)
                
            except Exception as e:
                self.progress_updated.emit(f"Step {step_name} failed: {str(e)}")
                self.execution_finished.emit(False)
                raise
                
        return completed
    
    def _execute_steps_parallel(self, steps, stage_name, current_completed, total_steps):
        """Execute steps in parallel using threading"""
        completed = 0
        
        def execute_single_step(step):
            """Execute a single step - to be run in thread"""
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            
            try:
                self.progress_updated.emit(f"[Parallel] Starting step: {step_name} on system: {system}")
                result = self.system_manager.execute_step(step)
                self.step_completed.emit(step_name)
                self.progress_updated.emit(f"[Parallel] Completed step: {step_name}")
                return step_name, True, None
                
            except Exception as e:
                error_msg = f"Step {step_name} failed: {str(e)}"
                self.progress_updated.emit(f"[Parallel] {error_msg}")
                return step_name, False, error_msg
        
        # Execute steps in parallel using ThreadPoolExecutor
        self.progress_updated.emit(f"Starting {len(steps)} parallel steps in stage: {stage_name}")
        
        with ThreadPoolExecutor(max_workers=len(steps)) as executor:
            # Submit all steps for execution
            future_to_step = {executor.submit(execute_single_step, step): step for step in steps}
            
            # Wait for all steps to complete
            for future in as_completed(future_to_step):
                step = future_to_step[future]
                try:
                    step_name, success, error_msg = future.result()
                    if success:
                        completed += 1
                        # Update progress bar as each parallel step completes
                        new_completed = current_completed + completed
                        progress_percent = int((new_completed / total_steps) * 100) if total_steps > 0 else 0
                        self.progress_percentage.emit(progress_percent)
                    else:
                        # If any step fails, we should stop execution
                        self.progress_updated.emit(f"Parallel execution failed: {error_msg}")
                        self.execution_finished.emit(False)
                        raise Exception(error_msg)
                        
                except Exception as e:
                    step_name = step.get('name', 'Unknown')
                    self.progress_updated.emit(f"Exception in parallel step {step_name}: {str(e)}")
                    raise
        
        self.progress_updated.emit(f"All {len(steps)} parallel steps completed successfully")
        return completed


class InspectionMainWindow(QMainWindow):
    """Main window for the AUAS Inspection Engine"""
    
    def __init__(self, user: User):
        super().__init__()
        self.user = user
        self.config_manager = ConfigManager()
        self.system_manager = SystemManager(self.config_manager)
        self.logger = logging.getLogger(__name__)
        self.execution_thread = None
        self.current_program = None
        self.piece_info = {}
        
        self.setup_ui()
        self.load_available_programs()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Set window properties
        gui_config = self.config_manager.get_gui_config()
        self.setWindowTitle(gui_config.get('window_title', 'AUAS Inspection Engine'))
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Header
        self.create_header(main_layout)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # Create tabs
        self.create_piece_info_tab()
        self.create_program_tab()
        self.create_execution_tab()
        self.create_status_tab()
        
        # Status bar
        self.statusBar().showMessage(f"Logged in as: {self.user.pseudo}")
        
    def create_header(self, layout):
        """Create the header section"""
        header_frame = QFrame()
        header_frame.setFrameStyle(QFrame.StyledPanel)
        header_frame.setMaximumHeight(80)
        
        header_layout = QHBoxLayout(header_frame)
        
        # Title
        title_label = QLabel("AUAS Inspection Engine")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        header_layout.addWidget(title_label)
        
        # Spacer
        header_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        # User info
        user_label = QLabel(f"User: {self.user.pseudo}")
        header_layout.addWidget(user_label)
        
        # Logout button
        logout_btn = QPushButton("Logout")
        logout_btn.clicked.connect(self.logout)
        header_layout.addWidget(logout_btn)
        
        layout.addWidget(header_frame)
        
    def create_piece_info_tab(self):
        """Create the piece information tab"""
        piece_widget = QWidget()
        layout = QVBoxLayout(piece_widget)
        
        # Piece information group
        piece_group = QGroupBox("Piece Information")
        piece_layout = QFormLayout(piece_group)
        
        self.piece_name_edit = QLineEdit()
        self.piece_ref_edit = QLineEdit()
        
        piece_layout.addRow("Piece Name:", self.piece_name_edit)
        piece_layout.addRow("Reference:", self.piece_ref_edit)
        
        # Save piece info button
        save_piece_btn = QPushButton("Save Piece Information")
        save_piece_btn.clicked.connect(self.save_piece_info)
        piece_layout.addWidget(save_piece_btn)
        
        layout.addWidget(piece_group)
        layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.tab_widget.addTab(piece_widget, "Piece Info")
        
    def create_program_tab(self):
        """Create the program selection tab"""
        program_widget = QWidget()
        layout = QVBoxLayout(program_widget)
        
        # Available programs group
        programs_group = QGroupBox("Available Programs")
        programs_layout = QVBoxLayout(programs_group)
        
        self.programs_list = QListWidget()
        programs_layout.addWidget(self.programs_list)
        
        # Program actions
        actions_layout = QHBoxLayout()
        
        select_program_btn = QPushButton("Select Program")
        select_program_btn.clicked.connect(self.select_program)
        actions_layout.addWidget(select_program_btn)
        
        upload_program_btn = QPushButton("Upload Program")
        upload_program_btn.clicked.connect(self.upload_program)
        actions_layout.addWidget(upload_program_btn)
        
        programs_layout.addLayout(actions_layout)
        layout.addWidget(programs_group)
        
        # Selected program info
        self.selected_program_group = QGroupBox("Selected Program")
        selected_layout = QVBoxLayout(self.selected_program_group)
        
        self.program_info_text = QTextEdit()
        self.program_info_text.setReadOnly(True)
        self.program_info_text.setMaximumHeight(200)
        selected_layout.addWidget(self.program_info_text)
        
        layout.addWidget(self.selected_program_group)
        
        self.tab_widget.addTab(program_widget, "Programs")
        
    def create_execution_tab(self):
        """Create the execution control tab"""
        exec_widget = QWidget()
        layout = QVBoxLayout(exec_widget)
        
        # Execution controls
        controls_group = QGroupBox("Execution Controls")
        controls_layout = QVBoxLayout(controls_group)
        
        # Start execution button
        self.start_execution_btn = QPushButton("Start Inspection")
        self.start_execution_btn.clicked.connect(self.start_execution)
        self.start_execution_btn.setEnabled(False)
        controls_layout.addWidget(self.start_execution_btn)
        
        # Stop execution button
        self.stop_execution_btn = QPushButton("Stop Inspection")
        self.stop_execution_btn.clicked.connect(self.stop_execution)
        self.stop_execution_btn.setEnabled(False)
        controls_layout.addWidget(self.stop_execution_btn)
        
        layout.addWidget(controls_group)
        
        # Progress group
        progress_group = QGroupBox("Execution Progress")
        progress_layout = QVBoxLayout(progress_group)
        
        self.progress_bar = QProgressBar()
        progress_layout.addWidget(self.progress_bar)
        
        self.progress_text = QTextEdit()
        self.progress_text.setReadOnly(True)
        progress_layout.addWidget(self.progress_text)
        
        layout.addWidget(progress_group)
        
        self.tab_widget.addTab(exec_widget, "Execution")
        
    def create_status_tab(self):
        """Create the system status tab"""
        status_widget = QWidget()
        layout = QVBoxLayout(status_widget)
        
        # Control buttons
        buttons_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("🔄 Refresh Status")
        refresh_btn.clicked.connect(self.refresh_system_status)
        refresh_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; }")
        buttons_layout.addWidget(refresh_btn)
        
        test_all_btn = QPushButton("🔍 Test All Connections")
        test_all_btn.clicked.connect(self.test_all_connections)
        test_all_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #2196F3; color: white; }")
        buttons_layout.addWidget(test_all_btn)
        
        layout.addLayout(buttons_layout)
        
        # System status group with scrollable area
        from PyQt5.QtWidgets import QScrollArea
        
        status_group = QGroupBox("System Status")
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        self.status_layout = QGridLayout(scroll_widget)
        
        # Set fixed column widths to prevent stretching
        self.status_layout.setColumnMinimumWidth(0, 120)  # System name
        self.status_layout.setColumnMinimumWidth(1, 60)   # Status
        self.status_layout.setColumnMinimumWidth(2, 200)  # Connection info
        self.status_layout.setColumnMinimumWidth(3, 300)  # Details
        
        # Configure scroll area
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setMaximumHeight(300)  # Limit height to prevent window stretching
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        status_group_layout = QVBoxLayout(status_group)
        status_group_layout.addWidget(scroll_area)
        
        # Initialize status display
        self._initialize_status_display()
        
        layout.addWidget(status_group)
        
        # Status log
        log_group = QGroupBox("Connection Log")
        log_layout = QVBoxLayout(log_group)
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(150)
        log_layout.addWidget(self.status_log)
        
        layout.addWidget(log_group)
        
        layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.tab_widget.addTab(status_widget, "System Status")
        
    def _initialize_status_display(self):
        """Initialize the status display grid"""
        # Clear existing layout
        for i in reversed(range(self.status_layout.count())): 
            self.status_layout.itemAt(i).widget().setParent(None)
        
        # Header row
        self.status_layout.addWidget(QLabel("System"), 0, 0)
        self.status_layout.addWidget(QLabel("Status"), 0, 1)
        self.status_layout.addWidget(QLabel("Connection Info"), 0, 2)
        self.status_layout.addWidget(QLabel("Details"), 0, 3)
        
        self.status_labels = {}
        self.status_info_labels = {}
        self.status_detail_labels = {}
        
        # Get systems from config
        systems_config = self.config_manager.get_systems_config()
        
        row = 1
        for system_name in systems_config.keys():
            # System name
            system_label = QLabel(f"{system_name}:")
            system_label.setMinimumWidth(100)
            system_label.setMaximumWidth(120)
            self.status_layout.addWidget(system_label, row, 0)
            
            # Status indicator
            status_label = QLabel("⏳")
            status_label.setStyleSheet("color: orange; font-size: 16px;")
            status_label.setToolTip("Status unknown")
            status_label.setMinimumWidth(50)
            status_label.setMaximumWidth(60)
            self.status_layout.addWidget(status_label, row, 1)
            self.status_labels[system_name] = status_label
            
            # Connection info
            info_label = QLabel("Not tested")
            info_label.setWordWrap(True)
            info_label.setMinimumWidth(150)
            info_label.setMaximumWidth(200)
            self.status_layout.addWidget(info_label, row, 2)
            self.status_info_labels[system_name] = info_label
            
            # Details
            detail_label = QLabel("-")
            detail_label.setWordWrap(True)
            detail_label.setMinimumWidth(200)
            detail_label.setMaximumWidth(300)
            detail_label.setAlignment(Qt.AlignTop)
            self.status_layout.addWidget(detail_label, row, 3)
            self.status_detail_labels[system_name] = detail_label
            
            row += 1
        
    def refresh_system_status(self):
        """Refresh system status display"""
        self.log_status_message("Refreshing system status...")
        
        try:
            systems_status = self.system_manager.get_all_systems_status()
            
            for system_name, status_info in systems_status.items():
                if system_name in self.status_labels:
                    self._update_system_status_display(system_name, status_info)
                    
            self.log_status_message("✓ System status refresh completed")
            
        except Exception as e:
            self.logger.error(f"Error refreshing system status: {e}")
            self.log_status_message(f"❌ Error refreshing status: {e}")
    
    def test_all_connections(self):
        """Test all system connections individually"""
        self.log_status_message("🔍 Testing all system connections...")
        
        try:
            systems_config = self.config_manager.get_systems_config()
            
            for system_name in systems_config.keys():
                self.log_status_message(f"Testing {system_name}...")
                
                # Update status to testing
                if system_name in self.status_labels:
                    self.status_labels[system_name].setText("🔄")
                    self.status_labels[system_name].setStyleSheet("color: blue; font-size: 16px;")
                    self.status_labels[system_name].setToolTip("Testing connection...")
                    self.status_info_labels[system_name].setText("Testing connection...")
                
                # Test the connection
                status_info = self.system_manager.get_system_status(system_name)
                self._update_system_status_display(system_name, status_info)
                
                # Add delay to show testing process
                from PyQt5.QtCore import QThread
                QThread.msleep(500)
                
            self.log_status_message("✓ All connection tests completed")
            
        except Exception as e:
            self.logger.error(f"Error testing connections: {e}")
            self.log_status_message(f"❌ Error testing connections: {e}")
    
    def _update_system_status_display(self, system_name: str, status_info: Dict[str, Any]):
        """Update the display for a specific system"""
        if system_name not in self.status_labels:
            return
            
        status_label = self.status_labels[system_name]
        info_label = self.status_info_labels[system_name]
        detail_label = self.status_detail_labels[system_name]
        
        status = status_info.get('status', 'unknown')
        message = status_info.get('message', 'No message')
        details = status_info.get('details', {})
        
        # Update status indicator
        if status == 'available':
            status_label.setText("✅")
            status_label.setStyleSheet("color: green; font-size: 16px;")
            status_label.setToolTip("System Available")
            self.log_status_message(f"✅ {system_name}: Connected successfully")
        elif status == 'not_available':
            status_label.setText("❌")
            status_label.setStyleSheet("color: red; font-size: 16px;")
            status_label.setToolTip("System Not Available")
            self.log_status_message(f"❌ {system_name}: Connection failed")
        elif status == 'error':
            status_label.setText("⚠️")
            status_label.setStyleSheet("color: orange; font-size: 16px;")
            status_label.setToolTip("System Error")
            self.log_status_message(f"⚠️ {system_name}: Error occurred")
        else:
            status_label.setText("❓")
            status_label.setStyleSheet("color: gray; font-size: 16px;")
            status_label.setToolTip("Unknown Status")
        
        # Update info with truncation
        system_config = self.config_manager.get_system_config(system_name)
        connection_type = system_config.get('connection_type', 'unknown')
        
        if connection_type == 'tcp':
            ip = system_config.get('ip', 'N/A')
            port = system_config.get('port', 'N/A')
            info_text = f"TCP: {ip}:{port}"
        elif connection_type == 'serial':
            port = system_config.get('port', 'N/A')
            baudrate = system_config.get('baudrate', 'N/A')
            info_text = f"Serial: {port} @ {baudrate}"
        elif connection_type == 'usb':
            device_id = system_config.get('device_id', 'N/A')
            info_text = f"USB: {device_id}"
        else:
            info_text = f"{connection_type}: {message}"
        
        # Truncate info text if too long
        info_text_display = self._truncate_text(info_text, 25)
        info_label.setText(info_text_display)
        info_label.setToolTip(info_text)  # Full text in tooltip
        
        # Update details with truncation
        detail_text = message
        if details:
            detail_parts = []
            for key, value in details.items():
                detail_parts.append(f"{key}: {value}")
            if detail_parts:
                detail_text += f" ({', '.join(detail_parts)})"
        
        # Truncate detail text if too long
        detail_text_display = self._truncate_text(detail_text, 40)
        detail_label.setText(detail_text_display)
        detail_label.setToolTip(detail_text)  # Full text in tooltip
    
    def _truncate_text(self, text: str, max_length: int) -> str:
        """Truncate text to max_length with ellipsis if needed"""
        if len(text) <= max_length:
            return text
        return text[:max_length-3] + "..."
    
    def log_status_message(self, message: str):
        """Log a message to the status log"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_log.append(f"[{timestamp}] {message}")
        
        # Auto-scroll to bottom
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    def refresh_system_status(self):
        """Refresh system status display"""
        self.log_status_message("Refreshing system status...")
        
        try:
            systems_status = self.system_manager.get_all_systems_status()
            
            for system_name, status_info in systems_status.items():
                if system_name in self.status_labels:
                    self._update_system_status_display(system_name, status_info)
                    
            self.log_status_message("✓ System status refresh completed")
            
        except Exception as e:
            self.logger.error(f"Error refreshing system status: {e}")
            self.log_status_message(f"❌ Error refreshing status: {e}")
        
    def load_available_programs(self):
        """Load available programs from the programs directory"""
        try:
            programs_dir = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 
                'programs'
            )
            
            if os.path.exists(programs_dir):
                for file in os.listdir(programs_dir):
                    if file.endswith('.yaml') or file.endswith('.yml'):
                        self.programs_list.addItem(file)
        except Exception as e:
            self.logger.error(f"Error loading programs: {e}")
            QMessageBox.warning(self, "Warning", f"Could not load programs: {e}")
            
    def save_piece_info(self):
        """Save piece information"""
        self.piece_info = {
            'name_piece': self.piece_name_edit.text(),
            'ref_piece': self.piece_ref_edit.text()
        }
        
        if not self.piece_info['name_piece'] or not self.piece_info['ref_piece']:
            QMessageBox.warning(self, "Warning", "Please fill in all piece information fields.")
            return
        
        QMessageBox.information(self, "Success", "Piece information saved successfully!")
        self.update_execution_button_state()
        
    def select_program(self):
        """Select a program from the list"""
        current_item = self.programs_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Warning", "Please select a program from the list.")
            return
            
        program_file = current_item.text()
        self.load_program(program_file)
        
    def upload_program(self):
        """Upload a new program file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Upload Program", 
            "", 
            "YAML files (*.yaml *.yml)"
        )
        
        if file_path:
            # Copy file to programs directory
            try:
                import shutil
                programs_dir = os.path.join(
                    os.path.dirname(os.path.dirname(__file__)), 
                    'programs'
                )
                
                file_name = os.path.basename(file_path)
                dest_path = os.path.join(programs_dir, file_name)
                shutil.copy2(file_path, dest_path)
                
                # Refresh programs list
                self.programs_list.clear()
                self.load_available_programs()
                
                # Load the uploaded program
                self.load_program(file_name)
                
                QMessageBox.information(self, "Success", f"Program {file_name} uploaded successfully!")
                
            except Exception as e:
                self.logger.error(f"Error uploading program: {e}")
                QMessageBox.critical(self, "Error", f"Failed to upload program: {e}")
                
    def load_program(self, program_file):
        """Load a program file"""
        try:
            programs_dir = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 
                'programs'
            )
            
            program_path = os.path.join(programs_dir, program_file)
            
            with open(program_path, 'r') as file:
                self.current_program = yaml.safe_load(file)
                
            # Display program information
            program_info = self.current_program.get('program', {})
            info_text = f"Name: {program_info.get('name', 'N/A')}\n"
            info_text += f"Description: {program_info.get('description', 'N/A')}\n"
            info_text += f"Stages: {len(program_info.get('stages', []))}\n"
            
            # Show stages and steps
            stages = program_info.get('stages', [])
            for stage in stages:
                info_text += f"\nStage {stage.get('stage', '?')}: {stage.get('name', 'Unnamed')}\n"
                steps = stage.get('steps', [])
                for step in steps:
                    info_text += f"  - Step {step.get('step', '?')}: {step.get('name', 'Unnamed')} ({step.get('system', 'Unknown system')})\n"
            
            self.program_info_text.setText(info_text)
            
            QMessageBox.information(self, "Success", f"Program {program_file} loaded successfully!")
            self.update_execution_button_state()
            
        except Exception as e:
            self.logger.error(f"Error loading program: {e}")
            QMessageBox.critical(self, "Error", f"Failed to load program: {e}")
            
    def update_execution_button_state(self):
        """Update the state of execution buttons based on current state"""
        can_execute = (
            self.current_program is not None and 
            bool(self.piece_info.get('name_piece')) and 
            bool(self.piece_info.get('ref_piece'))
        )
        
        self.start_execution_btn.setEnabled(can_execute)
        
    def start_execution(self):
        """Start program execution"""
        if not self.current_program or not self.piece_info:
            QMessageBox.warning(self, "Warning", "Please select a program and fill piece information.")
            return
            
        # Disable start button, enable stop button
        self.start_execution_btn.setEnabled(False)
        self.stop_execution_btn.setEnabled(True)
        
        # Clear progress
        self.progress_text.clear()
        self.progress_bar.setValue(0)
        
        # Start execution thread
        self.execution_thread = ProgramExecutionThread(
            self.current_program, 
            self.piece_info, 
            self.config_manager
        )
        
        # Connect signals
        self.execution_thread.progress_updated.connect(self.update_progress)
        self.execution_thread.execution_finished.connect(self.execution_finished)
        self.execution_thread.step_completed.connect(self.step_completed)
        self.execution_thread.progress_percentage.connect(self.update_progress_bar)
        
        # Start thread
        self.execution_thread.start()
        
    def stop_execution(self):
        """Stop program execution"""
        if self.execution_thread and self.execution_thread.isRunning():
            self.execution_thread.terminate()
            self.execution_thread.wait()
            
        self.execution_finished(False)
        self.update_progress("Execution stopped by user.")
        
    def update_progress(self, message):
        """Update progress display"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.progress_text.append(f"[{timestamp}] {message}")
        
    def update_progress_bar(self, percentage):
        """Update progress bar percentage"""
        self.progress_bar.setValue(percentage)
        
    def step_completed(self, step_name):
        """Handle step completion"""
        self.update_progress(f"✓ Step completed: {step_name}")
        
    def execution_finished(self, success):
        """Handle execution completion"""
        # Re-enable start button, disable stop button
        self.start_execution_btn.setEnabled(True)
        self.stop_execution_btn.setEnabled(False)
        
        if success:
            self.update_progress("✓ Program execution completed successfully!")
            QMessageBox.information(self, "Success", "Inspection program completed successfully!")
        else:
            self.update_progress("✗ Program execution failed or was stopped.")
            
    def logout(self):
        """Logout and close the application"""
        reply = QMessageBox.question(
            self, 
            "Logout", 
            "Are you sure you want to logout?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.close()
            
    def closeEvent(self, event):
        """Handle window close event"""
        if self.execution_thread and self.execution_thread.isRunning():
            reply = QMessageBox.question(
                self, 
                "Close Application", 
                "An inspection is currently running. Do you want to stop it and exit?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.stop_execution()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()