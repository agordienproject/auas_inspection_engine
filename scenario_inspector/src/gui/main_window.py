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
                             QSizePolicy, QFormLayout, QScrollArea, QSpinBox, QCheckBox,
                             QApplication)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPalette


class NoWheelSpinBox(QSpinBox):
    """Custom QSpinBox that ignores wheel events to prevent accidental value changes"""
    
    def wheelEvent(self, event):
        """Ignore wheel events to prevent accidental scrolling changes"""
        event.ignore()

from config.config_manager import ConfigManager
from database.models import User
from systems.system_manager import SystemManager
from auth.login import LoginDialog
from utils.ftp_manager import FTPManager


class ProgramExecutionThread(QThread):
    """Thread for executing inspection programs without blocking the UI"""
    
    progress_updated = pyqtSignal(str)  # Progress message
    execution_finished = pyqtSignal(bool, str, object)  # Success status, inspection folder path, inspection date
    step_completed = pyqtSignal(str)  # Step name
    progress_percentage = pyqtSignal(int)  # Progress percentage for progress bar
    
    def __init__(self, program_data, piece_info, config_manager):
        super().__init__()
        self.program_data = program_data
        self.piece_info = piece_info
        self.config_manager = config_manager
        self.logger = logging.getLogger(__name__)
        
        # Use ScenarioEngine instead of direct SystemManager
        from inspector.scenario_engine import ScenarioEngine
        self.scenario_engine = ScenarioEngine(config_manager, self._emit_progress)
        
        self.inspection_folder = None
        self.inspection_date = None
    
    def _emit_progress(self, message: str):
        """Emit progress update to GUI"""
        self.progress_updated.emit(message)
    
    def run(self):
        """Execute program in thread"""
        try:
            # Execute scenario using ScenarioEngine
            success = self.scenario_engine.execute_scenario_from_gui(self.program_data, self.piece_info)
            
            # Get results from scenario engine
            self.inspection_folder = self.scenario_engine.current_inspection_folder
            self.inspection_date = self.scenario_engine.current_inspection_date
            
            if success:
                self.progress_percentage.emit(100)
                self.execution_finished.emit(True, self.inspection_folder or "", self.inspection_date)
            else:
                self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)
            
        except Exception as e:
            self.logger.error(f"Program execution error: {e}")
            self.progress_updated.emit(f"Execution failed: {str(e)}")
            self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)
    
    def _execute_steps_sequential(self, steps, current_completed, total_steps):
        """Execute steps one after another"""
        completed = 0
        for step in steps:
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            
            self.progress_updated.emit(f"Executing step: {step_name} on system: {system}")
            
            start_time = datetime.now()
            try:
                result = self.system_manager.execute_step(step)
                end_time = datetime.now()
                
                # Store execution result
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': True,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'result': result
                }
                self.execution_results.append(step_result)
                
                self.step_completed.emit(step_name)
                completed += 1
                
                # Update progress bar after each step
                new_completed = current_completed + completed
                progress_percent = int((new_completed / total_steps) * 100) if total_steps > 0 else 0
                self.progress_percentage.emit(progress_percent)
                
            except Exception as e:
                end_time = datetime.now()
                
                # Store failed execution result
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': False,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'error': str(e)
                }
                self.execution_results.append(step_result)
                
                self.progress_updated.emit(f"Step {step_name} failed: {str(e)}")
                self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)
                raise
                
        return completed
    
    def _execute_steps_parallel(self, steps, stage_name, current_completed, total_steps):
        """Execute steps in parallel using threading"""
        completed = 0
        
        def execute_single_step(step):
            """Execute a single step - to be run in thread"""
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            start_time = datetime.now()
            
            try:
                self.progress_updated.emit(f"[Parallel] Starting step: {step_name} on system: {system}")
                result = self.system_manager.execute_step(step)
                end_time = datetime.now()
                
                self.step_completed.emit(step_name)
                self.progress_updated.emit(f"[Parallel] Completed step: {step_name}")
                
                # Return step result data
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': True,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'result': result
                }
                return step_result
                
            except Exception as e:
                end_time = datetime.now()
                error_msg = f"Step {step_name} failed: {str(e)}"
                self.progress_updated.emit(f"[Parallel] {error_msg}")
                
                # Return failed step result data
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': False,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'error': str(e)
                }
                return step_result
        
        # Execute steps in parallel using ThreadPoolExecutor
        self.progress_updated.emit(f"Starting {len(steps)} parallel steps in stage: {stage_name}")
        
        with ThreadPoolExecutor(max_workers=len(steps)) as executor:
            # Submit all steps for execution
            future_to_step = {executor.submit(execute_single_step, step): step for step in steps}
            
            # Wait for all steps to complete
            for future in as_completed(future_to_step):
                step = future_to_step[future]
                try:
                    step_result = future.result()
                    
                    # Store the execution result
                    self.execution_results.append(step_result)
                    
                    if step_result['success']:
                        completed += 1
                        # Update progress bar as each parallel step completes
                        new_completed = current_completed + completed
                        progress_percent = int((new_completed / total_steps) * 100) if total_steps > 0 else 0
                        self.progress_percentage.emit(progress_percent)
                    else:
                        # If any step fails, we should stop execution
                        self.progress_updated.emit(f"Parallel execution failed: {step_result.get('error', 'Unknown error')}")
                        self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)
                        raise Exception(step_result.get('error', 'Unknown error'))
                        
                except Exception as e:
                    step_name = step.get('name', 'Unknown')
                    self.progress_updated.emit(f"Exception in parallel step {step_name}: {str(e)}")
                    raise
        
        self.progress_updated.emit(f"All {len(steps)} parallel steps completed successfully")
        return completed

    def _create_inspection_report(self):
        """Create inspection report using the file manager"""
        if not self.inspection_folder or not self.execution_results:
            self.logger.warning("Cannot create inspection report: missing inspection folder or execution results")
            self.logger.debug(f"Inspection folder: {self.inspection_folder}")
            self.logger.debug(f"Execution results count: {len(self.execution_results) if self.execution_results else 0}")
            return
        
        try:
            # Prepare report data
            program = self.program_data.get('program', {})
            program_name = program.get('name', 'Unknown')
            piece_name = self.piece_info.get('name_piece', 'Unknown')
            ref_piece = self.piece_info.get('ref_piece', 'Unknown')
            
            # Use guest user info or actual user info
            inspector_name = "Guest User"  # Default for guest mode
            
            self.logger.debug(f"Creating report for {len(self.execution_results)} steps")
            
            report_data = {
                'program_name': program_name,
                'piece_name': piece_name,
                'ref_piece': ref_piece,
                'inspection_date': self.inspection_date.strftime('%Y-%m-%d %H:%M:%S') if self.inspection_date else 'Unknown',
                'inspector': inspector_name,
                'steps': self.execution_results
            }
            
            # Create the report using the file manager
            report_path = self.system_manager.file_manager.create_inspection_report(
                self.inspection_folder, report_data
            )
            
            if report_path:
                self.progress_updated.emit(f"Inspection report created: {os.path.basename(report_path)}")
                self.logger.info(f"Inspection report created at: {report_path}")
            else:
                self.logger.warning("Failed to create inspection report")
                
        except Exception as e:
            self.logger.error(f"Error creating inspection report: {e}")
            self.progress_updated.emit(f"Warning: Failed to create inspection report: {str(e)}")


class InspectionMainWindow(QMainWindow):
    """Main window for the AUAS Inspection Engine"""
    
    def __init__(self, user: User = None):
        super().__init__()
        self.user = user  # Can be None for guest mode
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
        self.create_settings_tab()
        
        # Status bar
        if self.user:
            self.statusBar().showMessage(f"Logged in as: {self.user.pseudo}")
        else:
            self.statusBar().showMessage("Guest mode - Login to access database features")
        
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
        
        # User info and login/logout
        if self.user:
            # User is logged in
            user_label = QLabel(f"User: {self.user.pseudo}")
            header_layout.addWidget(user_label)
            
            # Logout button
            logout_btn = QPushButton("Logout")
            logout_btn.clicked.connect(self.logout)
            header_layout.addWidget(logout_btn)
        else:
            # Guest mode
            guest_label = QLabel("Guest Mode")
            guest_label.setStyleSheet("color: orange; font-weight: bold;")
            header_layout.addWidget(guest_label)
            
            # Login button
            login_btn = QPushButton("Login")
            login_btn.clicked.connect(self.show_login)
            header_layout.addWidget(login_btn)
        
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
        
        # API status indicator
        self.api_status_group = QGroupBox("API Status")
        api_status_layout = QHBoxLayout(self.api_status_group)
        
        self.api_status_label = QLabel()
        self.update_api_status_display()
        api_status_layout.addWidget(self.api_status_label)
        
        layout.addWidget(self.api_status_group)
        
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
        
        # Inspection Results group
        results_group = QGroupBox("Inspection Results")
        results_layout = QVBoxLayout(results_group)
        
        # Last inspection path
        self.last_inspection_path_label = QLabel("No inspection completed yet")
        self.last_inspection_path_label.setStyleSheet("font-style: italic; color: gray;")
        results_layout.addWidget(self.last_inspection_path_label)
        
        # Upload to FTP button
        upload_layout = QHBoxLayout()
        
        self.ftp_upload_btn = QPushButton("📤 Upload to FTP Server")
        self.ftp_upload_btn.clicked.connect(self.upload_inspection_to_ftp)
        self.ftp_upload_btn.setEnabled(False)
        self.ftp_upload_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
        upload_layout.addWidget(self.ftp_upload_btn)
        
        # Save to Database button
        self.db_save_btn = QPushButton("💾 Save to Database")
        self.db_save_btn.clicked.connect(self.save_inspection_to_database)
        self.db_save_btn.setEnabled(False)
        self.db_save_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #2196F3; color: white; }")
        upload_layout.addWidget(self.db_save_btn)
        
        # Open folder button
        self.open_folder_btn = QPushButton("📁 Open Folder")
        self.open_folder_btn.clicked.connect(self.open_inspection_folder)
        self.open_folder_btn.setEnabled(False)
        self.open_folder_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #FF9800; color: white; }")
        upload_layout.addWidget(self.open_folder_btn)
        
        results_layout.addLayout(upload_layout)
        layout.addWidget(results_group)
        
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
        
    def create_settings_tab(self):
        """Create the settings configuration tab"""
        settings_widget = QWidget()
        layout = QVBoxLayout(settings_widget)
        
        # Title and save/restart buttons
        header_layout = QHBoxLayout()
        title_label = QLabel("Application Settings")
        title_label.setStyleSheet("QLabel { font-size: 16px; font-weight: bold; }")
        header_layout.addWidget(title_label)
        
        header_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        save_btn = QPushButton("💾 Save Settings")
        save_btn.clicked.connect(self.save_settings)
        save_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
        header_layout.addWidget(save_btn)
        
        restart_btn = QPushButton("🔄 Save & Restart")
        restart_btn.clicked.connect(self.save_and_restart)
        restart_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #FF9800; color: white; }")
        header_layout.addWidget(restart_btn)
        
        layout.addLayout(header_layout)
        
        # Create scrollable area for settings
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # Initialize settings storage
        self.settings_widgets = {}
        
        # API Settings (replaces database settings)
        api_group = QGroupBox("API Configuration")
        api_layout = QGridLayout(api_group)
        
        api_config = self.config_manager.get_api_config()
        
        # API URL
        api_layout.addWidget(QLabel("API URL:"), 0, 0)
        self.settings_widgets['api_url'] = QLineEdit(str(api_config.get('url', '127.0.0.1:3000/api')))
        api_layout.addWidget(self.settings_widgets['api_url'], 0, 1)
        
        # Login endpoint
        api_layout.addWidget(QLabel("Login Endpoint:"), 1, 0)
        self.settings_widgets['api_login_endpoint'] = QLineEdit(str(api_config.get('login_endpoint', '/auth/login')))
        api_layout.addWidget(self.settings_widgets['api_login_endpoint'], 1, 1)
        
        # Inspection endpoint
        api_layout.addWidget(QLabel("Inspection Endpoint:"), 2, 0)
        self.settings_widgets['api_inspection_endpoint'] = QLineEdit(str(api_config.get('inspection_endpoint', '/inspections')))
        api_layout.addWidget(self.settings_widgets['api_inspection_endpoint'], 2, 1)
        
        # Timeout
        api_layout.addWidget(QLabel("Timeout (s):"), 3, 0)
        self.settings_widgets['api_timeout'] = NoWheelSpinBox()
        self.settings_widgets['api_timeout'].setRange(5, 300)
        self.settings_widgets['api_timeout'].setValue(int(api_config.get('timeout', 30)))
        api_layout.addWidget(self.settings_widgets['api_timeout'], 3, 1)
        
        # Use HTTPS
        self.settings_widgets['api_use_https'] = QCheckBox("Use HTTPS")
        self.settings_widgets['api_use_https'].setChecked(bool(api_config.get('use_https', False)))
        api_layout.addWidget(self.settings_widgets['api_use_https'], 4, 0, 1, 2)
        
        scroll_layout.addWidget(api_group)
        
        # Legacy Database Settings (kept for reference)
        db_group = QGroupBox("Legacy Database Configuration (API is now used)")
        db_group.setEnabled(False)  # Disable since we're using API
        db_layout = QGridLayout(db_group)
        
        db_config = self.config_manager.get_database_config()
        
        # Database host
        db_layout.addWidget(QLabel("Host:"), 0, 0)
        self.settings_widgets['database_host'] = QLineEdit(str(db_config.get('host', 'localhost')))
        db_layout.addWidget(self.settings_widgets['database_host'], 0, 1)
        
        # Database port
        db_layout.addWidget(QLabel("Port:"), 1, 0)
        self.settings_widgets['database_port'] = NoWheelSpinBox()
        self.settings_widgets['database_port'].setRange(1, 65535)
        self.settings_widgets['database_port'].setValue(int(db_config.get('port', 5432)))
        db_layout.addWidget(self.settings_widgets['database_port'], 1, 1)
        
        # Database username
        db_layout.addWidget(QLabel("Username:"), 2, 0)
        self.settings_widgets['database_username'] = QLineEdit(str(db_config.get('username', 'postgres')))
        db_layout.addWidget(self.settings_widgets['database_username'], 2, 1)
        
        # Database password
        db_layout.addWidget(QLabel("Password:"), 3, 0)
        self.settings_widgets['database_password'] = QLineEdit(str(db_config.get('password', '')))
        self.settings_widgets['database_password'].setEchoMode(QLineEdit.Password)
        db_layout.addWidget(self.settings_widgets['database_password'], 3, 1)
        
        # Database name
        db_layout.addWidget(QLabel("Database:"), 4, 0)
        self.settings_widgets['database_database'] = QLineEdit(str(db_config.get('database', 'lab_inspection')))
        db_layout.addWidget(self.settings_widgets['database_database'], 4, 1)
        
        scroll_layout.addWidget(db_group)
        
        # Systems Settings
        systems_group = QGroupBox("Systems Configuration")
        systems_layout = QVBoxLayout(systems_group)
        
        systems_config = self.config_manager.get_systems_config()
        
        for system_name, system_config in systems_config.items():
            system_subgroup = QGroupBox(f"{system_name.title()} System")
            system_sublayout = QGridLayout(system_subgroup)
            
            row = 0
            # Connection type
            system_sublayout.addWidget(QLabel("Connection Type:"), row, 0)
            self.settings_widgets[f'systems_{system_name}_connection_type'] = QLineEdit(str(system_config.get('connection_type', '')))
            system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_connection_type'], row, 1)
            row += 1
            
            # IP address (for TCP connections)
            if system_config.get('connection_type') == 'tcp':
                system_sublayout.addWidget(QLabel("IP Address:"), row, 0)
                self.settings_widgets[f'systems_{system_name}_ip'] = QLineEdit(str(system_config.get('ip', '')))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_ip'], row, 1)
                row += 1
                
                if 'port' in system_config:
                    system_sublayout.addWidget(QLabel("Port:"), row, 0)
                    self.settings_widgets[f'systems_{system_name}_port'] = NoWheelSpinBox()
                    self.settings_widgets[f'systems_{system_name}_port'].setRange(1, 65535)
                    self.settings_widgets[f'systems_{system_name}_port'].setValue(int(system_config.get('port', 8080)))
                    system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_port'], row, 1)
                    row += 1
            
            # Serial port (for serial connections)
            if system_config.get('connection_type') == 'serial':
                system_sublayout.addWidget(QLabel("Port:"), row, 0)
                self.settings_widgets[f'systems_{system_name}_port'] = QLineEdit(str(system_config.get('port', '')))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_port'], row, 1)
                row += 1
                
                if 'baudrate' in system_config:
                    system_sublayout.addWidget(QLabel("Baudrate:"), row, 0)
                    self.settings_widgets[f'systems_{system_name}_baudrate'] = NoWheelSpinBox()
                    self.settings_widgets[f'systems_{system_name}_baudrate'].setRange(300, 115200)
                    self.settings_widgets[f'systems_{system_name}_baudrate'].setValue(int(system_config.get('baudrate', 9600)))
                    system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_baudrate'], row, 1)
                    row += 1
            
            # USB device (for USB connections)
            if system_config.get('connection_type') == 'usb':
                system_sublayout.addWidget(QLabel("Device ID:"), row, 0)
                self.settings_widgets[f'systems_{system_name}_device_id'] = QLineEdit(str(system_config.get('device_id', '')))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_device_id'], row, 1)
                row += 1
            
            # Timeout
            system_sublayout.addWidget(QLabel("Timeout (s):"), row, 0)
            self.settings_widgets[f'systems_{system_name}_timeout'] = NoWheelSpinBox()
            self.settings_widgets[f'systems_{system_name}_timeout'].setRange(1, 300)
            self.settings_widgets[f'systems_{system_name}_timeout'].setValue(int(system_config.get('timeout', 30)))
            system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_timeout'], row, 1)
            row += 1
            
            # Programs path (for gantry and xarm)
            if 'programs_path' in system_config:
                system_sublayout.addWidget(QLabel("Programs Path:"), row, 0)
                self.settings_widgets[f'systems_{system_name}_programs_path'] = QLineEdit(str(system_config.get('programs_path', '')))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_programs_path'], row, 1)
                row += 1
            
            # Scanner-specific settings
            if 'llt_path' in system_config:
                system_sublayout.addWidget(QLabel("LLT SDK Path:"), row, 0)
                self.settings_widgets[f'systems_{system_name}_llt_path'] = QLineEdit(str(system_config.get('llt_path', '')))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_llt_path'], row, 1)
                row += 1
            
            # Camera-specific settings
            if 'use_realsense_sdk' in system_config:
                self.settings_widgets[f'systems_{system_name}_use_realsense_sdk'] = QCheckBox("Use RealSense SDK")
                self.settings_widgets[f'systems_{system_name}_use_realsense_sdk'].setChecked(bool(system_config.get('use_realsense_sdk', False)))
                system_sublayout.addWidget(self.settings_widgets[f'systems_{system_name}_use_realsense_sdk'], row, 0, 1, 2)
                row += 1
            
            systems_layout.addWidget(system_subgroup)
        
        scroll_layout.addWidget(systems_group)
        
        # Output Settings
        output_group = QGroupBox("Output Configuration")
        output_layout = QGridLayout(output_group)
        
        output_config = self.config_manager.get_config().get('output', {})
        
        output_layout.addWidget(QLabel("Base Directory:"), 0, 0)
        self.settings_widgets['output_base_directory'] = QLineEdit(str(output_config.get('base_directory', 'output')))
        output_layout.addWidget(self.settings_widgets['output_base_directory'], 0, 1)
        
        output_layout.addWidget(QLabel("Inspection Prefix:"), 1, 0)
        self.settings_widgets['output_inspection_folder_prefix'] = QLineEdit(str(output_config.get('inspection_folder_prefix', 'inspection_')))
        output_layout.addWidget(self.settings_widgets['output_inspection_folder_prefix'], 1, 1)
        
        output_layout.addWidget(QLabel("Date Format:"), 2, 0)
        self.settings_widgets['output_date_format'] = QLineEdit(str(output_config.get('date_format', '%Y-%m-%d')))
        output_layout.addWidget(self.settings_widgets['output_date_format'], 2, 1)
        
        output_layout.addWidget(QLabel("Time Format:"), 3, 0)
        self.settings_widgets['output_time_format'] = QLineEdit(str(output_config.get('time_format', '%Y%m%d_%H%M%S')))
        output_layout.addWidget(self.settings_widgets['output_time_format'], 3, 1)
        
        scroll_layout.addWidget(output_group)
        
        # FTP Settings
        ftp_group = QGroupBox("FTP Configuration")
        ftp_layout = QGridLayout(ftp_group)
        
        ftp_config = self.config_manager.get_config().get('ftp', {})
        
        ftp_layout.addWidget(QLabel("FTP Server:"), 0, 0)
        self.settings_widgets['ftp_server'] = QLineEdit(str(ftp_config.get('server', 'ftp://127.0.0.1')))
        ftp_layout.addWidget(self.settings_widgets['ftp_server'], 0, 1)
        
        ftp_layout.addWidget(QLabel("Username:"), 1, 0)
        self.settings_widgets['ftp_username'] = QLineEdit(str(ftp_config.get('username', 'anonymous')))
        ftp_layout.addWidget(self.settings_widgets['ftp_username'], 1, 1)
        
        ftp_layout.addWidget(QLabel("Password:"), 2, 0)
        self.settings_widgets['ftp_password'] = QLineEdit(str(ftp_config.get('password', '')))
        self.settings_widgets['ftp_password'].setEchoMode(QLineEdit.Password)
        ftp_layout.addWidget(self.settings_widgets['ftp_password'], 2, 1)
        
        ftp_layout.addWidget(QLabel("Base Path:"), 3, 0)
        self.settings_widgets['ftp_base_path'] = QLineEdit(str(ftp_config.get('base_path', '/inspections')))
        ftp_layout.addWidget(self.settings_widgets['ftp_base_path'], 3, 1)
        
        # Passive mode option
        ftp_layout.addWidget(QLabel("Passive Mode:"), 4, 0)
        self.settings_widgets['ftp_passive_mode'] = QCheckBox("Use passive mode (recommended)")
        self.settings_widgets['ftp_passive_mode'].setChecked(ftp_config.get('passive_mode', True))
        ftp_layout.addWidget(self.settings_widgets['ftp_passive_mode'], 4, 1)
        
        # FTP test connection button
        ftp_test_btn = QPushButton("🔗 Test FTP Connection")
        ftp_test_btn.clicked.connect(self.test_ftp_connection)
        ftp_test_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #2196F3; color: white; }")
        ftp_layout.addWidget(ftp_test_btn, 5, 0, 1, 2)
        
        scroll_layout.addWidget(ftp_group)
        
        # Logging Settings
        logging_group = QGroupBox("Logging Configuration")
        logging_layout = QGridLayout(logging_group)
        
        logging_config = self.config_manager.get_config().get('logging', {})
        
        logging_layout.addWidget(QLabel("Log Level:"), 0, 0)
        self.settings_widgets['logging_level'] = QComboBox()
        self.settings_widgets['logging_level'].addItems(['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'])
        current_level = str(logging_config.get('level', 'INFO'))
        if current_level in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
            self.settings_widgets['logging_level'].setCurrentText(current_level)
        logging_layout.addWidget(self.settings_widgets['logging_level'], 0, 1)
        
        logging_layout.addWidget(QLabel("Log Folder:"), 1, 0)
        self.settings_widgets['logging_log_folder_path'] = QLineEdit(str(logging_config.get('log_folder_path', 'logs')))
        logging_layout.addWidget(self.settings_widgets['logging_log_folder_path'], 1, 1)
        
        logging_layout.addWidget(QLabel("Log File Prefix:"), 2, 0)
        self.settings_widgets['logging_log_file_prefix'] = QLineEdit(str(logging_config.get('log_file_prefix', 'app')))
        logging_layout.addWidget(self.settings_widgets['logging_log_file_prefix'], 2, 1)
        
        self.settings_widgets['logging_console'] = QCheckBox("Enable Console Logging")
        self.settings_widgets['logging_console'].setChecked(bool(logging_config.get('console', True)))
        logging_layout.addWidget(self.settings_widgets['logging_console'], 3, 0, 1, 2)
        
        scroll_layout.addWidget(logging_group)
        
        # GUI Settings
        gui_group = QGroupBox("GUI Configuration")
        gui_layout = QGridLayout(gui_group)
        
        gui_config = self.config_manager.get_config().get('gui', {})
        
        gui_layout.addWidget(QLabel("Window Title:"), 0, 0)
        self.settings_widgets['gui_window_title'] = QLineEdit(str(gui_config.get('window_title', 'AUAS Scenario Inspector')))
        gui_layout.addWidget(self.settings_widgets['gui_window_title'], 0, 1)
        
        gui_layout.addWidget(QLabel("Theme:"), 1, 0)
        self.settings_widgets['gui_theme'] = QComboBox()
        self.settings_widgets['gui_theme'].addItems(['default', 'dark', 'light'])
        current_theme = str(gui_config.get('theme', 'default'))
        if current_theme in ['default', 'dark', 'light']:
            self.settings_widgets['gui_theme'].setCurrentText(current_theme)
        gui_layout.addWidget(self.settings_widgets['gui_theme'], 1, 1)
        
        gui_layout.addWidget(QLabel("Default Geometry:"), 2, 0)
        self.settings_widgets['gui_default_geometry'] = QLineEdit(str(gui_config.get('default_geometry', '1200x800')))
        gui_layout.addWidget(self.settings_widgets['gui_default_geometry'], 2, 1)
        
        scroll_layout.addWidget(gui_group)
        
        # Security Settings
        security_group = QGroupBox("Security Configuration")
        security_layout = QGridLayout(security_group)
        
        security_config = self.config_manager.get_config().get('security', {})
        
        security_layout.addWidget(QLabel("Bcrypt Rounds:"), 0, 0)
        self.settings_widgets['security_bcrypt_rounds'] = NoWheelSpinBox()
        self.settings_widgets['security_bcrypt_rounds'].setRange(4, 20)
        self.settings_widgets['security_bcrypt_rounds'].setValue(int(security_config.get('bcrypt_rounds', 12)))
        security_layout.addWidget(self.settings_widgets['security_bcrypt_rounds'], 0, 1)
        
        security_layout.addWidget(QLabel("Session Timeout (s):"), 1, 0)
        self.settings_widgets['security_session_timeout'] = NoWheelSpinBox()
        self.settings_widgets['security_session_timeout'].setRange(60, 86400)
        self.settings_widgets['security_session_timeout'].setValue(int(security_config.get('session_timeout', 3600)))
        security_layout.addWidget(self.settings_widgets['security_session_timeout'], 1, 1)
        
        scroll_layout.addWidget(security_group)
        
        # Set up scroll area
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)
        layout.addWidget(scroll_area)
        
        self.tab_widget.addTab(settings_widget, "Settings")
        
    def save_settings(self):
        """Save the current settings to app_config.yaml"""
        try:
            # Update configuration with current widget values
            config = self.config_manager.get_config().copy()
            
            # API settings (new)
            if 'api' not in config:
                config['api'] = {}
            config['api']['url'] = self.settings_widgets['api_url'].text()
            config['api']['login_endpoint'] = self.settings_widgets['api_login_endpoint'].text()
            config['api']['inspection_endpoint'] = self.settings_widgets['api_inspection_endpoint'].text()
            config['api']['timeout'] = self.settings_widgets['api_timeout'].value()
            config['api']['use_https'] = self.settings_widgets['api_use_https'].isChecked()
            
            # Database settings (legacy, kept for reference)
            if 'database' not in config:
                config['database'] = {}
            config['database']['host'] = self.settings_widgets['database_host'].text()
            config['database']['port'] = self.settings_widgets['database_port'].value()
            config['database']['username'] = self.settings_widgets['database_username'].text()
            config['database']['password'] = self.settings_widgets['database_password'].text()
            config['database']['database'] = self.settings_widgets['database_database'].text()
            
            # Systems settings
            if 'systems' not in config:
                config['systems'] = {}
            
            systems_config = self.config_manager.get_systems_config()
            for system_name, system_config in systems_config.items():
                if system_name not in config['systems']:
                    config['systems'][system_name] = {}
                
                # Update each system's settings
                for key, value in system_config.items():
                    widget_key = f'systems_{system_name}_{key}'
                    if widget_key in self.settings_widgets:
                        widget = self.settings_widgets[widget_key]
                        if isinstance(widget, QLineEdit):
                            config['systems'][system_name][key] = widget.text()
                        elif isinstance(widget, (QSpinBox, NoWheelSpinBox)):
                            config['systems'][system_name][key] = widget.value()
                        elif isinstance(widget, QCheckBox):
                            config['systems'][system_name][key] = widget.isChecked()
                        elif isinstance(widget, QComboBox):
                            config['systems'][system_name][key] = widget.currentText()
            
            # Output settings
            if 'output' not in config:
                config['output'] = {}
            config['output']['base_directory'] = self.settings_widgets['output_base_directory'].text()
            config['output']['inspection_folder_prefix'] = self.settings_widgets['output_inspection_folder_prefix'].text()
            config['output']['date_format'] = self.settings_widgets['output_date_format'].text()
            config['output']['time_format'] = self.settings_widgets['output_time_format'].text()
            
            # FTP settings
            if 'ftp' not in config:
                config['ftp'] = {}
            config['ftp']['server'] = self.settings_widgets['ftp_server'].text()
            config['ftp']['username'] = self.settings_widgets['ftp_username'].text()
            config['ftp']['password'] = self.settings_widgets['ftp_password'].text()
            config['ftp']['base_path'] = self.settings_widgets['ftp_base_path'].text()
            config['ftp']['passive_mode'] = self.settings_widgets['ftp_passive_mode'].isChecked()
            config['ftp']['passive_mode'] = self.settings_widgets['ftp_passive_mode'].isChecked()
            config['ftp']['tls_mode'] = self.settings_widgets['ftp_tls_mode'].currentText()
            
            # Logging settings
            if 'logging' not in config:
                config['logging'] = {}
            config['logging']['level'] = self.settings_widgets['logging_level'].currentText()
            config['logging']['log_folder_path'] = self.settings_widgets['logging_log_folder_path'].text()
            config['logging']['log_file_prefix'] = self.settings_widgets['logging_log_file_prefix'].text()
            config['logging']['console'] = self.settings_widgets['logging_console'].isChecked()
            
            # GUI settings
            if 'gui' not in config:
                config['gui'] = {}
            config['gui']['window_title'] = self.settings_widgets['gui_window_title'].text()
            config['gui']['theme'] = self.settings_widgets['gui_theme'].currentText()
            config['gui']['default_geometry'] = self.settings_widgets['gui_default_geometry'].text()
            
            # Security settings
            if 'security' not in config:
                config['security'] = {}
            config['security']['bcrypt_rounds'] = self.settings_widgets['security_bcrypt_rounds'].value()
            config['security']['session_timeout'] = self.settings_widgets['security_session_timeout'].value()
            
            # Save to file with preserved formatting
            import yaml
            import os
            
            config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'config', 'app_config.yaml')
            
            # Read the original file as text to preserve formatting
            with open(config_path, 'r') as f:
                lines = f.readlines()
            
            # Function to update specific values while preserving format
            def update_yaml_line(lines, section, key, new_value, subsection=None):
                in_section = False
                in_subsection = False if subsection else True
                
                for i, line in enumerate(lines):
                    # Check if we're entering the target section
                    if line.strip().startswith(f'{section}:'):
                        in_section = True
                        continue
                    
                    # Check if we're leaving the section (new top-level section)
                    if in_section and line.startswith((' ', '\t')) == False and line.strip() and ':' in line:
                        in_section = False
                        in_subsection = False
                        continue
                    
                    # If we need a subsection, check for it
                    if in_section and subsection and line.strip().startswith(f'{subsection}:'):
                        in_subsection = True
                        continue
                    
                    # Check if we're leaving the subsection
                    if in_section and subsection and in_subsection and line.strip() and not line.startswith((' ', '\t', f'  {key}:')):
                        if line.strip().endswith(':') and not line.strip().startswith('#'):
                            in_subsection = False
                            continue
                    
                    # Look for the target key
                    if in_section and in_subsection and line.strip().startswith(f'{key}:'):
                        # Format the new value
                        if isinstance(new_value, str):
                            # Handle Windows paths - double the backslashes
                            if '\\' in new_value and not new_value.startswith(('ftp://', 'http://', 'https://')):
                                # This is a Windows path, escape backslashes
                                escaped_value = new_value.replace('\\', '\\\\')
                                formatted_value = f'"{escaped_value}"'
                            else:
                                formatted_value = f'"{new_value}"'
                        elif isinstance(new_value, bool):
                            formatted_value = str(new_value).lower()
                        else:
                            formatted_value = str(new_value)
                        
                        # Preserve indentation
                        indent = line[:len(line) - len(line.lstrip())]
                        lines[i] = f'{indent}{key}: {formatted_value}\n'
                        break
                
                return lines
            
            # Update each changed value
            updated_lines = lines[:]
            
            # Database settings
            updated_lines = update_yaml_line(updated_lines, 'database', 'host', self.settings_widgets['database_host'].text())
            updated_lines = update_yaml_line(updated_lines, 'database', 'port', self.settings_widgets['database_port'].value())
            updated_lines = update_yaml_line(updated_lines, 'database', 'username', self.settings_widgets['database_username'].text())
            updated_lines = update_yaml_line(updated_lines, 'database', 'password', self.settings_widgets['database_password'].text())
            updated_lines = update_yaml_line(updated_lines, 'database', 'database', self.settings_widgets['database_database'].text())
            
            # Systems settings
            systems_config = self.config_manager.get_systems_config()
            for system_name, system_config in systems_config.items():
                for key, value in system_config.items():
                    widget_key = f'systems_{system_name}_{key}'
                    if widget_key in self.settings_widgets:
                        widget = self.settings_widgets[widget_key]
                        if isinstance(widget, QLineEdit):
                            new_value = widget.text()
                        elif isinstance(widget, (QSpinBox, NoWheelSpinBox)):
                            new_value = widget.value()
                        elif isinstance(widget, QCheckBox):
                            new_value = widget.isChecked()
                        elif isinstance(widget, QComboBox):
                            new_value = widget.currentText()
                        else:
                            continue
                        
                        updated_lines = update_yaml_line(updated_lines, 'systems', key, new_value, system_name)
            
            # Output settings
            updated_lines = update_yaml_line(updated_lines, 'output', 'base_directory', self.settings_widgets['output_base_directory'].text())
            updated_lines = update_yaml_line(updated_lines, 'output', 'inspection_folder_prefix', self.settings_widgets['output_inspection_folder_prefix'].text())
            updated_lines = update_yaml_line(updated_lines, 'output', 'date_format', self.settings_widgets['output_date_format'].text())
            updated_lines = update_yaml_line(updated_lines, 'output', 'time_format', self.settings_widgets['output_time_format'].text())
            
            # Logging settings
            updated_lines = update_yaml_line(updated_lines, 'logging', 'level', self.settings_widgets['logging_level'].currentText())
            updated_lines = update_yaml_line(updated_lines, 'logging', 'log_folder_path', self.settings_widgets['logging_log_folder_path'].text())
            updated_lines = update_yaml_line(updated_lines, 'logging', 'log_file_prefix', self.settings_widgets['logging_log_file_prefix'].text())
            updated_lines = update_yaml_line(updated_lines, 'logging', 'console', self.settings_widgets['logging_console'].isChecked())
            
            # GUI settings
            updated_lines = update_yaml_line(updated_lines, 'gui', 'window_title', self.settings_widgets['gui_window_title'].text())
            updated_lines = update_yaml_line(updated_lines, 'gui', 'theme', self.settings_widgets['gui_theme'].currentText())
            updated_lines = update_yaml_line(updated_lines, 'gui', 'default_geometry', self.settings_widgets['gui_default_geometry'].text())
            
            # Security settings
            updated_lines = update_yaml_line(updated_lines, 'security', 'bcrypt_rounds', self.settings_widgets['security_bcrypt_rounds'].value())
            updated_lines = update_yaml_line(updated_lines, 'security', 'session_timeout', self.settings_widgets['security_session_timeout'].value())
            
            # Write the updated content back
            with open(config_path, 'w') as f:
                f.writelines(updated_lines)
            
            QMessageBox.information(self, "Settings Saved", "Settings have been saved successfully!")
            
        except Exception as e:
            QMessageBox.critical(self, "Error Saving Settings", f"Failed to save settings: {str(e)}")
    
    def save_and_restart(self):
        """Save settings and restart the application"""
        self.save_settings()
        
        # Ask for confirmation
        reply = QMessageBox.question(self, "Restart Application", 
                                   "Settings saved. Do you want to restart the application now?",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Restart the application
            import sys
            QApplication.quit()
            os.execv(sys.executable, ['python'] + sys.argv)
        
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
        
        # Inform guest users about limited functionality (but don't block)
        if not self.user:
            QMessageBox.information(
                self, 
                "Guest Mode", 
                "You are running in guest mode.\n\n"
                "• Inspection will execute normally\n"
                "• Data will be saved locally in output folder\n"
                "• Results will NOT be saved to database\n\n"
                "Login anytime to enable database logging."
            )
                
        # Disable start button, enable stop button
        self.start_execution_btn.setEnabled(False)
        self.stop_execution_btn.setEnabled(True)
        
        # Store inspection metadata for later database save
        self.last_piece_name = self.piece_info.get('name_piece', 'Unknown')
        self.last_ref_piece = self.piece_info.get('ref_piece', 'Unknown')
        self.last_program_name = self.current_program.get('program', {}).get('name', 'Unknown')
        
        # Clear progress
        self.progress_text.clear()
        self.progress_bar.setValue(0)
        
        # Add status message about database logging
        if self.user:
            self.update_progress(f"🔐 Authenticated user: {self.user.pseudo} - Database logging enabled")
        else:
            self.update_progress("⚠️ Guest mode: Inspection data will not be saved to database")
        
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
        
    def execution_finished(self, success, inspection_folder="", inspection_date=None):
        """Handle execution completion"""
        # Re-enable start button, disable stop button
        self.start_execution_btn.setEnabled(True)
        self.stop_execution_btn.setEnabled(False)
        
        # Store inspection folder and date for FTP upload
        if inspection_folder and os.path.exists(inspection_folder):
            self.last_inspection_folder = inspection_folder
            self.last_inspection_date = inspection_date or datetime.now()
            
            # Update the label to show the folder path
            folder_name = os.path.basename(inspection_folder)
            self.last_inspection_path_label.setText(f"📁 Last inspection: {folder_name}")
            self.last_inspection_path_label.setStyleSheet("font-weight: bold; color: green;")
            
            # Enable upload and open folder buttons
            self.ftp_upload_btn.setEnabled(True)
            self.db_save_btn.setEnabled(True)
            self.open_folder_btn.setEnabled(True)
        else:
            # Reset if no valid folder
            self.last_inspection_folder = None
            self.last_inspection_date = None
            self.last_inspection_path_label.setText("No inspection completed yet")
            self.last_inspection_path_label.setStyleSheet("font-style: italic; color: gray;")
            self.ftp_upload_btn.setEnabled(False)
            self.db_save_btn.setEnabled(False)
            self.open_folder_btn.setEnabled(False)
        
        if success:
            self.update_progress("✓ Program execution completed successfully!")
            QMessageBox.information(self, "Success", "Inspection program completed successfully!")
        else:
            self.update_progress("✗ Program execution failed or was stopped.")
            
    def show_login(self):
        """Show login dialog and authenticate user"""
        try:
            login_dialog = LoginDialog()
            if login_dialog.exec_() == LoginDialog.Accepted:
                # Get authenticated user
                authenticated_user = login_dialog.get_authenticated_user()
                if authenticated_user:
                    self.user = authenticated_user
                    self.logger.info(f"User {authenticated_user.pseudo} logged in successfully")
                    
                    # Update UI to reflect logged in state
                    self.update_ui_after_login()
                    
                    # Update status bar
                    self.statusBar().showMessage(f"Logged in as: {self.user.pseudo}")
                else:
                    QMessageBox.warning(self, "Login Failed", "Authentication failed. Please try again.")
        except Exception as e:
            self.logger.error(f"Login error: {e}")
            QMessageBox.critical(self, "Login Error", f"An error occurred during login: {str(e)}")
    
    def logout(self):
        """Logout user and switch to guest mode"""
        if self.user:
            reply = QMessageBox.question(
                self, 
                "Logout", 
                f"Are you sure you want to logout user '{self.user.pseudo}'?\nYou will switch to guest mode.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.user = None
                self.logger.info("User logged out, switched to guest mode")
                
                # Update UI to reflect guest mode
                self.update_ui_after_logout()
                
                # Update status bar
                self.statusBar().showMessage("Guest mode - Login to access database features")
        else:
            # If no user is logged in, offer to close application
            reply = QMessageBox.question(
                self, 
                "Close Application", 
                "Close the application?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.close()
    
    def update_api_status_display(self):
        """Update the API status display"""
        if hasattr(self, 'api_status_label'):
            if self.user and hasattr(self.user, 'api_connection') and self.user.api_connection:
                if self.user.api_connection.is_authenticated():
                    self.api_status_label.setText(f"🔐 Logged in as: {self.user.pseudo} | API connection: ACTIVE")
                    self.api_status_label.setStyleSheet("color: green; font-weight: bold;")
                else:
                    self.api_status_label.setText(f"⚠️ User: {self.user.pseudo} | API connection: EXPIRED")
                    self.api_status_label.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.api_status_label.setText("⚠️ Guest Mode | API connection: DISABLED")
                self.api_status_label.setStyleSheet("color: orange; font-weight: bold;")
    
    def test_ftp_connection(self):
        """Test FTP connection using current settings"""
        try:
            # Get current FTP settings from widgets
            ftp_config = {
                'server': self.settings_widgets['ftp_server'].text(),
                'username': self.settings_widgets['ftp_username'].text(),
                'password': self.settings_widgets['ftp_password'].text(),
                'base_path': self.settings_widgets['ftp_base_path'].text(),
                'passive_mode': self.settings_widgets['ftp_passive_mode'].isChecked()
            }
            
            # Create FTP manager and test connection
            ftp_manager = FTPManager(ftp_config)
            result = ftp_manager.test_connection()
            
            if result['success']:
                QMessageBox.information(
                    self, 
                    "FTP Connection Test", 
                    f"✅ {result['message']}\n\nDetails:\n"
                    f"• Host: {result['details']['host']}:{result['details']['port']}\n"
                    f"• Base Path: {result['details']['base_path']}\n"
                    f"• Encryption: {result['details']['encryption']}\n"
                    f"• Files in root: {result['details']['files_in_root']}"
                )
            else:
                QMessageBox.warning(
                    self, 
                    "FTP Connection Test Failed", 
                    f"❌ {result['message']}\n\nPlease check your FTP settings and ensure the server is accessible."
                )
                
        except Exception as e:
            QMessageBox.critical(
                self, 
                "FTP Test Error", 
                f"❌ Error testing FTP connection:\n{str(e)}"
            )
    
    def upload_inspection_to_ftp(self):
        """Upload the last inspection folder to FTP server"""
        if not hasattr(self, 'last_inspection_folder') or not self.last_inspection_folder:
            QMessageBox.warning(self, "No Inspection", "No inspection folder available for upload.")
            return
        
        if not os.path.exists(self.last_inspection_folder):
            QMessageBox.warning(self, "Folder Not Found", f"Inspection folder not found:\n{self.last_inspection_folder}")
            return
        
        try:
            # Get FTP configuration
            ftp_config = self.config_manager.get_config().get('ftp', {})
            
            if not ftp_config.get('server'):
                QMessageBox.warning(self, "FTP Not Configured", "FTP server is not configured. Please check your settings.")
                return
            
            # Disable button during upload
            self.ftp_upload_btn.setEnabled(False)
            self.ftp_upload_btn.setText("📤 Uploading...")
            
            # Create FTP manager and upload
            ftp_manager = FTPManager(ftp_config)
            
            # Use the inspection date if available, otherwise current date
            inspection_date = getattr(self, 'last_inspection_date', datetime.now())
            
            result = ftp_manager.upload_inspection_folder(self.last_inspection_folder, inspection_date)
            
            if result['success']:
                # Store FTP path for database save
                self.last_ftp_path = result['details']['ftp_path']
                
                QMessageBox.information(
                    self, 
                    "Upload Successful", 
                    f"✅ {result['message']}\n\n"
                    f"📁 FTP Path: {result['details']['ftp_path']}\n"
                    f"📄 Uploaded {len(result['details']['uploaded_files'])} files"
                )
            else:
                error_details = ""
                if 'details' in result and 'failed_files' in result['details']:
                    failed_count = len(result['details']['failed_files'])
                    error_details = f"\n\n❌ {failed_count} files failed to upload"
                
                QMessageBox.warning(
                    self, 
                    "Upload Failed", 
                    f"❌ {result['message']}{error_details}"
                )
                
        except Exception as e:
            QMessageBox.critical(
                self, 
                "Upload Error", 
                f"❌ Error during FTP upload:\n{str(e)}"
            )
        finally:
            # Re-enable button
            self.ftp_upload_btn.setEnabled(True)
            self.ftp_upload_btn.setText("📤 Upload to FTP Server")
    
    def save_inspection_to_database(self):
        """Save inspection data to database"""
        if not hasattr(self, 'last_inspection_folder') or not self.last_inspection_folder:
            QMessageBox.warning(self, "No Inspection", "No inspection data available to save.")
            return
        
        # Check if user is logged in
        if not self.user:
            QMessageBox.warning(
                self, 
                "Login Required", 
                "You must be logged in to save inspection data to the database."
            )
            return
        
        try:
            # Prepare inspection data for the dialog
            inspection_data = {
                'piece_name': getattr(self, 'last_piece_name', 'Unknown'),
                'ref_piece': getattr(self, 'last_ref_piece', 'Unknown'), 
                'program_name': getattr(self, 'last_program_name', 'Unknown'),
                'inspection_date': self.last_inspection_date.strftime('%Y-%m-%d %H:%M:%S') if self.last_inspection_date else datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'inspection_path': ''  # Will be filled after FTP upload
            }
            
            # Create and show the database dialog
            dialog = InspectionDatabaseDialog(inspection_data, self)
            
            # If user has uploaded to FTP, get the path
            if hasattr(self, 'last_ftp_path') and self.last_ftp_path:
                dialog.update_ftp_path(self.last_ftp_path)
            
            if dialog.exec_() == QMessageBox.Accepted:
                form_data = dialog.get_result_data()
                if form_data:
                    self._send_to_database(form_data)
                    
        except Exception as e:
            QMessageBox.critical(
                self,
                "Database Save Error",
                f"❌ Error preparing database save:\n{str(e)}"
            )
    
    def _send_to_database(self, inspection_data):
        """Send inspection data to database via API"""
        try:
            # Disable button during save
            self.db_save_btn.setEnabled(False)
            self.db_save_btn.setText("💾 Saving...")
            
            # Import requests for API calls
            import requests
            
            # Prepare the JSON payload
            # Convert datetime to ISO format for API
            inspection_date = inspection_data['inspection_date']
            if isinstance(inspection_date, str):
                # Parse the date string and convert to ISO format
                from datetime import datetime
                dt = datetime.strptime(inspection_date, '%Y-%m-%d %H:%M:%S')
                iso_date = dt.strftime('%Y-%m-%dT%H:%M:%S.000Z')
            else:
                iso_date = inspection_date.strftime('%Y-%m-%dT%H:%M:%S.000Z')
            
            payload = {
                "name_piece": inspection_data['name_piece'],
                "ref_piece": inspection_data['ref_piece'], 
                "name_program": inspection_data['name_program'],
                "state": inspection_data['state'],
                "dents": inspection_data['dents'],
                "corrosions": inspection_data['corrosions'],
                "scratches": inspection_data['scratches'],
                "details": inspection_data['details'],
                "inspection_date": iso_date,
                "inspection_path": inspection_data['inspection_path']
            }
            
            # Use the authenticated API connection instead of creating a new session
            if hasattr(self.user, 'api_connection') and self.user.api_connection:
                # Use the authenticated API connection
                api_result = self.user.api_connection.save_inspection_data(payload)
                
                if api_result.get('success', False):
                    QMessageBox.information(
                        self,
                        "Success",
                        "✅ Inspection data saved to database successfully!"
                    )
                    self.update_progress("✓ Inspection data saved to database")
                else:
                    error_msg = api_result.get('message', 'Unknown error')
                    QMessageBox.critical(
                        self,
                        "Database Save Failed",
                        f"❌ Failed to save to database:\n{error_msg}"
                    )
            else:
                QMessageBox.critical(
                    self,
                    "Authentication Error",
                    "❌ No authenticated API connection available. Please login again."
                )
                
        except Exception as e:
            QMessageBox.critical(
                self,
                "Database Save Error",
                f"❌ Error saving to database:\n{str(e)}"
            )
        finally:
            # Re-enable button
            self.db_save_btn.setEnabled(True)
            self.db_save_btn.setText("💾 Save to Database")
    
    def open_inspection_folder(self):
        """Open the last inspection folder in file explorer"""
        if not hasattr(self, 'last_inspection_folder') or not self.last_inspection_folder:
            QMessageBox.warning(self, "No Inspection", "No inspection folder available.")
            return
        
        if not os.path.exists(self.last_inspection_folder):
            QMessageBox.warning(self, "Folder Not Found", f"Inspection folder not found:\n{self.last_inspection_folder}")
            return
        
        try:
            # Open folder in Windows Explorer
            os.startfile(self.last_inspection_folder)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open folder:\n{str(e)}")
    
    def update_database_status_display(self):
        """Update the database status display (legacy, now shows API status)"""
        self.update_api_status_display()
    
    def update_ui_after_login(self):
        """Update UI elements after successful login"""
        # Update database status display
        self.update_database_status_display()
        
        # Update header to show logged in state
        self.update_header()
        
        # Update status bar
        self.statusBar().showMessage(f"Logged in as: {self.user.pseudo}")
        
        # Update execution button state
        self.update_execution_button_state()
        
    def update_ui_after_logout(self):
        """Update UI elements after logout"""
        # Update database status display
        self.update_database_status_display()
        
        # Update header to show guest mode
        self.update_header()
        
        # Update status bar
        self.statusBar().showMessage("Guest mode - Login to access database features")
        
        # Update execution button state
        self.update_execution_button_state()
    
    def update_header(self):
        """Update just the header section without recreating entire UI"""
        # Find and remove the existing header frame
        main_layout = self.centralWidget().layout()
        header_frame = None
        
        for i in range(main_layout.count()):
            item = main_layout.itemAt(i)
            if item.widget() and isinstance(item.widget(), QFrame):
                # This is likely our header frame (first QFrame in layout)
                header_frame = item.widget()
                break
        
        if header_frame:
            header_frame.setParent(None)
        
        # Create new header and insert at position 0
        new_header_frame = QFrame()
        new_header_frame.setFrameStyle(QFrame.StyledPanel)
        new_header_frame.setMaximumHeight(80)
        
        header_layout = QHBoxLayout(new_header_frame)
        
        # Title
        title_label = QLabel("AUAS Inspection Engine")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        header_layout.addWidget(title_label)
        
        # Spacer
        header_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        # User info and login/logout
        if self.user:
            # User is logged in
            user_label = QLabel(f"User: {self.user.pseudo}")
            header_layout.addWidget(user_label)
            
            # Logout button
            logout_btn = QPushButton("Logout")
            logout_btn.clicked.connect(self.logout)
            header_layout.addWidget(logout_btn)
        else:
            # Guest mode
            guest_label = QLabel("Guest Mode")
            guest_label.setStyleSheet("color: orange; font-weight: bold;")
            header_layout.addWidget(guest_label)
            
            # Login button
            login_btn = QPushButton("Login")
            login_btn.clicked.connect(self.show_login)
            header_layout.addWidget(login_btn)
        
        # Insert new header at the top
        main_layout.insertWidget(0, new_header_frame)
            
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


class InspectionDatabaseDialog(QMessageBox):
    """Dialog for saving inspection data to database"""
    
    def __init__(self, inspection_data, parent=None):
        super().__init__(parent)
        self.inspection_data = inspection_data
        self.result_data = None
        
        self.setWindowTitle("Save Inspection to Database")
        self.setIcon(QMessageBox.Question)
        self.setText("Please fill in the inspection details:")
        
        # Create custom widget for the form
        self.setup_form()
        
    def setup_form(self):
        """Setup the form fields"""
        from PyQt5.QtWidgets import QWidget, QVBoxLayout, QFormLayout, QComboBox, QCheckBox, QTextEdit
        
        # Create main widget
        widget = QWidget()
        main_layout = QVBoxLayout(widget)
        
        # Create form layout
        form_layout = QFormLayout()
        
        # Name of piece (read-only, pre-filled)
        self.name_piece_edit = QLineEdit(self.inspection_data.get('piece_name', ''))
        self.name_piece_edit.setReadOnly(True)
        self.name_piece_edit.setStyleSheet("background-color: #f0f0f0;")
        form_layout.addRow("Piece Name:", self.name_piece_edit)
        
        # Reference of piece (read-only, pre-filled)
        self.ref_piece_edit = QLineEdit(self.inspection_data.get('ref_piece', ''))
        self.ref_piece_edit.setReadOnly(True)
        self.ref_piece_edit.setStyleSheet("background-color: #f0f0f0;")
        form_layout.addRow("Piece Reference:", self.ref_piece_edit)
        
        # Program name (read-only, pre-filled)
        self.program_name_edit = QLineEdit(self.inspection_data.get('program_name', ''))
        self.program_name_edit.setReadOnly(True)
        self.program_name_edit.setStyleSheet("background-color: #f0f0f0;")
        form_layout.addRow("Program Name:", self.program_name_edit)
        
        # State dropdown (user selectable)
        self.state_combo = QComboBox()
        self.state_combo.addItems([
            "Perfect",
            "Almost perfect", 
            "Good",
            "Average",
            "Not good",
            "Really bad",
            "Destroyed"
        ])
        self.state_combo.setCurrentText("Good")  # Default selection
        form_layout.addRow("State:", self.state_combo)
        
        # Checkboxes for conditions
        self.dents_checkbox = QCheckBox("Has dents")
        form_layout.addRow("Dents:", self.dents_checkbox)
        
        self.corrosions_checkbox = QCheckBox("Has corrosions")
        form_layout.addRow("Corrosions:", self.corrosions_checkbox)
        
        self.scratches_checkbox = QCheckBox("Has scratches")
        form_layout.addRow("Scratches:", self.scratches_checkbox)
        
        # Details text area (optional)
        self.details_text = QTextEdit()
        self.details_text.setMaximumHeight(80)
        self.details_text.setPlaceholderText("Optional details about the inspection...")
        form_layout.addRow("Details:", self.details_text)
        
        # Inspection date (read-only, pre-filled)
        inspection_date = self.inspection_data.get('inspection_date', datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self.inspection_date_edit = QLineEdit(inspection_date)
        self.inspection_date_edit.setReadOnly(True)
        self.inspection_date_edit.setStyleSheet("background-color: #f0f0f0;")
        form_layout.addRow("Inspection Date:", self.inspection_date_edit)
        
        # FTP path (read-only, will be filled after FTP upload)
        self.ftp_path_edit = QLineEdit(self.inspection_data.get('inspection_path', ''))
        self.ftp_path_edit.setReadOnly(True)
        self.ftp_path_edit.setStyleSheet("background-color: #f0f0f0;")
        self.ftp_path_edit.setPlaceholderText("Upload to FTP first to get the path")
        form_layout.addRow("Inspection Path:", self.ftp_path_edit)
        
        main_layout.addLayout(form_layout)
        
        # Add custom buttons
        button_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("💾 Save to Database")
        self.save_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
        self.save_btn.clicked.connect(self.save_to_database)
        button_layout.addWidget(self.save_btn)
        
        self.cancel_btn = QPushButton("❌ Cancel")
        self.cancel_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #f44336; color: white; }")
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_btn)
        
        main_layout.addLayout(button_layout)
        
        # Set the custom widget as the message box's layout
        self.layout().addWidget(widget, 1, 0, 1, self.layout().columnCount())
        
        # Remove default buttons
        self.setStandardButtons(QMessageBox.NoButton)
    
    def update_ftp_path(self, ftp_path):
        """Update the FTP path field after successful upload"""
        self.ftp_path_edit.setText(ftp_path)
    
    def save_to_database(self):
        """Collect form data and prepare for database save"""
        # Validate required fields
        if not self.name_piece_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Piece name is required!")
            return
        
        if not self.ref_piece_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Piece reference is required!")
            return
            
        if not self.ftp_path_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Please upload to FTP first to get the inspection path!")
            return
        
        # Collect all form data
        self.result_data = {
            "name_piece": self.name_piece_edit.text().strip(),
            "ref_piece": self.ref_piece_edit.text().strip(),
            "name_program": self.program_name_edit.text().strip(),
            "state": self.state_combo.currentText(),
            "dents": self.dents_checkbox.isChecked(),
            "corrosions": self.corrosions_checkbox.isChecked(),
            "scratches": self.scratches_checkbox.isChecked(),
            "details": self.details_text.toPlainText().strip(),
            "inspection_date": self.inspection_date_edit.text(),
            "inspection_path": self.ftp_path_edit.text().strip()
        }
        
        self.accept()
    
    def get_result_data(self):
        """Get the collected form data"""
        return self.result_data