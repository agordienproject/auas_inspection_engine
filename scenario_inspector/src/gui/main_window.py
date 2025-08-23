"""
Main window for AUAS Inspection Engine.

This module now focuses on wiring and high-level GUI flow. Heavy components
like custom widgets, background execution threads, and dialogs live in
dedicated modules under gui/widgets, gui/dialogs, etc.
"""
import os
import logging
from datetime import datetime
import yaml
from typing import Dict, Any
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QProgressBar,
    QGroupBox, QGridLayout, QMessageBox, QFileDialog,
    QLineEdit, QListWidget, QTabWidget, QFrame, QSpacerItem,
    QSizePolicy, QFormLayout, QSpinBox, QCheckBox,
    QApplication
)
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtGui import QFont


# Local, lighter modules
from gui.widgets.nowheel_spinbox import NoWheelSpinBox
from gui.execution_thread import ProgramExecutionThread
from gui.dialogs.inspection_database_dialog import InspectionDatabaseDialog
from gui.tabs.piece_tab import build_piece_info_tab
from gui.tabs.program_tab import build_program_tab
from gui.tabs.execution_tab import build_execution_tab
from gui.tabs.status_tab import build_status_tab
from gui.tabs.settings_tab import build_settings_tab

from config.config_manager import ConfigManager
from database.models import User
from systems.system_manager import SystemManager
from auth.login import LoginDialog
from utils.ftp_manager import FTPManager


class InspectionMainWindow(QMainWindow):
    """Main window for the AUAS Inspection Engine"""

    def __init__(self, user: User = None):
        """Initialize the main application window.

        Args:
            user: Optional authenticated user; when None the app runs in guest
                mode and disables database-saving features.
        """
        super().__init__()
        self.user = user  # Can be None for guest mode
        self.config_manager = ConfigManager()
        self.system_manager = SystemManager(self.config_manager)
        self.logger = logging.getLogger(__name__)

        # Runtime state
        self.execution_thread = None
        self.current_program = None
        self.piece_info = {}

        # Predefine commonly used attributes
        self.status_labels = {}
        self.status_info_labels = {}
        self.status_detail_labels = {}
        self.last_piece_name = None
        self.last_ref_piece = None
        self.last_program_name = None
        self.last_inspection_folder = None
        self.last_inspection_date = None
        self.last_ftp_path = None
        self.continue_stage_btn = None

        # Initialize UI and load programs
        self.logger.debug("Initializing main window UI")
        self.setup_ui()
        self.logger.debug("Loading available programs after UI setup")
        self.load_available_programs()

        # NOTE: setup_ui is defined later with enhanced styling; only one version retained

    def create_header(self, layout):
        """Create and insert the header section.

        Args:
            layout: Parent layout to which the header frame is added.
        """
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
            self.logger.debug("Creating header for authenticated user: %s", self.user.pseudo)
            user_label = QLabel(f"User: {self.user.pseudo}")
            header_layout.addWidget(user_label)

            # Logout button
            logout_btn = QPushButton("Logout")
            logout_btn.clicked.connect(self.logout)
            header_layout.addWidget(logout_btn)
        else:
            # Guest mode
            self.logger.debug("Creating header for guest mode")
            guest_label = QLabel("Guest Mode")
            guest_label.setStyleSheet("color: orange; font-weight: bold;")
            header_layout.addWidget(guest_label)

            # Login button
            login_btn = QPushButton("Login")
            login_btn.clicked.connect(self.show_login)
            header_layout.addWidget(login_btn)

        layout.addWidget(header_frame)

    def create_piece_info_tab(self):
        """Create the piece information tab via builder."""
        piece_widget = build_piece_info_tab(self)
        self.tab_widget.addTab(piece_widget, "Piece Info")

    def create_program_tab(self):
        """Create the program selection tab via builder."""
        program_widget = build_program_tab(self)
        self.tab_widget.addTab(program_widget, "Programs")

    def create_execution_tab(self):
        """Create the execution control tab via builder."""
        exec_widget = build_execution_tab(self)
        self.tab_widget.addTab(exec_widget, "Execution")

    def create_status_tab(self):
        """Create the system status tab via builder."""
        status_widget = build_status_tab(self)
        self.tab_widget.addTab(status_widget, "System Status")

    def create_settings_tab(self):
        """Create the settings configuration tab via builder."""
        settings_widget = build_settings_tab(self)
        self.tab_widget.addTab(settings_widget, "Settings")

    def save_settings(self):
        """Save the current settings to app_config.yaml"""
        try:
            self.logger.debug("Saving settings initiated")
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
                for key in system_config.keys():
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
            # Programs path (top-level key)
            config['programs_path'] = self.settings_widgets['programs_path'].text()

            # Security settings
            if 'security' not in config:
                config['security'] = {}
            config['security']['bcrypt_rounds'] = self.settings_widgets['security_bcrypt_rounds'].value()
            config['security']['session_timeout'] = self.settings_widgets['security_session_timeout'].value()

            # Save to file with preserved formatting
            config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'config', 'app_config.yaml')

            # Read the original file as text to preserve formatting
            with open(config_path, 'r', encoding='utf-8') as f:
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
                for key in system_config.keys():
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
            # Programs path (top-level key, not in a section)
            def update_top_level_yaml_line_after_section(lines, key, new_value, after_section='systems'):
                # Remove all existing top-level programs_path lines
                new_lines = []
                for line in lines:
                    if not (line.strip().startswith(f'{key}:') and (line.startswith('programs_path') or line.lstrip() == line)):
                        new_lines.append(line)
                # Find where to insert: after the section (e.g. systems: ...)
                insert_idx = None
                for i, line in enumerate(new_lines):
                    if line.strip().startswith(f'{after_section}:'):
                        # Find the end of the section (next non-indented line or end of file)
                        for j in range(i+1, len(new_lines)):
                            if new_lines[j].strip() == '' or (not new_lines[j].startswith(' ') and not new_lines[j].startswith('\t')):
                                insert_idx = j
                                break
                        else:
                            insert_idx = len(new_lines)
                        break
                if insert_idx is None:
                    insert_idx = len(new_lines)
                # Format the new value
                if isinstance(new_value, str):
                    if '\\' in new_value and not new_value.startswith(('ftp://', 'http://', 'https://')):
                        escaped_value = new_value.replace('\\', '\\\\')
                        formatted_value = f'"{escaped_value}"'
                    else:
                        formatted_value = f'"{new_value}"'
                elif isinstance(new_value, bool):
                    formatted_value = str(new_value).lower()
                else:
                    formatted_value = str(new_value)
                new_lines.insert(insert_idx, f'{key}: {formatted_value}\n')
                return new_lines
            updated_lines = update_top_level_yaml_line_after_section(updated_lines, 'programs_path', self.settings_widgets['programs_path'].text(), after_section='systems')

            # Security settings
            updated_lines = update_yaml_line(updated_lines, 'security', 'bcrypt_rounds', self.settings_widgets['security_bcrypt_rounds'].value())
            updated_lines = update_yaml_line(updated_lines, 'security', 'session_timeout', self.settings_widgets['security_session_timeout'].value())

            # Write the updated content back
            with open(config_path, 'w', encoding='utf-8') as f:
                f.writelines(updated_lines)

            self.logger.info("Settings saved to app_config.yaml")
            QMessageBox.information(self, "Settings Saved", "Settings have been saved successfully!")

        except Exception as e:
            self.logger.exception("Failed to save settings")
            QMessageBox.critical(self, "Error Saving Settings", f"Failed to save settings: {str(e)}")

    def save_and_restart(self):
        """Save settings and restart the application"""
        self.logger.info("Save-and-restart requested by user")
        self.save_settings()

        # Ask for confirmation
        reply = QMessageBox.question(
            self,
            "Restart Application",
            "Settings saved. Do you want to restart the application now?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            # Restart the application
            import sys
            self.logger.info("Restarting application...")
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
        self.logger.debug("Refreshing system status")
        self.log_status_message("Refreshing system status...")
        try:
            systems_status = self.system_manager.get_all_systems_status()
            for system_name, status_info in systems_status.items():
                if system_name in self.status_labels:
                    self._update_system_status_display(system_name, status_info)
            self.log_status_message("[OK] System status refresh completed")
        except Exception as e:
            self.logger.exception("Error refreshing system status")
            self.log_status_message(f"❌ Error refreshing status: {e}")

    def test_all_connections(self):
        """Test all system connections individually"""
        self.logger.info("Testing all system connections")
        self.log_status_message("🔍 Testing all system connections...")

        try:
            systems_config = self.config_manager.get_systems_config()

            for system_name in systems_config.keys():
                self.logger.debug("Testing connection to system: %s", system_name)
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

                # Add small delay to show testing process
                QThread.msleep(500)

            self.log_status_message("[OK] All connection tests completed")

        except Exception as e:
            self.logger.exception("Error testing connections")
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
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_log.append(f"[{timestamp}] {message}")

        # Auto-scroll to bottom
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


    def load_available_programs(self):
        """Load available programs from the programs directory"""
        try:
            programs_dir = self.config_manager.get_config().get('programs_path', 'programs')
            self.logger.debug("Loading programs from path: %s", programs_dir)
            if not os.path.isabs(programs_dir):
                scenario_inspector_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                programs_dir = os.path.abspath(os.path.join(scenario_inspector_root, programs_dir))
            self.logger.debug("Resolved programs absolute path: %s", programs_dir)
            if os.path.exists(programs_dir):
                count = 0
                for file in os.listdir(programs_dir):
                    if file.endswith('.yaml') or file.endswith('.yml'):
                        self.programs_list.addItem(file)
                        count += 1
                self.logger.info("Loaded %d program(s)", count)
            else:
                QMessageBox.warning(self, "Warning", f"Programs directory does not exist:\n{programs_dir}")
        except Exception as e:
            self.logger.exception("Error loading programs")
            QMessageBox.warning(self, "Warning", f"Could not load programs: {e}")

    def save_piece_info(self):
        """Save piece information"""
        self.logger.debug(
            "Saving piece info: name=%s ref=%s",
            self.piece_name_edit.text(),
            self.piece_ref_edit.text(),
        )
        self.piece_info = {
            'name_piece': self.piece_name_edit.text(),
            'ref_piece': self.piece_ref_edit.text()
        }

        if not self.piece_info['name_piece'] or not self.piece_info['ref_piece']:
            self.logger.warning("Piece info incomplete; prompting user")
            QMessageBox.warning(self, "Warning", "Please fill in all piece information fields.")
            return

        self.logger.info("Piece information saved")
        QMessageBox.information(self, "Success", "Piece information saved successfully!")
        self.update_execution_button_state()

    def select_program(self):
        """Select a program from the list"""
        current_item = self.programs_list.currentItem()
        if not current_item:
            self.logger.warning("Select program requested with no selection")
            QMessageBox.warning(self, "Warning", "Please select a program from the list.")
            return

        program_file = current_item.text()
        self.logger.info("Program selected: %s", program_file)
        self.load_program(program_file)

    def upload_program(self):
        """Upload a new program file"""
        self.logger.info("Upload program dialog opened")
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
                programs_dir = self.config_manager.get_config().get('programs_path', 'programs')
                if not os.path.isabs(programs_dir):
                    scenario_inspector_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                    programs_dir = os.path.abspath(os.path.join(scenario_inspector_root, programs_dir))
                file_name = os.path.basename(file_path)
                dest_path = os.path.join(programs_dir, file_name)
                shutil.copy2(file_path, dest_path)
                self.logger.info("Uploaded program copied to %s", dest_path)

                # Refresh programs list
                self.programs_list.clear()
                self.load_available_programs()

                # Load the uploaded program
                self.load_program(file_name)

                QMessageBox.information(self, "Success", f"Program {file_name} uploaded successfully!")

            except Exception as e:
                self.logger.exception("Error uploading program")
                QMessageBox.critical(self, "Error", f"Failed to upload program: {e}")

    def load_program(self, program_file):
        """Load a program file"""
        try:
            self.logger.debug("Loading program file: %s", program_file)
            programs_dir = self.config_manager.get_config().get('programs_path', 'programs')
            if not os.path.isabs(programs_dir):
                scenario_inspector_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                programs_dir = os.path.abspath(os.path.join(scenario_inspector_root, programs_dir))
            program_path = os.path.join(programs_dir, program_file)
            with open(program_path, 'r', encoding='utf-8') as file:
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

            self.logger.info("Program loaded: %s", program_file)
            QMessageBox.information(self, "Success", f"Program {program_file} loaded successfully!")
            self.update_execution_button_state()

        except Exception as e:
            self.logger.exception("Error loading program")
            QMessageBox.critical(self, "Error", f"Failed to load program: {e}")

    def update_execution_button_state(self):
        """Update the state of execution buttons based on current state"""
        can_execute = (
            self.current_program is not None
            and bool(self.piece_info.get('name_piece'))
            and bool(self.piece_info.get('ref_piece'))
        )
        self.logger.debug(
            "Execution button state -> %s",
            "ENABLED" if can_execute else "DISABLED",
        )
        self.start_execution_btn.setEnabled(can_execute)

    def start_execution(self):
        """Start program execution"""
        if not self.current_program or not self.piece_info:
            self.logger.warning("Start execution blocked: missing program or piece info")
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
        self.logger.info(
            "Starting execution thread for program '%s' and piece '%s'/%s",
            self.current_program.get('program', {}).get('name', 'Unknown'),
            self.piece_info.get('name_piece', ''),
            self.piece_info.get('ref_piece', '')
        )
        self.execution_thread = ProgramExecutionThread(
            self.current_program,
            self.piece_info,
            self.config_manager,
        )

        # Connect signals
        self.execution_thread.progress_updated.connect(self.update_progress)
        self.execution_thread.execution_finished.connect(self.execution_finished)
        self.execution_thread.step_completed.connect(self.step_completed)
        self.execution_thread.progress_percentage.connect(self.update_progress_bar)
        self.execution_thread.pause_signal.connect(self._on_stage_pause)

        # Start thread
        self.execution_thread.start()
        self.logger.debug("Execution thread started")

    def _on_stage_pause(self, stage_name):
        """Handle pause requested by the engine after a stage.

        Shows a Continue button and logs the pause message.

        Args:
            stage_name: The name of the stage that just completed.
        """
        # Show continue button and update progress
        if not hasattr(self, 'continue_stage_btn'):
            self.continue_stage_btn = QPushButton("Continue Next Stage")
            self.continue_stage_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #FFC107; color: #222; }")
            self.continue_stage_btn.setVisible(False)
            self.continue_stage_btn.clicked.connect(self._on_continue_stage)
            # Add to execution tab layout
            exec_tab = self.tab_widget.widget(2)  # Assuming execution tab is at index 2
            exec_layout = exec_tab.layout()
            exec_layout.addWidget(self.continue_stage_btn)
        self.update_progress(f"Paused after stage '{stage_name}'. Waiting for user to continue...")
        self.continue_stage_btn.setVisible(True)
        self.continue_stage_btn.setEnabled(True)

    def _on_continue_stage(self):
        """Resume execution after a stage pause when user clicks Continue."""
        if self.execution_thread:
            self.logger.info("User requested to continue after stage pause")
            self.execution_thread.resume_from_pause()
        if hasattr(self, 'continue_stage_btn'):
            self.continue_stage_btn.setVisible(False)

    def setup_ui(self):
        """Setup the user interface"""
        self.logger.debug("Setting up UI components and styles")
        # Set window properties
        gui_config = self.config_manager.get_gui_config()
        self.setWindowTitle(gui_config.get('window_title', 'AUAS Inspection Engine'))
        self.setGeometry(100, 100, 1200, 800)

        # Apply modern light theme and style
        self.setStyleSheet('''
            QWidget {
                background-color: #F7F7FA;
                color: #222;
                font-family: "Segoe UI", "Arial", "sans-serif";
                font-size: 15px;
            }
            QGroupBox {
                border: 1.5px solid #D0D0D0;
                border-radius: 12px;
                margin-top: 16px;
                background-color: #FFFFFF;
                padding: 12px;
            }
            QGroupBox:title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 6px 0 6px;
                color: #1976D2;
                font-weight: bold;
                font-size: 17px;
            }
            QLabel {
                color: #333;
                font-size: 15px;
                background: transparent;
            }
            QLineEdit, QTextEdit, QComboBox {
                background: #FAFAFA;
                border: 1.5px solid #D0D0D0;
                border-radius: 8px;
                padding: 7px 10px;
                color: #222;
                font-size: 15px;
            }
            QLineEdit:focus, QTextEdit:focus, QComboBox:focus {
                border: 1.5px solid #1976D2;
                background: #FFFFFF;
            }
            QPushButton {
                background-color: #1976D2;
                color: #fff;
                border: none;
                border-radius: 8px;
                padding: 10px 18px;
                font-size: 15px;
                font-weight: 600;
                margin: 8px 0;
                outline: none;
            }
            QPushButton:hover {
                background-color: #1565C0;
                color: #fff;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
                color: #fff;
            }
            QPushButton:focus {
                border: 2px solid #1976D2;
            }
            QTabBar::tab {
                background: #E3E7EF;
                color: #222;
                border: 1px solid #D0D0D0;
                border-bottom: none;
                border-radius: 8px 8px 0 0;
                padding: 8px 20px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #FFFFFF;
                color: #1976D2;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #D0D0D0;
                border-radius: 0 0 8px 8px;
                top: -1px;
            }
            QCheckBox {
                spacing: 8px;
                font-size: 15px;
            }
            QCheckBox::indicator:checked {
                background-color: #1976D2;
                border: 1.5px solid #1565C0;
            }
            QScrollArea {
                border: none;
                background: #F7F7FA;
            }
            QFormLayout > QLabel {
                min-width: 120px;
            }
        ''')

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

    def stop_execution(self):
        """Stop program execution"""
        if self.execution_thread and self.execution_thread.isRunning():
            self.logger.info("Stopping execution thread on user request")
            self.execution_thread.terminate()
            self.execution_thread.wait()

        self.execution_finished(False)
        self.update_progress("Execution stopped by user.")

    def update_progress(self, message):
        """Update progress display"""
        self.logger.debug("Progress: %s", message)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.progress_text.append(f"[{timestamp}] {message}")

    def update_progress_bar(self, percentage):
        """Update progress bar percentage"""
        self.logger.debug("Progress bar -> %d%%", percentage)
        self.progress_bar.setValue(percentage)

    def step_completed(self, step_name):
        """Handle step completion"""
        self.logger.info("Step completed: %s", step_name)
        self.update_progress(f"[OK] Step completed: {step_name}")

    def execution_finished(self, success, inspection_folder="", inspection_date=None):
        """Handle execution completion"""
        self.logger.info("Execution finished. success=%s folder=%s", success, inspection_folder)
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
            self.logger.debug("Post-execution actions enabled; folder=%s", inspection_folder)
        else:
            # Reset if no valid folder
            self.last_inspection_folder = None
            self.last_inspection_date = None
            self.last_inspection_path_label.setText("No inspection completed yet")
            self.last_inspection_path_label.setStyleSheet("font-style: italic; color: gray;")
            self.ftp_upload_btn.setEnabled(False)
            self.db_save_btn.setEnabled(False)
            self.open_folder_btn.setEnabled(False)
            self.logger.debug("No valid inspection folder; actions disabled")

        if success:
            self.update_progress("[OK] Program execution completed successfully!")
            QMessageBox.information(self, "Success", "Inspection program completed successfully!")
        else:
            self.update_progress("[ERROR] Program execution failed or was stopped.")

    def show_login(self):
        """Show login dialog and authenticate user"""
        try:
            login_dialog = LoginDialog()
            if login_dialog.exec_() == LoginDialog.Accepted:
                # Get authenticated user
                authenticated_user = login_dialog.get_authenticated_user()
                if authenticated_user:
                    self.user = authenticated_user
                    self.logger.info("User %s logged in successfully", authenticated_user.pseudo)

                    # Update UI to reflect logged in state
                    self.update_ui_after_login()

                    # Update status bar
                    self.statusBar().showMessage(f"Logged in as: {self.user.pseudo}")
                else:
                    self.logger.warning("Login accepted but no authenticated user returned")
                    QMessageBox.warning(self, "Login Failed", "Authentication failed. Please try again.")
        except Exception as e:
            self.logger.exception("Login error")
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
                self.logger.info("User %s confirmed logout", self.user.pseudo)
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
                self.logger.info("User confirmed application close from guest mode")
                self.close()

    def update_api_status_display(self):
        """Update the API status display"""
        if hasattr(self, 'api_status_label'):
            if self.user and hasattr(self.user, 'api_connection') and self.user.api_connection:
                if self.user.api_connection.is_authenticated():
                    self.logger.debug("API connection active for user %s", self.user.pseudo)
                    self.api_status_label.setText(f"🔐 Logged in as: {self.user.pseudo} | API connection: ACTIVE")
                    self.api_status_label.setStyleSheet("color: green; font-weight: bold;")
                else:
                    self.logger.debug("API token expired for user %s", self.user.pseudo)
                    self.api_status_label.setText(f"⚠️ User: {self.user.pseudo} | API connection: EXPIRED")
                    self.api_status_label.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.logger.debug("Guest mode or no API connection; API status disabled")
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
            self.logger.debug("Testing FTP connection: host=%s base=%s passive=%s", ftp_config['server'], ftp_config['base_path'], ftp_config['passive_mode'])

            # Create FTP manager and test connection
            ftp_manager = FTPManager(ftp_config)
            result = ftp_manager.test_connection()

            if result['success']:
                self.logger.info("FTP connection test succeeded")
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
                self.logger.warning("FTP connection test failed: %s", result.get('message'))
                QMessageBox.warning(
                    self,
                    "FTP Connection Test Failed",
                    f"❌ {result['message']}\n\nPlease check your FTP settings and ensure the server is accessible."
                )

        except Exception as e:
            self.logger.exception("FTP connection test error")
            QMessageBox.critical(
                self,
                "FTP Test Error",
                f"❌ Error testing FTP connection:\n{str(e)}"
            )

    def upload_inspection_to_ftp(self):
        """Upload the last inspection folder to FTP server"""
        if not hasattr(self, 'last_inspection_folder') or not self.last_inspection_folder:
            self.logger.warning("FTP upload requested but no last inspection folder available")
            QMessageBox.warning(self, "No Inspection", "No inspection folder available for upload.")
            return

        if not os.path.exists(self.last_inspection_folder):
            self.logger.warning("FTP upload path missing: %s", self.last_inspection_folder)
            QMessageBox.warning(self, "Folder Not Found", f"Inspection folder not found:\n{self.last_inspection_folder}")
            return

        try:
            # Get FTP configuration
            ftp_config = self.config_manager.get_config().get('ftp', {})

            if not ftp_config.get('server'):
                self.logger.warning("FTP upload attempted without FTP configuration")
                QMessageBox.warning(self, "FTP Not Configured", "FTP server is not configured. Please check your settings.")
                return

            # Disable button during upload
            self.ftp_upload_btn.setEnabled(False)
            self.ftp_upload_btn.setText("📤 Uploading...")
            self.logger.info("Uploading inspection folder to FTP: %s", self.last_inspection_folder)

            # Create FTP manager and upload
            ftp_manager = FTPManager(ftp_config)

            # Use the inspection date if available, otherwise current date
            inspection_date = getattr(self, 'last_inspection_date', datetime.now())

            result = ftp_manager.upload_inspection_folder(self.last_inspection_folder, inspection_date)

            if result['success']:
                # Store FTP path for database save
                self.last_ftp_path = result['details']['ftp_path']
                self.logger.info("FTP upload successful -> %s (files: %d)", self.last_ftp_path, len(result['details']['uploaded_files']))

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
                self.logger.warning("FTP upload failed: %s%s", result.get('message'), f"; failed_files={len(result['details'].get('failed_files', []))}" if 'details' in result else "")

                QMessageBox.warning(
                    self,
                    "Upload Failed",
                    f"❌ {result['message']}{error_details}"
                )

        except Exception as e:
            self.logger.exception("FTP upload error")
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
            self.logger.warning("Database save requested but no inspection folder available")
            QMessageBox.warning(self, "No Inspection", "No inspection data available to save.")
            return

        # Check if user is logged in
        if not self.user:
            self.logger.warning("Database save attempted in guest mode")
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
            self.logger.debug("Opening InspectionDatabaseDialog with data: %s", inspection_data)

            # Create and show the database dialog
            dialog = InspectionDatabaseDialog(inspection_data, self)

            # If user has uploaded to FTP, get the path
            if hasattr(self, 'last_ftp_path') and self.last_ftp_path:
                dialog.update_ftp_path(self.last_ftp_path)

            if dialog.exec_() == QMessageBox.Accepted:
                form_data = dialog.get_result_data()
                if form_data:
                    self.logger.info("Submitting inspection data to API")
                    self._send_to_database(form_data)

        except Exception as e:
            self.logger.exception("Error preparing database save")
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
            self.logger.debug("Preparing payload for API save")

            # Prepare the JSON payload
            # Convert datetime to ISO format for API
            inspection_date = inspection_data['inspection_date']
            if isinstance(inspection_date, str):
                # Parse the date string and convert to ISO format
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
            self.logger.debug("Payload ready: %s", payload)

            # Use the authenticated API connection instead of creating a new session
            if hasattr(self.user, 'api_connection') and self.user.api_connection:
                # Use the authenticated API connection
                api_result = self.user.api_connection.save_inspection_data(payload)

                if api_result.get('success', False):
                    self.logger.info("Inspection data saved to database")
                    QMessageBox.information(
                        self,
                        "Success",
                        "✅ Inspection data saved to database successfully!"
                    )
                    self.update_progress("[OK] Inspection data saved to database")
                else:
                    error_msg = api_result.get('message', 'Unknown error')
                    self.logger.warning("Database save failed: %s", error_msg)
                    QMessageBox.critical(
                        self,
                        "Database Save Failed",
                        f"❌ Failed to save to database:\n{error_msg}"
                    )
            else:
                self.logger.error("No authenticated API connection available")
                QMessageBox.critical(
                    self,
                    "Authentication Error",
                    "❌ No authenticated API connection available. Please login again."
                )

        except Exception as e:
            self.logger.exception("Database save error")
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
            self.logger.warning("Open folder requested but no last inspection folder is set")
            QMessageBox.warning(self, "No Inspection", "No inspection folder available.")
            return

        if not os.path.exists(self.last_inspection_folder):
            self.logger.warning("Open folder requested but path does not exist: %s", self.last_inspection_folder)
            QMessageBox.warning(self, "Folder Not Found", f"Inspection folder not found:\n{self.last_inspection_folder}")
            return

        try:
            # Open folder in Windows Explorer
            self.logger.info("Opening inspection folder: %s", self.last_inspection_folder)
            os.startfile(self.last_inspection_folder)
        except Exception as e:
            self.logger.exception("Failed to open inspection folder")
            QMessageBox.critical(self, "Error", f"Failed to open folder:\n{str(e)}")

    def update_database_status_display(self):
        """Update the database status display (legacy, now shows API status)"""
        self.update_api_status_display()

    def update_ui_after_login(self):
        """Update UI elements after successful login"""
        self.logger.debug("Updating UI after login for user %s", self.user.pseudo)
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
        self.logger.debug("Updating UI after logout (guest mode)")
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
        self.logger.debug("Rebuilding header frame for current auth state")
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
        self.logger.debug("Window close requested")
        if self.execution_thread and self.execution_thread.isRunning():
            reply = QMessageBox.question(
                self,
                "Close Application",
                "An inspection is currently running. Do you want to stop it and exit?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )

            if reply == QMessageBox.Yes:
                self.logger.info("Closing app with running execution: stopping thread")
                self.stop_execution()
                event.accept()
            else:
                self.logger.debug("Close canceled by user")
                event.ignore()
        else:
            self.logger.debug("No execution running; closing app")
            event.accept()
