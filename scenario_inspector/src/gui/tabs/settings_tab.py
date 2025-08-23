"""
Settings tab for configuring the application.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QGroupBox, QGridLayout, QLineEdit, QComboBox, QCheckBox
)
from gui.widgets.nowheel_spinbox import NoWheelSpinBox


def build_settings_tab(window) -> QWidget:
    """Build the Settings tab. Reads values from window.config_manager and
    creates input widgets stored in window.settings_widgets.
    """
    settings_widget = QWidget()
    layout = QVBoxLayout(settings_widget)

    header_layout = QHBoxLayout()
    title_label = QLabel("Application Settings")
    title_label.setStyleSheet("QLabel { font-size: 16px; font-weight: bold; }")
    header_layout.addWidget(title_label)

    header_layout.addStretch(1)

    save_btn = QPushButton("💾 Save Settings")
    save_btn.clicked.connect(window.save_settings)
    save_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
    header_layout.addWidget(save_btn)

    restart_btn = QPushButton("🔄 Save & Restart")
    restart_btn.clicked.connect(window.save_and_restart)
    restart_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #FF9800; color: white; }")
    header_layout.addWidget(restart_btn)

    layout.addLayout(header_layout)

    scroll_area = QScrollArea()
    scroll_widget = QWidget()
    scroll_layout = QVBoxLayout(scroll_widget)

    window.settings_widgets = {}

    # API configuration
    api_group = QGroupBox("API Configuration")
    api_layout = QGridLayout(api_group)
    api_config = window.config_manager.get_api_config()

    api_layout.addWidget(QLabel("API URL:"), 0, 0)
    window.settings_widgets['api_url'] = QLineEdit(str(api_config.get('url', '127.0.0.1:3000/api')))
    api_layout.addWidget(window.settings_widgets['api_url'], 0, 1)

    api_layout.addWidget(QLabel("Login Endpoint:"), 1, 0)
    window.settings_widgets['api_login_endpoint'] = QLineEdit(str(api_config.get('login_endpoint', '/auth/login')))
    api_layout.addWidget(window.settings_widgets['api_login_endpoint'], 1, 1)

    api_layout.addWidget(QLabel("Inspection Endpoint:"), 2, 0)
    window.settings_widgets['api_inspection_endpoint'] = QLineEdit(str(api_config.get('inspection_endpoint', '/inspections')))
    api_layout.addWidget(window.settings_widgets['api_inspection_endpoint'], 2, 1)

    api_layout.addWidget(QLabel("Timeout (s):"), 3, 0)
    window.settings_widgets['api_timeout'] = NoWheelSpinBox()
    window.settings_widgets['api_timeout'].setRange(5, 300)
    window.settings_widgets['api_timeout'].setValue(int(api_config.get('timeout', 30)))
    api_layout.addWidget(window.settings_widgets['api_timeout'], 3, 1)

    window.settings_widgets['api_use_https'] = QCheckBox("Use HTTPS")
    window.settings_widgets['api_use_https'].setChecked(bool(api_config.get('use_https', False)))
    api_layout.addWidget(window.settings_widgets['api_use_https'], 4, 0, 1, 2)

    scroll_layout.addWidget(api_group)

    # Legacy DB (disabled)
    db_group = QGroupBox("Legacy Database Configuration (API is now used)")
    db_group.setEnabled(False)
    db_layout = QGridLayout(db_group)
    db_config = window.config_manager.get_database_config()

    db_layout.addWidget(QLabel("Host:"), 0, 0)
    window.settings_widgets['database_host'] = QLineEdit(str(db_config.get('host', 'localhost')))
    db_layout.addWidget(window.settings_widgets['database_host'], 0, 1)

    db_layout.addWidget(QLabel("Port:"), 1, 0)
    window.settings_widgets['database_port'] = NoWheelSpinBox()
    window.settings_widgets['database_port'].setRange(1, 65535)
    window.settings_widgets['database_port'].setValue(int(db_config.get('port', 5432)))
    db_layout.addWidget(window.settings_widgets['database_port'], 1, 1)

    db_layout.addWidget(QLabel("Username:"), 2, 0)
    window.settings_widgets['database_username'] = QLineEdit(str(db_config.get('username', 'postgres')))
    db_layout.addWidget(window.settings_widgets['database_username'], 2, 1)

    db_layout.addWidget(QLabel("Password:"), 3, 0)
    window.settings_widgets['database_password'] = QLineEdit(str(db_config.get('password', '')))
    window.settings_widgets['database_password'].setEchoMode(QLineEdit.Password)
    db_layout.addWidget(window.settings_widgets['database_password'], 3, 1)

    db_layout.addWidget(QLabel("Database:"), 4, 0)
    window.settings_widgets['database_database'] = QLineEdit(str(db_config.get('database', 'lab_inspection')))
    db_layout.addWidget(window.settings_widgets['database_database'], 4, 1)

    scroll_layout.addWidget(db_group)

    # Systems configuration
    systems_group = QGroupBox("Systems Configuration")
    systems_layout = QVBoxLayout(systems_group)
    systems_config = window.config_manager.get_systems_config()

    for system_name, system_config in systems_config.items():
        system_subgroup = QGroupBox(f"{system_name.title()} System")
        system_sublayout = QGridLayout(system_subgroup)
        row = 0

        system_sublayout.addWidget(QLabel("Connection Type:"), row, 0)
        window.settings_widgets[f'systems_{system_name}_connection_type'] = QLineEdit(str(system_config.get('connection_type', '')))
        system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_connection_type'], row, 1)
        row += 1

        if system_config.get('connection_type') == 'tcp':
            system_sublayout.addWidget(QLabel("IP Address:"), row, 0)
            window.settings_widgets[f'systems_{system_name}_ip'] = QLineEdit(str(system_config.get('ip', '')))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_ip'], row, 1)
            row += 1
            if 'port' in system_config:
                system_sublayout.addWidget(QLabel("Port:"), row, 0)
                window.settings_widgets[f'systems_{system_name}_port'] = NoWheelSpinBox()
                window.settings_widgets[f'systems_{system_name}_port'].setRange(1, 65535)
                window.settings_widgets[f'systems_{system_name}_port'].setValue(int(system_config.get('port', 8080)))
                system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_port'], row, 1)
                row += 1

        if system_config.get('connection_type') == 'serial':
            system_sublayout.addWidget(QLabel("Port:"), row, 0)
            window.settings_widgets[f'systems_{system_name}_port'] = QLineEdit(str(system_config.get('port', '')))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_port'], row, 1)
            row += 1
            if 'baudrate' in system_config:
                system_sublayout.addWidget(QLabel("Baudrate:"), row, 0)
                window.settings_widgets[f'systems_{system_name}_baudrate'] = NoWheelSpinBox()
                window.settings_widgets[f'systems_{system_name}_baudrate'].setRange(300, 115200)
                window.settings_widgets[f'systems_{system_name}_baudrate'].setValue(int(system_config.get('baudrate', 9600)))
                system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_baudrate'], row, 1)
                row += 1

        if system_config.get('connection_type') == 'usb':
            system_sublayout.addWidget(QLabel("Device ID:"), row, 0)
            window.settings_widgets[f'systems_{system_name}_device_id'] = QLineEdit(str(system_config.get('device_id', '')))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_device_id'], row, 1)
            row += 1

        system_sublayout.addWidget(QLabel("Timeout (s):"), row, 0)
        window.settings_widgets[f'systems_{system_name}_timeout'] = NoWheelSpinBox()
        window.settings_widgets[f'systems_{system_name}_timeout'].setRange(1, 300)
        window.settings_widgets[f'systems_{system_name}_timeout'].setValue(int(system_config.get('timeout', 30)))
        system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_timeout'], row, 1)
        row += 1

        if 'programs_path' in system_config:
            system_sublayout.addWidget(QLabel("Programs Path:"), row, 0)
            window.settings_widgets[f'systems_{system_name}_programs_path'] = QLineEdit(str(system_config.get('programs_path', '')))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_programs_path'], row, 1)
            row += 1

        if 'llt_path' in system_config:
            system_sublayout.addWidget(QLabel("LLT SDK Path:"), row, 0)
            window.settings_widgets[f'systems_{system_name}_llt_path'] = QLineEdit(str(system_config.get('llt_path', '')))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_llt_path'], row, 1)
            row += 1

        if 'use_realsense_sdk' in system_config:
            window.settings_widgets[f'systems_{system_name}_use_realsense_sdk'] = QCheckBox("Use RealSense SDK")
            window.settings_widgets[f'systems_{system_name}_use_realsense_sdk'].setChecked(bool(system_config.get('use_realsense_sdk', False)))
            system_sublayout.addWidget(window.settings_widgets[f'systems_{system_name}_use_realsense_sdk'], row, 0, 1, 2)
            row += 1

        systems_layout.addWidget(system_subgroup)

    scroll_layout.addWidget(systems_group)

    # Output configuration
    output_group = QGroupBox("Output Configuration")
    output_layout = QGridLayout(output_group)
    output_config = window.config_manager.get_config().get('output', {})

    output_layout.addWidget(QLabel("Base Directory:"), 0, 0)
    window.settings_widgets['output_base_directory'] = QLineEdit(str(output_config.get('base_directory', 'output')))
    output_layout.addWidget(window.settings_widgets['output_base_directory'], 0, 1)

    output_layout.addWidget(QLabel("Inspection Prefix:"), 1, 0)
    window.settings_widgets['output_inspection_folder_prefix'] = QLineEdit(str(output_config.get('inspection_folder_prefix', 'inspection_')))
    output_layout.addWidget(window.settings_widgets['output_inspection_folder_prefix'], 1, 1)

    output_layout.addWidget(QLabel("Date Format:"), 2, 0)
    window.settings_widgets['output_date_format'] = QLineEdit(str(output_config.get('date_format', '%Y-%m-%d')))
    output_layout.addWidget(window.settings_widgets['output_date_format'], 2, 1)

    output_layout.addWidget(QLabel("Time Format:"), 3, 0)
    window.settings_widgets['output_time_format'] = QLineEdit(str(output_config.get('time_format', '%Y%m%d_%H%M%S')))
    output_layout.addWidget(window.settings_widgets['output_time_format'], 3, 1)

    scroll_layout.addWidget(output_group)

    # FTP Settings
    ftp_group = QGroupBox("FTP Configuration")
    ftp_layout = QGridLayout(ftp_group)
    ftp_config = window.config_manager.get_config().get('ftp', {})

    ftp_layout.addWidget(QLabel("FTP Server:"), 0, 0)
    window.settings_widgets['ftp_server'] = QLineEdit(str(ftp_config.get('server', 'ftp://127.0.0.1')))
    ftp_layout.addWidget(window.settings_widgets['ftp_server'], 0, 1)

    ftp_layout.addWidget(QLabel("Username:"), 1, 0)
    window.settings_widgets['ftp_username'] = QLineEdit(str(ftp_config.get('username', 'anonymous')))
    ftp_layout.addWidget(window.settings_widgets['ftp_username'], 1, 1)

    ftp_layout.addWidget(QLabel("Password:"), 2, 0)
    window.settings_widgets['ftp_password'] = QLineEdit(str(ftp_config.get('password', '')))
    window.settings_widgets['ftp_password'].setEchoMode(QLineEdit.Password)
    ftp_layout.addWidget(window.settings_widgets['ftp_password'], 2, 1)

    ftp_layout.addWidget(QLabel("Base Path:"), 3, 0)
    window.settings_widgets['ftp_base_path'] = QLineEdit(str(ftp_config.get('base_path', '/inspections')))
    ftp_layout.addWidget(window.settings_widgets['ftp_base_path'], 3, 1)

    ftp_layout.addWidget(QLabel("Passive Mode:"), 4, 0)
    window.settings_widgets['ftp_passive_mode'] = QCheckBox("Use passive mode (recommended)")
    window.settings_widgets['ftp_passive_mode'].setChecked(ftp_config.get('passive_mode', True))
    ftp_layout.addWidget(window.settings_widgets['ftp_passive_mode'], 4, 1)

    ftp_test_btn = QPushButton("🔗 Test FTP Connection")
    ftp_test_btn.clicked.connect(window.test_ftp_connection)
    ftp_test_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #2196F3; color: white; }")
    ftp_layout.addWidget(ftp_test_btn, 5, 0, 1, 2)

    scroll_layout.addWidget(ftp_group)

    # Logging
    logging_group = QGroupBox("Logging Configuration")
    logging_layout = QGridLayout(logging_group)
    logging_config = window.config_manager.get_config().get('logging', {})

    logging_layout.addWidget(QLabel("Log Level:"), 0, 0)
    window.settings_widgets['logging_level'] = QComboBox()
    window.settings_widgets['logging_level'].addItems(['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'])
    current_level = str(logging_config.get('level', 'INFO'))
    if current_level in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
        window.settings_widgets['logging_level'].setCurrentText(current_level)
    logging_layout.addWidget(window.settings_widgets['logging_level'], 0, 1)

    logging_layout.addWidget(QLabel("Log Folder:"), 1, 0)
    window.settings_widgets['logging_log_folder_path'] = QLineEdit(str(logging_config.get('log_folder_path', 'logs')))
    logging_layout.addWidget(window.settings_widgets['logging_log_folder_path'], 1, 1)

    logging_layout.addWidget(QLabel("Log File Prefix:"), 2, 0)
    window.settings_widgets['logging_log_file_prefix'] = QLineEdit(str(logging_config.get('log_file_prefix', 'app')))
    logging_layout.addWidget(window.settings_widgets['logging_log_file_prefix'], 2, 1)

    window.settings_widgets['logging_console'] = QCheckBox("Enable Console Logging")
    window.settings_widgets['logging_console'].setChecked(bool(logging_config.get('console', True)))
    logging_layout.addWidget(window.settings_widgets['logging_console'], 3, 0, 1, 2)

    scroll_layout.addWidget(logging_group)

    # GUI conf
    gui_group = QGroupBox("GUI Configuration")
    gui_layout = QGridLayout(gui_group)
    gui_config = window.config_manager.get_config().get('gui', {})
    programs_path = window.config_manager.get_config().get('programs_path', 'programs')

    gui_layout.addWidget(QLabel("Window Title:"), 0, 0)
    window.settings_widgets['gui_window_title'] = QLineEdit(str(gui_config.get('window_title', 'AUAS Scenario Inspector')))
    gui_layout.addWidget(window.settings_widgets['gui_window_title'], 0, 1)

    gui_layout.addWidget(QLabel("Theme:"), 1, 0)
    window.settings_widgets['gui_theme'] = QComboBox()
    window.settings_widgets['gui_theme'].addItems(['default', 'dark', 'light'])
    current_theme = str(gui_config.get('theme', 'default'))
    if current_theme in ['default', 'dark', 'light']:
        window.settings_widgets['gui_theme'].setCurrentText(current_theme)
    gui_layout.addWidget(window.settings_widgets['gui_theme'], 1, 1)

    gui_layout.addWidget(QLabel("Default Geometry:"), 2, 0)
    window.settings_widgets['gui_default_geometry'] = QLineEdit(str(gui_config.get('default_geometry', '1200x800')))
    gui_layout.addWidget(window.settings_widgets['gui_default_geometry'], 2, 1)

    scroll_layout.addWidget(gui_group)

    programs_group = QGroupBox("Programs Folder")
    programs_layout = QGridLayout(programs_group)
    programs_layout.addWidget(QLabel("Programs Folder Path:"), 0, 0)
    window.settings_widgets['programs_path'] = QLineEdit(str(programs_path))
    programs_layout.addWidget(window.settings_widgets['programs_path'], 0, 1)
    scroll_layout.addWidget(programs_group)

    # Security
    security_group = QGroupBox("Security Configuration")
    security_layout = QGridLayout(security_group)
    security_config = window.config_manager.get_config().get('security', {})

    security_layout.addWidget(QLabel("Bcrypt Rounds:"), 0, 0)
    window.settings_widgets['security_bcrypt_rounds'] = NoWheelSpinBox()
    window.settings_widgets['security_bcrypt_rounds'].setRange(4, 20)
    window.settings_widgets['security_bcrypt_rounds'].setValue(int(security_config.get('bcrypt_rounds', 12)))
    security_layout.addWidget(window.settings_widgets['security_bcrypt_rounds'], 0, 1)

    security_layout.addWidget(QLabel("Session Timeout (s):"), 1, 0)
    window.settings_widgets['security_session_timeout'] = NoWheelSpinBox()
    window.settings_widgets['security_session_timeout'].setRange(60, 86400)
    window.settings_widgets['security_session_timeout'].setValue(int(security_config.get('session_timeout', 3600)))
    security_layout.addWidget(window.settings_widgets['security_session_timeout'], 1, 1)

    scroll_layout.addWidget(security_group)

    # Set scroll
    scroll_area.setWidget(scroll_widget)
    scroll_area.setWidgetResizable(True)
    layout.addWidget(scroll_area)

    return settings_widget
