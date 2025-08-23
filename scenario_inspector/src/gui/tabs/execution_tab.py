"""Builder for the Execution tab UI."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QProgressBar, QTextEdit
)
from PyQt5.QtCore import Qt


def build_execution_tab(window) -> QWidget:
    """Build the Execution tab and wire handlers to the main window.

    Contract:
    - window must provide update_api_status_display(), start_execution(),
        stop_execution(), upload_inspection_to_ftp(), save_inspection_to_database(),
        and open_inspection_folder().
    - The following attributes will be attached to window: api_status_group,
        api_status_label, start_execution_btn, stop_execution_btn, progress_bar,
        progress_text, last_inspection_path_label, ftp_upload_btn, db_save_btn,
        open_folder_btn.
    """
    exec_widget = QWidget()
    layout = QVBoxLayout(exec_widget)

    # API status
    window.api_status_group = QGroupBox("API Status")
    api_status_layout = QHBoxLayout(window.api_status_group)
    window.api_status_label = QLabel()
    window.update_api_status_display()
    api_status_layout.addWidget(window.api_status_label)
    layout.addWidget(window.api_status_group)

    # Controls
    controls_group = QGroupBox("Execution Controls")
    controls_layout = QVBoxLayout(controls_group)

    window.start_execution_btn = QPushButton("Start Inspection")
    window.start_execution_btn.clicked.connect(window.start_execution)
    window.start_execution_btn.setEnabled(False)
    controls_layout.addWidget(window.start_execution_btn)

    window.stop_execution_btn = QPushButton("Stop Inspection")
    window.stop_execution_btn.clicked.connect(window.stop_execution)
    window.stop_execution_btn.setEnabled(False)
    controls_layout.addWidget(window.stop_execution_btn)

    layout.addWidget(controls_group)

    # Progress
    progress_group = QGroupBox("Execution Progress")
    progress_layout = QVBoxLayout(progress_group)

    window.progress_bar = QProgressBar()
    progress_layout.addWidget(window.progress_bar)

    window.progress_text = QTextEdit()
    window.progress_text.setReadOnly(True)
    progress_layout.addWidget(window.progress_text)

    layout.addWidget(progress_group)

    # Results
    results_group = QGroupBox("Inspection Results")
    results_layout = QVBoxLayout(results_group)

    window.last_inspection_path_label = QLabel("No inspection completed yet")
    window.last_inspection_path_label.setStyleSheet("font-style: italic; color: gray;")
    results_layout.addWidget(window.last_inspection_path_label)

    upload_layout = QHBoxLayout()

    window.ftp_upload_btn = QPushButton("📤 Upload to FTP Server")
    window.ftp_upload_btn.clicked.connect(window.upload_inspection_to_ftp)
    window.ftp_upload_btn.setEnabled(False)
    window.ftp_upload_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #4CAF50; color: white; }")
    upload_layout.addWidget(window.ftp_upload_btn)

    window.db_save_btn = QPushButton("💾 Save to Database")
    window.db_save_btn.clicked.connect(window.save_inspection_to_database)
    window.db_save_btn.setEnabled(False)
    window.db_save_btn.setStyleSheet("QPushButton { font-weight: bold; border-radius: 8px; padding: 8px 24px; background-color: #2196F3; color: white; } QPushButton:pressed { background-color: #1976D2; }")
    upload_layout.addWidget(window.db_save_btn)

    window.open_folder_btn = QPushButton("📁 Open Folder")
    window.open_folder_btn.clicked.connect(window.open_inspection_folder)
    window.open_folder_btn.setEnabled(False)
    window.open_folder_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #FF9800; color: white; }")
    upload_layout.addWidget(window.open_folder_btn)

    results_layout.addLayout(upload_layout)
    layout.addWidget(results_group)

    return exec_widget
