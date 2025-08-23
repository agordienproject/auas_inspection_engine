"""Builder for the System Status tab UI."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout, QPushButton, QGridLayout,
    QScrollArea, QTextEdit, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import Qt


def build_status_tab(window) -> QWidget:
    """Build the System Status tab and wire actions to the main window.

    Contract:
    - window must provide refresh_system_status() and test_all_connections().
    - window must have _initialize_status_display() to seed the grid.
    - Attaches to window: status_layout and status_log.
    """
    status_widget = QWidget()
    layout = QVBoxLayout(status_widget)

    # Control buttons
    buttons_layout = QHBoxLayout()

    refresh_btn = QPushButton("🔄 Refresh Status")
    refresh_btn.clicked.connect(window.refresh_system_status)
    refresh_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; }")
    buttons_layout.addWidget(refresh_btn)

    test_all_btn = QPushButton("🔍 Test All Connections")
    test_all_btn.clicked.connect(window.test_all_connections)
    test_all_btn.setStyleSheet("QPushButton { font-weight: bold; padding: 8px; background-color: #2196F3; color: white; }")
    buttons_layout.addWidget(test_all_btn)

    layout.addLayout(buttons_layout)

    # Status group with scroll area
    status_group = QGroupBox("System Status")
    scroll_area = QScrollArea()
    scroll_widget = QWidget()
    window.status_layout = QGridLayout(scroll_widget)

    window.status_layout.setColumnMinimumWidth(0, 120)
    window.status_layout.setColumnMinimumWidth(1, 60)
    window.status_layout.setColumnMinimumWidth(2, 200)
    window.status_layout.setColumnMinimumWidth(3, 300)

    scroll_area.setWidget(scroll_widget)
    scroll_area.setWidgetResizable(True)
    scroll_area.setMaximumHeight(300)
    scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

    status_group_layout = QVBoxLayout(status_group)
    status_group_layout.addWidget(scroll_area)

    # Initialize display using window helper
    window._initialize_status_display()

    layout.addWidget(status_group)

    # Status log
    log_group = QGroupBox("Connection Log")
    log_layout = QVBoxLayout(log_group)

    window.status_log = QTextEdit()
    window.status_log.setReadOnly(True)
    window.status_log.setMaximumHeight(150)
    log_layout.addWidget(window.status_log)

    layout.addWidget(log_group)
    layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

    return status_widget
