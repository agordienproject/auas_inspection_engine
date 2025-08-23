"""Builder for the Programs tab UI."""

from PyQt5.QtWidgets import (
                QWidget,
                QVBoxLayout,
                QGroupBox,
                QHBoxLayout,
                QListWidget,
                QPushButton,
                QTextEdit,
)


def build_program_tab(window) -> QWidget:
    """Build the Programs tab and wire actions to the main window.

    Contract:
    - window must provide select_program() and upload_program() handlers.
    - The following attributes will be attached to window:
            programs_list, program_info_text, selected_program_group
    """
    program_widget = QWidget()
    layout = QVBoxLayout(program_widget)

    # Available programs group
    programs_group = QGroupBox("Available Programs")
    programs_layout = QVBoxLayout(programs_group)

    window.programs_list = QListWidget()
    programs_layout.addWidget(window.programs_list)

    # Program actions
    actions_layout = QHBoxLayout()

    select_program_btn = QPushButton("Select Program")
    select_program_btn.clicked.connect(window.select_program)
    actions_layout.addWidget(select_program_btn)

    upload_program_btn = QPushButton("Upload Program")
    upload_program_btn.clicked.connect(window.upload_program)
    actions_layout.addWidget(upload_program_btn)

    programs_layout.addLayout(actions_layout)
    layout.addWidget(programs_group)

    # Selected program info
    window.selected_program_group = QGroupBox("Selected Program")
    selected_layout = QVBoxLayout(window.selected_program_group)

    window.program_info_text = QTextEdit()
    window.program_info_text.setReadOnly(True)
    window.program_info_text.setMaximumHeight(200)
    selected_layout.addWidget(window.program_info_text)

    layout.addWidget(window.selected_program_group)
    return program_widget
