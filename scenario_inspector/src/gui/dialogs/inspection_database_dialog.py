"""
Dialog for collecting inspection metadata before saving it to the database via API.
"""
from datetime import datetime
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QLineEdit, QTextEdit, QPushButton, QComboBox, QCheckBox, QMessageBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class InspectionDatabaseDialog(QDialog):
    """Modern dialog for preparing inspection data to save to the database."""

    def __init__(self, inspection_data: dict, parent=None):
        """Initialize the dialog.

        Args:
            inspection_data: Pre-filled values from the last inspection
                (piece, program, date, ftp path).
            parent: Optional parent widget.
        """
        super().__init__(parent)
        self.inspection_data = inspection_data or {}
        self.result_data = None

        self.setWindowTitle("Save Inspection to Database")
        self.setModal(True)
        self.setFixedSize(800, 600)
        self.setStyleSheet("QDialog { background: #fff; border-radius: 12px; }")

        self._build_ui()

    def _build_ui(self):
        """Build the dialog UI elements and layout."""
        main_layout = QVBoxLayout(self)

        # Title
        title_label = QLabel("Inspection Details")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # Form
        form_layout = QFormLayout()
        form_layout.setLabelAlignment(Qt.AlignRight)
        form_layout.setHorizontalSpacing(24)
        form_layout.setVerticalSpacing(12)

        # Piece Name (read-only)
        self.name_piece_edit = QLineEdit(self.inspection_data.get('piece_name', ''))
        self.name_piece_edit.setReadOnly(True)
        self.name_piece_edit.setStyleSheet("background-color: #f0f0f0; border-radius: 6px; padding: 6px;")
        form_layout.addRow("Piece Name:", self.name_piece_edit)

        # Piece Reference (read-only)
        self.ref_piece_edit = QLineEdit(self.inspection_data.get('ref_piece', ''))
        self.ref_piece_edit.setReadOnly(True)
        self.ref_piece_edit.setStyleSheet("background-color: #f0f0f0; border-radius: 6px; padding: 6px;")
        form_layout.addRow("Piece Reference:", self.ref_piece_edit)

        # Program name (read-only)
        self.program_name_edit = QLineEdit(self.inspection_data.get('program_name', ''))
        self.program_name_edit.setReadOnly(True)
        self.program_name_edit.setStyleSheet("background-color: #f0f0f0; border-radius: 6px; padding: 6px;")
        form_layout.addRow("Program Name:", self.program_name_edit)

        # State dropdown
        self.state_combo = QComboBox()
        self.state_combo.addItems([
            "Perfect",
            "Almost perfect",
            "Good",
            "Average",
            "Not good",
            "Really bad",
            "Destroyed",
        ])
        self.state_combo.setCurrentText("Good")
        form_layout.addRow("State:", self.state_combo)

        # Conditions
        self.dents_checkbox = QCheckBox("Has dents")
        form_layout.addRow("Dents:", self.dents_checkbox)

        self.corrosions_checkbox = QCheckBox("Has corrosions")
        form_layout.addRow("Corrosions:", self.corrosions_checkbox)

        self.scratches_checkbox = QCheckBox("Has scratches")
        form_layout.addRow("Scratches:", self.scratches_checkbox)

        # Details
        self.details_text = QTextEdit()
        self.details_text.setMinimumHeight(100)
        self.details_text.setPlaceholderText("Optional details about the inspection...")
        self.details_text.setStyleSheet("border-radius: 6px; border: 1px solid #bdbdbd; background: #fafafa; padding: 6px;")
        form_layout.addRow("Details:", self.details_text)

        # Inspection date (read-only)
        inspection_date = self.inspection_data.get(
            'inspection_date', datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        )
        self.inspection_date_edit = QLineEdit(inspection_date)
        self.inspection_date_edit.setReadOnly(True)
        self.inspection_date_edit.setStyleSheet("background-color: #f0f0f0; border-radius: 6px; padding: 6px;")
        form_layout.addRow("Inspection Date:", self.inspection_date_edit)

        # FTP path (read-only)
        self.ftp_path_edit = QLineEdit(self.inspection_data.get('inspection_path', ''))
        self.ftp_path_edit.setReadOnly(True)
        self.ftp_path_edit.setPlaceholderText("Upload to FTP first to get the path")
        self.ftp_path_edit.setStyleSheet("background-color: #f0f0f0; border-radius: 6px; padding: 6px;")
        form_layout.addRow("Inspection Path:", self.ftp_path_edit)

        main_layout.addLayout(form_layout)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        self.save_btn = QPushButton("💾 Save to Database")
        self.save_btn.setStyleSheet("QPushButton { font-weight: bold; border-radius: 8px; padding: 10px 32px; background-color: #4CAF50; color: white; } QPushButton:pressed { background-color: #388E3C; }")
        self.save_btn.clicked.connect(self.save_to_database)
        button_layout.addWidget(self.save_btn)

        self.cancel_btn = QPushButton("❌ Cancel")
        self.cancel_btn.setStyleSheet("QPushButton { font-weight: bold; border-radius: 8px; padding: 10px 32px; background-color: #f44336; color: white; } QPushButton:pressed { background-color: #b71c1c; }")
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_btn)
        button_layout.addStretch()

        main_layout.addLayout(button_layout)

    def update_ftp_path(self, ftp_path: str):
        """Update the read-only FTP path field."""
        self.ftp_path_edit.setText(ftp_path)

    def save_to_database(self):
        """Validate form fields and finalize the result payload.

        On success, stores payload in self.result_data and accepts the dialog.
        """
        # Simple validation
        if not self.name_piece_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Piece name is required!")
            return
        if not self.ref_piece_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Piece reference is required!")
            return
        if not self.ftp_path_edit.text().strip():
            QMessageBox.warning(self, "Validation Error", "Please upload to FTP first to get the inspection path!")
            return

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
            "inspection_path": self.ftp_path_edit.text().strip(),
        }
        self.accept()

    def get_result_data(self):
        """Return the form result payload or None if the dialog was cancelled."""
        return self.result_data
