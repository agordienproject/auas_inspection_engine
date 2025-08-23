"""Builders for the Piece Info tab UI."""

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLineEdit,
    QPushButton,
    QSpacerItem,
    QSizePolicy,
)


def build_piece_info_tab(window) -> QWidget:
    """Construct the Piece Information tab.

    Contract:
    - window must define save_piece_info() and will receive attributes:
      piece_name_edit, piece_ref_edit
    """
    piece_widget = QWidget()
    layout = QVBoxLayout(piece_widget)

    piece_group = QGroupBox("Piece Information")
    piece_layout = QFormLayout(piece_group)

    window.piece_name_edit = QLineEdit()
    window.piece_ref_edit = QLineEdit()

    piece_layout.addRow("Piece Name:", window.piece_name_edit)
    piece_layout.addRow("Reference:", window.piece_ref_edit)

    # Save button delegates to the main window handler
    save_piece_btn = QPushButton("Save Piece Information")
    save_piece_btn.clicked.connect(window.save_piece_info)
    piece_layout.addWidget(save_piece_btn)

    layout.addWidget(piece_group)
    layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
    return piece_widget
