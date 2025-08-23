"""
Login dialog and authentication for Scenario Inspector
Now using API instead of direct database connection
"""
import logging
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel,
                             QLineEdit, QPushButton, QMessageBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QPalette

from database.api_connection import APIConnection
from database.models import User
from config.config_manager import ConfigManager

class LoginDialog(QDialog):
    """Login dialog for user authentication using API"""

    # Signal emitted when user successfully logs in
    user_authenticated = pyqtSignal(User)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.logger = logging.getLogger(__name__)

        # Initialize API connection
        self.config_manager = ConfigManager()
        api_config = self.config_manager.get_api_config()
        self.api_connection = APIConnection(api_config)

        self.authenticated_user = None

        self.setup_ui()
        self.setModal(True)

    def setup_ui(self):
        """Setup the user interface with modern style"""
        self.setWindowTitle("AUAS Scenario Inspector - Login")
        self.setFixedSize(400, 320)

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)

        # Title
        title_label = QLabel("Scenario Inspector")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # Subtitle
        subtitle_label = QLabel("Please enter your credentials")
        subtitle_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(subtitle_label)

        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(separator)

        # Email field (changed from username)
        email_layout = QHBoxLayout()
        email_label = QLabel("Email:")
        email_label.setMinimumWidth(80)
        self.email_edit = QLineEdit()
        self.email_edit.setPlaceholderText("Enter your email address")
        self.email_edit.setStyleSheet("QLineEdit { border-radius: 8px; border: 1px solid #bdbdbd; padding: 6px; background: #fafafa; }")
        email_layout.addWidget(email_label)
        email_layout.addWidget(self.email_edit)
        main_layout.addLayout(email_layout)

        # Password field
        password_layout = QHBoxLayout()
        password_label = QLabel("Password:")
        password_label.setMinimumWidth(80)
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.password_edit.setPlaceholderText("Enter your password")
        self.password_edit.setStyleSheet("QLineEdit { border-radius: 8px; border: 1px solid #bdbdbd; padding: 6px; background: #fafafa; }")
        password_layout.addWidget(password_label)
        password_layout.addWidget(self.password_edit)
        main_layout.addLayout(password_layout)

        # Buttons
        button_layout = QHBoxLayout()
        self.login_button = QPushButton("Login")
        self.login_button.setDefault(True)
        self.login_button.setStyleSheet("QPushButton { font-weight: bold; border-radius: 8px; padding: 8px 24px; background-color: #2196F3; color: white; } QPushButton:pressed { background-color: #1976D2; }")
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setStyleSheet("QPushButton { font-weight: bold; border-radius: 8px; padding: 8px 24px; background-color: #bdbdbd; color: #333; } QPushButton:pressed { background-color: #9e9e9e; }")

        button_layout.addStretch()
        button_layout.addWidget(self.login_button)
        button_layout.addWidget(self.cancel_button)
        main_layout.addLayout(button_layout)

        main_layout.addStretch()
        self.setLayout(main_layout)

        # Add drop shadow effect to dialog
        self.setStyleSheet("QDialog { background: #fff; border-radius: 12px; }")

        # Connect signals
        self.login_button.clicked.connect(self.attempt_login)
        self.cancel_button.clicked.connect(self.reject)
        self.email_edit.returnPressed.connect(self.attempt_login)
        self.password_edit.returnPressed.connect(self.attempt_login)

    def attempt_login(self):
        """Attempt to authenticate user via API"""
        email = self.email_edit.text().strip()
        password = self.password_edit.text()

        if not email or not password:
            QMessageBox.warning(self, "Login Error", "Please enter both email and password.")
            return

        # Disable login button during authentication
        self.login_button.setEnabled(False)
        self.login_button.setText("Authenticating...")

        try:
            # Test API connection first
            connection_result = self.api_connection.test_connection()
            if connection_result['status'] != 'available':
                error_msg = connection_result.get('message', 'Unknown API error')
                self.logger.error(f"API connection failed: {error_msg}")
                QMessageBox.critical(self, "API Connection Failed",
                    f"Cannot connect to API server:\n{error_msg}\n\nPlease check your configuration and ensure the API server is running.")
                return

            # Authenticate user via API
            auth_result = self.api_connection.authenticate(email, password)

            if not auth_result.get('success', False):
                error_msg = auth_result.get('message', 'Authentication failed')
                QMessageBox.warning(self, "Login Failed", error_msg)
                return

            # Create user object from API response
            user_data = auth_result.get('user_data', {})
            user = User(
                id_user=user_data.get('id'),
                pseudo=user_data.get('pseudo', email),
                email=user_data.get('email', email),
                role=user_data.get('role', 'user')
            )

            # Store API connection for later use
            user.api_connection = self.api_connection

            # Authentication successful
            self.authenticated_user = user
            self.user_authenticated.emit(user)
            self.accept()

        except Exception as e:
            self.logger.error(f"Login error: {e}")
            QMessageBox.critical(self, "Login Error",
                               f"An error occurred during login: {str(e)}")
        finally:
            # Re-enable login button
            self.login_button.setEnabled(True)
            self.login_button.setText("Login")

    def get_authenticated_user(self) -> User:
        """Get the authenticated user"""
        return self.authenticated_user

    def clear_fields(self):
        """Clear input fields"""
        self.username_edit.clear()
        self.password_edit.clear()
        self.username_edit.setFocus()
