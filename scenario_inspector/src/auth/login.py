"""
Login dialog and authentication for Scenario Inspector
"""
import logging
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QLineEdit, QPushButton, QMessageBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QPalette

from database.connection import DatabaseConnection
from database.models import User
from auth.password_utils import PasswordUtils

class LoginDialog(QDialog):
    """Login dialog for user authentication"""
    
    # Signal emitted when user successfully logs in
    user_authenticated = pyqtSignal(User)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.logger = logging.getLogger(__name__)
        self.db_connection = DatabaseConnection()
        self.password_utils = PasswordUtils()
        self.authenticated_user = None
        
        self.setup_ui()
        self.setModal(True)
        
    def setup_ui(self):
        """Setup the user interface"""
        self.setWindowTitle("AUAS Scenario Inspector - Login")
        self.setFixedSize(400, 300)
        
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
        
        # Username field
        username_layout = QHBoxLayout()
        username_label = QLabel("Username:")
        username_label.setMinimumWidth(80)
        self.username_edit = QLineEdit()
        self.username_edit.setPlaceholderText("Enter your username")
        username_layout.addWidget(username_label)
        username_layout.addWidget(self.username_edit)
        main_layout.addLayout(username_layout)
        
        # Password field
        password_layout = QHBoxLayout()
        password_label = QLabel("Password:")
        password_label.setMinimumWidth(80)
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.password_edit.setPlaceholderText("Enter your password")
        password_layout.addWidget(password_label)
        password_layout.addWidget(self.password_edit)
        main_layout.addLayout(password_layout)
        
        # Buttons
        button_layout = QHBoxLayout()
        self.login_button = QPushButton("Login")
        self.login_button.setDefault(True)
        self.cancel_button = QPushButton("Cancel")
        
        button_layout.addStretch()
        button_layout.addWidget(self.login_button)
        button_layout.addWidget(self.cancel_button)
        main_layout.addLayout(button_layout)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
        
        # Connect signals
        self.login_button.clicked.connect(self.attempt_login)
        self.cancel_button.clicked.connect(self.reject)
        self.username_edit.returnPressed.connect(self.attempt_login)
        self.password_edit.returnPressed.connect(self.attempt_login)
    
    def attempt_login(self):
        """Attempt to authenticate user"""
        username = self.username_edit.text().strip()
        password = self.password_edit.text()
        
        if not username or not password:
            QMessageBox.warning(self, "Login Error", "Please enter both username and password.")
            return
        
        # Disable login button during authentication
        self.login_button.setEnabled(False)
        self.login_button.setText("Authenticating...")
        
        try:
            # Test database connection first
            if not self.db_connection.test_connection():
                QMessageBox.critical(self, "Database Error", 
                                   "Cannot connect to database. Please check your configuration.")
                return
            
            # Authenticate user (simplified version without password hashing)
            user = self.db_connection.authenticate_user(username, password)
            if not user:
                QMessageBox.warning(self, "Login Failed", "Invalid username or password.")
                return
            
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