from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox
from src.auth.password_utils import verify_password
from src.database.connection import get_user_credentials

class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super(LoginDialog, self).__init__(parent)
        self.setWindowTitle("User Login")
        self.setFixedSize(300, 150)
        
        layout = QVBoxLayout()
        
        self.username_label = QLabel("Username:")
        self.username_input = QLineEdit()
        layout.addWidget(self.username_label)
        layout.addWidget(self.username_input)
        
        self.password_label = QLabel("Password:")
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        layout.addWidget(self.password_label)
        layout.addWidget(self.password_input)
        
        self.login_button = QPushButton("Login")
        self.login_button.clicked.connect(self.handle_login)
        layout.addWidget(self.login_button)
        
        self.setLayout(layout)

    def handle_login(self):
        username = self.username_input.text()
        password = self.password_input.text()
        
        user_credentials = get_user_credentials(username)
        
        if user_credentials and verify_password(password, user_credentials['hashed_password']):
            QMessageBox.information(self, "Login Successful", "Welcome!")
            self.accept()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid username or password.")