"""
Password utilities for user authentication using bcrypt
"""
import logging
import bcrypt

class PasswordUtils:
    """Password utilities using bcrypt for secure password hashing"""
    
    def __init__(self, rounds: int = 12):
        self.rounds = rounds
        self.logger = logging.getLogger(__name__)
    
    def hash_password(self, password: str) -> str:
        """Hash a password with bcrypt using salt rounds"""
        try:
            salt = bcrypt.gensalt(rounds=self.rounds)
            hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
            return hashed.decode('utf-8')
        except Exception as e:
            self.logger.error("Password hashing error: %s", e)
            raise
    
    def verify_password(self, password: str, hashed_password: str) -> bool:
        """Verify a password against its bcrypt hash"""
        try:
            return bcrypt.checkpw(password.encode('utf-8'), hashed_password.encode('utf-8'))
        except Exception as e:
            self.logger.error("Password verification error: %s", e)
            return False