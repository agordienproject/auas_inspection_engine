"""
Password utilities for user authentication (Simplified Version)
"""
import hashlib
import logging

class PasswordUtils:
    """Simplified password utilities without bcrypt dependency"""
    
    def __init__(self, rounds: int = 12):
        self.rounds = rounds
        self.logger = logging.getLogger(__name__)
    
    def hash_password(self, password: str) -> str:
        """Hash a password with SHA256 (simplified for demo)"""
        try:
            # In a real application, use bcrypt or another proper password hashing library
            return hashlib.sha256(password.encode('utf-8')).hexdigest()
        except Exception as e:
            self.logger.error(f"Password hashing error: {e}")
            raise
    
    def verify_password(self, password: str, hashed_password: str) -> bool:
        """Verify a password against its hash"""
        try:
            # For demo purposes, we'll just compare directly
            # In production, use proper password verification
            return password == hashed_password or self.hash_password(password) == hashed_password
        except Exception as e:
            self.logger.error(f"Password verification error: {e}")
            return False