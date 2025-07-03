"""
Simple database connection for Scenario Inspector
"""
import psycopg2
import logging
from typing import Optional
from datetime import datetime

from database.models import User, Inspection
from config.config_manager import ConfigManager
from auth.password_utils import PasswordUtils

class DatabaseConnection:
    """Simple database manager for authentication and inspection data insertion"""
    
    def __init__(self):
        self.config_manager = ConfigManager()
        self.db_config = self.config_manager.get_database_config()
        self.logger = logging.getLogger(__name__)
        self.password_utils = PasswordUtils(rounds=12)
    
    def get_connection(self):
        """Get database connection"""
        try:
            conn = psycopg2.connect(
                host=self.db_config['host'],
                port=self.db_config['port'],
                database=self.db_config['database'],
                user=self.db_config['username'],
                password=self.db_config['password']
            )
            return conn
        except Exception as e:
            self.logger.error(f"Database connection failed: {e}")
            raise
    
    def test_connection(self) -> bool:
        """Test database connection"""
        try:
            conn = self.get_connection()
            conn.close()
            return True
        except Exception as e:
            self.logger.error(f"Database connection test failed: {e}")
            return False
    
    def authenticate_user(self, email: str, password: str) -> Optional[User]:
        """Authenticate user with email and password using bcrypt verification"""
        try:
            conn = self.get_connection()
            cursor = conn.cursor()
            
            # Get user data including the hashed password
            cursor.execute(
                'SELECT id_user, first_name, last_name, pseudo, email, role, password FROM "DIM_USER" WHERE email = %s AND deleted = FALSE',
                (email,)
            )
            
            row = cursor.fetchone()
            cursor.close()
            conn.close()
            
            if row:
                # Verify the password using bcrypt
                stored_password_hash = row[6]  # password is now the 7th column (index 6)
                if self.password_utils.verify_password(password, stored_password_hash):
                    return User(
                        id_user=row[0],
                        first_name=row[1], 
                        last_name=row[2],
                        pseudo=row[3],
                        email=row[4],
                        role=row[5]
                    )
            return None
            
        except Exception as e:
            self.logger.error(f"Authentication error: {e}")
            return None
    
    def insert_inspection(self, inspection: Inspection) -> bool:
        """Insert new inspection into database"""
        try:
            conn = self.get_connection()
            cursor = conn.cursor()
            
            cursor.execute(
                '''INSERT INTO "FCT_INSPECTION" 
                   (name_piece, ref_piece, name_program, state, dents, corrosions, 
                    scratches, details, inspection_date, inspection_path, 
                    inspection_status, creation_date, user_creation)
                   VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)''',
                (inspection.name_piece, inspection.ref_piece, inspection.name_program,
                 inspection.state, inspection.dents, inspection.corrosions,
                 inspection.scratches, inspection.details, inspection.inspection_date,
                 inspection.inspection_path, inspection.inspection_status,
                 inspection.creation_date, inspection.user_creation)
            )
            
            conn.commit()
            cursor.close()
            conn.close()
            
            self.logger.info("Inspection data inserted successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to insert inspection: {e}")
            return False
    
    def hash_password(self, password: str) -> str:
        """Hash a password using bcrypt with salt rounds of 12"""
        return self.password_utils.hash_password(password)