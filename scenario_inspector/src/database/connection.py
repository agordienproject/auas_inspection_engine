"""
Simple database connection for Scenario Inspector
"""
import psycopg2
import logging
from typing import Optional
from datetime import datetime
import signal

from database.models import User, Inspection
from config.config_manager import ConfigManager
from auth.password_utils import PasswordUtils


class ConnectionTimeoutError(Exception):
    """Raised when database connection times out"""
    pass


class DatabaseConnection:
    """Simple database manager for authentication and inspection data insertion"""
    
    def __init__(self):
        self.config_manager = ConfigManager()
        self.db_config = self.config_manager.get_database_config()
        self.logger = logging.getLogger(__name__)
        self.password_utils = PasswordUtils(rounds=12)
        self.connection_timeout = 10  # 10 seconds timeout
    
    def _timeout_handler(self, signum, frame):
        """Handler for connection timeout"""
        raise ConnectionTimeoutError("Database connection timed out")
    
    def get_connection(self):
        """Get database connection with timeout and better error handling"""
        try:
            self.logger.info("Attempting to connect to database...")
            
            # Set up timeout signal (only works on Unix systems)
            if hasattr(signal, 'SIGALRM'):
                signal.signal(signal.SIGALRM, self._timeout_handler)
                signal.alarm(self.connection_timeout)
            
            conn = psycopg2.connect(
                host=self.db_config['host'],
                port=self.db_config['port'],
                database=self.db_config['database'],
                user=self.db_config['username'],
                password=self.db_config['password'],
                connect_timeout=self.connection_timeout
            )
            
            # Cancel the alarm if connection succeeded
            if hasattr(signal, 'SIGALRM'):
                signal.alarm(0)
            
            self.logger.info("Database connection established successfully")
            return conn
            
        except ConnectionTimeoutError:
            error_msg = f"Database connection timed out after {self.connection_timeout} seconds"
            self.logger.error(error_msg)
            raise ConnectionTimeoutError(error_msg)
            
        except psycopg2.OperationalError as e:
            error_str = str(e)
            if "authentication failed" in error_str.lower() or "password authentication failed" in error_str.lower():
                error_msg = "Database authentication failed - check username and password in configuration"
                self.logger.error(error_msg)
                raise psycopg2.OperationalError(error_msg)
            elif "could not connect to server" in error_str.lower() or "connection refused" in error_str.lower():
                error_msg = f"Connection to database server failed - server may be down or unreachable at {self.db_config['host']}:{self.db_config['port']}"
                self.logger.error(error_msg)
                raise psycopg2.OperationalError(error_msg)
            elif "database" in error_str.lower() and "does not exist" in error_str.lower():
                error_msg = f"Database '{self.db_config['database']}' does not exist on the server"
                self.logger.error(error_msg)
                raise psycopg2.OperationalError(error_msg)
            else:
                error_msg = f"Database connection failed: {error_str}"
                self.logger.error(error_msg)
                raise psycopg2.OperationalError(error_msg)
                
        except Exception as e:
            # Cancel the alarm if an exception occurred
            if hasattr(signal, 'SIGALRM'):
                signal.alarm(0)
            error_msg = f"Unexpected database connection error: {str(e)}"
            self.logger.error(error_msg)
            raise Exception(error_msg)
    
    def test_connection(self) -> dict:
        """Test database connection with detailed status information"""
        try:
            self.logger.info("Testing database connection...")
            conn = self.get_connection()
            
            # Test with a simple query
            cursor = conn.cursor()
            cursor.execute('SELECT 1')
            cursor.fetchone()
            cursor.close()
            conn.close()
            
            self.logger.info("Database connection test successful")
            return {
                'status': 'success',
                'message': 'Database connection successful',
                'details': {
                    'host': self.db_config['host'],
                    'port': self.db_config['port'],
                    'database': self.db_config['database'],
                    'username': self.db_config['username']
                }
            }
            
        except ConnectionTimeoutError as e:
            error_msg = f"Connection to database server timed out after {self.connection_timeout} seconds"
            self.logger.error(error_msg)
            return {
                'status': 'timeout',
                'message': error_msg,
                'error': str(e)
            }
            
        except psycopg2.OperationalError as e:
            error_str = str(e)
            self.logger.error(f"Database connection test failed: {error_str}")
            return {
                'status': 'connection_failed',
                'message': error_str,
                'error': str(e)
            }
            
        except Exception as e:
            error_msg = f"Database connection test failed with unexpected error: {str(e)}"
            self.logger.error(error_msg)
            return {
                'status': 'error',
                'message': error_msg,
                'error': str(e)
            }
    
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