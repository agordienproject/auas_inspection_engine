"""
Database connection manager for Scenario Inspector (Simplified Version)
"""
import logging
from typing import Optional, Dict, Any, List
from datetime import datetime

from database.models import User, Inspection
from config.config_manager import ConfigManager

class DatabaseConnection:
    """Simplified database manager without PostgreSQL dependency"""
    
    def __init__(self):
        self.config_manager = ConfigManager()
        self.logger = logging.getLogger(__name__)
        
        # Mock users for testing
        self.mock_users = [
            User(
                id_user=1,
                first_name="Admin",
                last_name="User", 
                pseudo="admin",
                email="admin@test.com",
                password="admin123",  # In real app, this would be hashed
                role="admin"
            ),
            User(
                id_user=2,
                first_name="Test",
                last_name="Inspector",
                pseudo="inspector", 
                email="inspector@test.com",
                password="inspector123",
                role="inspector"
            )
        ]
        
        # Mock inspections storage
        self.mock_inspections = []
        self.next_inspection_id = 1
    
    def test_connection(self) -> bool:
        """Test database connection (always returns True in mock mode)"""
        return True
    
    def authenticate_user(self, pseudo: str, password: str) -> Optional[User]:
        """Authenticate user with pseudo and password (simplified - no hashing)"""
        try:
            for user in self.mock_users:
                if user.pseudo == pseudo and user.password == password:
                    self.logger.info(f"User {pseudo} authenticated successfully")
                    return user
            self.logger.warning(f"Authentication failed for user {pseudo}")
            return None
        except Exception as e:
            self.logger.error(f"Authentication error: {e}")
            return None
    
    def get_user_by_pseudo(self, pseudo: str) -> Optional[User]:
        """Get user by pseudo"""
        try:
            for user in self.mock_users:
                if user.pseudo == pseudo:
                    return user
            return None
        except Exception as e:
            self.logger.error(f"Get user error: {e}")
            return None
    
    def create_inspection(self, inspection: Inspection) -> Optional[int]:
        """Create new inspection and return inspection ID"""
        try:
            inspection.id_inspection = self.next_inspection_id
            self.next_inspection_id += 1
            self.mock_inspections.append(inspection)
            self.logger.info(f"Created inspection with ID: {inspection.id_inspection}")
            return inspection.id_inspection
        except Exception as e:
            self.logger.error(f"Create inspection error: {e}")
            return None
    
    def update_inspection(self, inspection: Inspection) -> bool:
        """Update existing inspection"""
        try:
            for i, existing in enumerate(self.mock_inspections):
                if existing.id_inspection == inspection.id_inspection:
                    self.mock_inspections[i] = inspection
                    self.logger.info(f"Updated inspection ID: {inspection.id_inspection}")
                    return True
            return False
        except Exception as e:
            self.logger.error(f"Update inspection error: {e}")
            return False
    
    def get_inspection(self, inspection_id: int) -> Optional[Inspection]:
        """Get inspection by ID"""
        try:
            for inspection in self.mock_inspections:
                if inspection.id_inspection == inspection_id:
                    return inspection
            return None
        except Exception as e:
            self.logger.error(f"Get inspection error: {e}")
            return None
    
    def get_recent_inspections(self, limit: int = 50) -> List[Inspection]:
        """Get recent inspections"""
        try:
            # Sort by creation date and return most recent
            sorted_inspections = sorted(
                self.mock_inspections, 
                key=lambda x: x.creation_date or datetime.now(), 
                reverse=True
            )
            return sorted_inspections[:limit]
        except Exception as e:
            self.logger.error(f"Get recent inspections error: {e}")
            return []