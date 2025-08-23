"""
Data models for AUAS Inspection Engine
API-based data structures matching the database schema
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Any

@dataclass
class User:
    """User model matching DIM_USER table structure"""
    id_user: Optional[int] = None
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    pseudo: Optional[str] = None
    email: Optional[str] = None
    password: Optional[str] = None
    role: Optional[str] = None
    deleted: bool = False

    # API connection reference for authenticated users
    api_connection: Optional[Any] = None

@dataclass
class Inspection:
    """Inspection model matching FCT_INSPECTION table structure"""
    id_inspection: Optional[int] = None
    name_piece: Optional[str] = None
    ref_piece: Optional[str] = None
    name_program: Optional[str] = None
    state: Optional[str] = None
    dents: Optional[bool] = None
    corrosions: Optional[bool] = None
    scratches: Optional[bool] = None
    details: Optional[str] = None
    inspection_date: Optional[datetime] = None
    inspection_path: Optional[str] = None
    inspection_status: str = 'PENDING'
    user_validation: Optional[int] = None
    validation_date: Optional[datetime] = None
    creation_date: Optional[datetime] = None
    user_creation: Optional[int] = None
    modification_date: Optional[datetime] = None
    user_modification: Optional[int] = None
    deleted: bool = False

    def to_dict(self) -> dict:
        """Convert inspection to dictionary for API calls"""
        data = {}
        for field in self.__dataclass_fields__:
            value = getattr(self, field)
            if value is not None:
                if isinstance(value, datetime):
                    data[field] = value.isoformat()
                else:
                    data[field] = value
        return data

    @classmethod
    def from_dict(cls, data: dict) -> 'Inspection':
        """Create inspection from dictionary (API response)"""
        # Handle datetime fields
        datetime_fields = ['inspection_date', 'validation_date', 'creation_date', 'modification_date']

        for field in datetime_fields:
            if field in data and data[field] is not None:
                if isinstance(data[field], str):
                    try:
                        data[field] = datetime.fromisoformat(data[field].replace('Z', '+00:00'))
                    except ValueError:
                        data[field] = None

        # Filter only known fields
        known_fields = set(cls.__dataclass_fields__.keys())
        filtered_data = {k: v for k, v in data.items() if k in known_fields}

        return cls(**filtered_data)
