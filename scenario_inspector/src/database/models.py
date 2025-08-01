"""
Database models for Scenario Inspector
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

@dataclass
class User:
    """User model matching DIM_USER table"""
    id_user: Optional[int] = None
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    pseudo: Optional[str] = None
    email: Optional[str] = None
    password: Optional[str] = None
    role: Optional[str] = None
    deleted: bool = False

@dataclass
class Inspection:
    """Inspection model matching FCT_INSPECTION table"""
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