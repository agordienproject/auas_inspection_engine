"""
Date and time utilities for Scenario Inspector
"""
import os
from datetime import datetime

def format_inspection_timestamp() -> str:
    """Format current timestamp for inspection folders"""
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def format_inspection_date() -> str:
    """Format current date for inspection folders"""
    return datetime.now().strftime("%Y-%m-%d")

def format_database_timestamp() -> str:
    """Format timestamp for database storage"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def get_current_date() -> str:
    """Get current date as YYYY-MM-DD string"""
    return datetime.now().strftime("%Y-%m-%d")

def create_date_stamped_folder(base_path: str) -> str:
    """Create a date-stamped folder for storing inspection results"""
    date_folder = get_current_date()
    full_path = os.path.join(base_path, date_folder)
    os.makedirs(full_path, exist_ok=True)
    return full_path

def format_timestamp(timestamp):
    """Format timestamp"""
    return timestamp.strftime("%Y-%m-%d %H:%M:%S")
