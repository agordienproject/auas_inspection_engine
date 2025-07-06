"""
AUAS Inspection Engine - Main Entry Point
"""
import sys
import os
import logging
from PyQt5.QtWidgets import QApplication

# Add the src directory to Python path to enable absolute imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gui.main_window import InspectionMainWindow
from config.config_manager import ConfigManager
from auth.login import LoginDialog

def setup_logging():
    """Setup logging configuration with daily folders and timestamped files"""
    from datetime import datetime
    
    config = ConfigManager()
    log_config = config.get_config()['logging']
    
    # Get configuration values
    log_level = getattr(logging, log_config['level'])
    log_folder_path = log_config['log_folder_path']
    log_file_prefix = log_config['log_file_prefix']
    console_logging = log_config.get('console', True)
    
    # Create timestamp strings
    now = datetime.now()
    date_folder = now.strftime("%Y-%m-%d")
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    
    # Construct the full log directory path
    daily_log_dir = os.path.join(log_folder_path, date_folder)
    
    # Create the daily log directory if it doesn't exist
    os.makedirs(daily_log_dir, exist_ok=True)
    
    # Construct the log file name
    log_filename = f"{log_file_prefix}_{timestamp}.log"
    log_file_path = os.path.join(daily_log_dir, log_filename)
    
    # Setup logging handlers
    handlers = [logging.FileHandler(log_file_path)]
    if console_logging:
        handlers.append(logging.StreamHandler())
    
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )
    
    # Log the file path for reference
    logger = logging.getLogger(__name__)
    logger.info(f"Log file created: {log_file_path}")

def main():
    """Main entry point for the Scenario Inspector application"""
    # Initialize logging
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("Starting Scenario Inspector")
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    try:
        # Show login dialog
        login_dialog = LoginDialog()
        if login_dialog.exec_() != LoginDialog.Accepted:
            logger.info("Login cancelled by user")
            return 0
        
        # Get authenticated user
        authenticated_user = login_dialog.get_authenticated_user()
        if not authenticated_user:
            logger.error("Authentication failed")
            return 1
        
        logger.info(f"User {authenticated_user.pseudo} logged in successfully")
        
        # Create and show main window
        main_window = InspectionMainWindow(authenticated_user)
        main_window.show()
        
        # Start application event loop
        return app.exec_()
        
    except Exception as e:
        logger.error(f"Application error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())