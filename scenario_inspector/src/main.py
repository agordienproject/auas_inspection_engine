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
    """Setup logging configuration"""
    config = ConfigManager()
    log_level = getattr(logging, config.get_config()['logging']['level'])
    log_file = config.get_config()['logging']['file']
    
    # Create logs directory if it doesn't exist
    import os
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler() if config.get_config()['logging']['console'] else logging.NullHandler()
        ]
    )

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