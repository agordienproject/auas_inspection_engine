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
    debug_mode = log_config.get('debug_mode', False)
    
    # Override log level to DEBUG if debug_mode is enabled
    if debug_mode:
        log_level = logging.DEBUG
    
    # Create timestamp strings
    now = datetime.now()
    date_folder = now.strftime("%Y-%m-%d")
    timestamp = now.strftime("%Y%m%d_%H%M%S")  # New format: YYYYMMDD_HHMMSS
    
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
    
    # Log debug mode status
    if debug_mode:
        logger.info("DEBUG MODE ENABLED - Enhanced API logging is active")
        logger.debug("Debug logging level is active")

def main():
    """Main entry point for the Scenario Inspector application"""
    # Initialize logging
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("Starting Scenario Inspector")
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    try:
        # Start with guest user (no authentication required)
        logger.info("Starting application in guest mode (no login required)")
        
        # Create and show main window without authenticated user
        main_window = InspectionMainWindow()
        main_window.show()
        
        # Start application event loop
        return app.exec_()
        
    except Exception as e:
        logger.error(f"Application error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())