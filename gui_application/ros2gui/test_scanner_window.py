#!/usr/bin/env python3
"""
Test script for scanCONTROL Scanner Configuration Window

This script launches the scanner configuration window independently 
for testing purposes.
"""

import sys
import os
from PyQt5.QtWidgets import QApplication

# Add the ros2gui directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scanner_config import ScannerConfigWindow

def main():
    """Main function to run the scanner configuration window"""
    print("🚀 Starting scanCONTROL Scanner Configuration Window")
    print("=" * 60)
    print("Features:")
    print("  ✓ Device discovery and connection")
    print("  ✓ Parameter configuration (exposure, idle time, trigger)")
    print("  ✓ Real-time profile visualization")
    print("  ✓ Data recording and export (CSV, NPZ, AVI, HDF5)")
    print("  ✓ Live plotting with history overlay")
    print("=" * 60)
    
    app = QApplication(sys.argv)
    
    # Create and show the scanner window
    scanner_window = ScannerConfigWindow()
    scanner_window.show()
    
    print("📱 Scanner configuration window opened")
    print("💡 If no real scanner is connected, simulated data will be used")
    
    # Run the application
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
