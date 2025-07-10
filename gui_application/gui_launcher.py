import sys
import os
from PyQt5.QtWidgets import QApplication
from ros2gui.main_window import ROS2GuiApp


def main():
    # Fix Qt platform plugin issues (especially for VM/remote environments)
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    
    # Additional Qt fixes for stability
    os.environ['QT_X11_NO_MITSHM'] = '1'
    
    # Suppress GTK warnings (harmless audio module warnings)
    os.environ['GTK_MODULES'] = ''
    
    # Unset problematic environment variables
    if 'GTK_PATH' in os.environ:
        del os.environ['GTK_PATH']
    
    print("🚀 Starting GUI Application...")
    print("🔧 Qt Platform: xcb (X11)")
    
    # Create Qt application without ROS2 initialization
    app = QApplication(sys.argv)
    
    # Create and show main window
    gui = ROS2GuiApp()
    gui.show()
    
    print("✅ GUI Application started successfully")

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("🛑 Shutting down GUI...")
    finally:
        print("👋 GUI Application closed")


if __name__ == "__main__":
    main()
