import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from ros2gui.main_window import ROS2GuiApp


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ROS2GuiApp()
    gui.show()

    # Start ROS 2 spinning in a background thread
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
