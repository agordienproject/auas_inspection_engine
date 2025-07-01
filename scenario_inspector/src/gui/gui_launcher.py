import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from scenario_inspector.gui.main_window import MainWindow


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = MainWindow()
    gui.show()

    # Start ROS 2 spinning in a background thread
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()