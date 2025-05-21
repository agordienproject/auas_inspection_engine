import rclpy
from rclpy.node import Node
import serial
import time

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Give time for Arduino to reset

        self.send_command("start_program")
        self.read_serial()

    def send_command(self, command):
        self.ser.write((command + '\n').encode())
        self.get_logger().info(f"Sent command: {command}")

    def read_serial(self):
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                self.get_logger().info(f"[Arduino]: {line}")
                if line == "Program done":
                    break

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
