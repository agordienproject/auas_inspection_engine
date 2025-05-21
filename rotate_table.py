# rotate_table.py
import serial
import time
import sys
import signal

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)
ser.write(b"rotate_table\n")
print("[✓] Sent: rotate_table")

def handle_exit(signum, frame):
    print("[!] Stopping table...")
    ser.write(b"stop_table\n")
    time.sleep(0.5)
    ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

try:
    while True:
        if ser.in_waiting:
            print(ser.readline().decode().strip())
except KeyboardInterrupt:
    handle_exit(None, None)
