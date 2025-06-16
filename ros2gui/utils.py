import os
import signal
import subprocess
import serial.tools.list_ports

def start_process(processes, key, command):
    if key in processes and processes[key].poll() is None:
        print(f"[INFO] {key} is already running.")
        return
    print(f"[INFO] Starting {key}...")
    proc = subprocess.Popen(command, preexec_fn=os.setsid)
    processes[key] = proc

def stop_process(processes, key, match=None):
    proc = processes.get(key)
    if proc and proc.poll() is None:
        print(f"[INFO] Stopping {key} (from GUI)...")
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    else:
        print(f"[INFO] {key} not tracked or already stopped. Trying pattern match...")
        if match:
            subprocess.run(['pkill', '-f', match])
            print(f"[INFO] Killed matching process: {match}")
        else:
            print(f"[WARN] No pattern provided for {key}")

def check_arduino_connection():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return "/dev/ttyACM0" in ports

def check_camera_connection():
    try:
        output = subprocess.check_output(['lsusb']).decode()
        for line in output.splitlines():
            if "Intel Corp." in line or "RealSense" in line:
                return True
        return False
    except Exception as e:
        print(f"[ERROR] USB check failed: {e}")
        return False
