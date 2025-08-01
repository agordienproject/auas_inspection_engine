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

def check_scanner_connection():
    """Check if scanCONTROL scanner SDK is available and potentially connected"""
    try:
        import sys
        sys.path.insert(0, '/home/agordien/projects/auas_inspection_engine/scanCONTROL-Linux-SDK-1-0-1/python_bindings')
        import pylinllt as llt
        
        # Try to discover devices (quick check)
        import ctypes as ct
        available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
        available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))
        
        ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
        return ret >= 0  # SDK available, device detection works
        
    except ImportError:
        print("[WARN] scanCONTROL SDK not found")
        return False
    except Exception as e:
        print(f"[WARN] Scanner check failed: {e}")
        return False
