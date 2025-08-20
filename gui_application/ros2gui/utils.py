import os
import subprocess
import serial.tools.list_ports

def start_process(processes, key, command):
    if key in processes and processes[key].poll() is None:
        print(f"[INFO] {key} is already running.")
        return
    print(f"[INFO] Starting {key}...")
    proc = subprocess.Popen(command, shell=True)
    processes[key] = proc

def stop_process(processes, key, match=None):
    proc = processes.get(key)
    if proc and proc.poll() is None:
        print(f"[INFO] Stopping {key} (from GUI)...")
        proc.terminate()
    else:
        print(f"[INFO] {key} not tracked or already stopped. Trying pattern match...")
        if match:
            subprocess.run(['taskkill', '/F', '/IM', match], shell=True)
            print(f"[INFO] Killed matching process: {match}")
        else:
            print(f"[WARN] No pattern provided for {key}")

def check_arduino_connection():
    """Check if Arduino is connected to a Windows COM port"""
    ports = [port.device for port in serial.tools.list_ports.comports()]
    # Look for common Arduino COM ports or specific device descriptions
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description or 'CH340' in port.description or 'USB Serial' in port.description:
            return True
    return len(ports) > 0  # Fallback: return True if any COM port is available

def check_camera_connection():
    """Check if Intel RealSense camera is connected (Windows)"""
    try:
        # Try using Intel RealSense SDK first
        try:
            import pyrealsense2 as rs
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) > 0:
                return True
        except ImportError:
            pass
        
        # Fallback: Check using Windows Device Manager via WMI
        try:
            import wmi
            c = wmi.WMI()
            for device in c.Win32_PnPEntity():
                if device.Name and ('Intel' in device.Name and 'RealSense' in device.Name):
                    return True
                if device.Name and 'Depth Camera' in device.Name:
                    return True
        except ImportError:
            pass
        
        # Final fallback: Try to open any camera with OpenCV
        import cv2
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.release()
            return True
        
        return False
    except Exception as e:
        print(f"[ERROR] Camera check failed: {e}")
        return False

def check_scanner_connection():
    """Check if scanCONTROL scanner SDK is available and potentially connected"""
    try:
        import sys
        # Update this path to your Windows SDK location
        sys.path.insert(0, 'C:\\scanCONTROL-Windows-SDK\\python_bindings')
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
