#!/usr/bin/env python3
"""
xArm Simple Movement Program
Moves the robot to an inspection position and returns home
"""

import os
import sys
import time

# Add xArm SDK to path if needed
try:
    from xarm.wrapper import XArmAPI
except ImportError:
    print("Error: xArm Python SDK not found")
    print("Install with: pip install xarm-python-sdk")
    sys.exit(1)


def main():
    # Get IP address from command line argument
    if len(sys.argv) >= 2:
        ip = sys.argv[1]
    else:
        print("Error: IP address required")
        print("Usage: python move_arm.py <robot_ip>")
        sys.exit(1)
    
    print(f"Connecting to xArm at {ip}")
    
    # Initialize xArm
    arm = XArmAPI(ip)
    
    try:
        # Enable motion and set initial state
        print("Initializing robot...")
        arm.motion_enable(enable=True)
        arm.set_mode(0)  # Position mode
        arm.set_state(state=0)  # Ready state
        
        # Move to home position first
        print("Moving to home position...")
        arm.move_gohome(wait=True)
        time.sleep(1)
        
        # Move to inspection position (220, 0, 400, 120, 0, 0)
        print("Moving to inspection position...")
        ret = arm.set_position(300, 0, 150, 180, 0, 0, speed=100, wait=True)
        if ret != 0:
            print(f"Error moving to inspection position: {ret}")
            return False
        
        # Wait at inspection position
        print("Waiting at inspection position...")
        time.sleep(2)
        
        # Return to home position
        print("Returning to home position...")
        arm.move_gohome(wait=True)
        
        print("Program completed successfully!")
        return True
        
    except Exception as e:
        print(f"Error during program execution: {e}")
        return False
        
    finally:
        # Clean up
        try:
            arm.motion_enable(enable=False)
            arm.disconnect()
        except:
            pass


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
