"""
Camera System using ROS2 connection functions from GUI application
"""
import logging
import subprocess
import time
import cv2
import os
import datetime
from typing import Dict, Any, Optional

from systems.base_system import BaseSystem


class CameraController:
    """Controller class for camera operations (from GUI application)"""
    
    def __init__(self):
        self.connected = False
        self.camera_process = None
        self.capturing = False
        self.device_id = None
        
    def connect(self, device_id: str = "Intel Corp"):
        """Connect to camera system"""
        try:
            self.device_id = device_id
            
            # Test if camera is available by trying to open it
            # First, try to find Intel RealSense camera
            cap = cv2.VideoCapture(0)  # Try default camera first
            
            if cap.isOpened():
                # Test if we can read a frame
                ret, frame = cap.read()
                cap.release()
                
                if ret:
                    self.connected = True
                    return True
            
            return False
                
        except Exception as e:
            self.connected = False
            raise e
    
    def disconnect(self):
        """Disconnect from camera"""
        try:
            if self.camera_process:
                self.camera_process.terminate()
                self.camera_process = None
            self.connected = False
            self.capturing = False
            return True
        except Exception:
            self.connected = False
            return False
    
    def start_camera(self):
        """Start camera capture"""
        if not self.connected:
            raise ConnectionError("Camera not connected")
        
        try:
            # Launch RealSense camera via ROS2 (similar to GUI implementation)
            cmd = [
                "gnome-terminal", "--", "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"
            ]
            
            self.camera_process = subprocess.Popen(cmd)
            self.capturing = True
            
            # Give some time for the camera to start
            time.sleep(3)
            
            return True
            
        except Exception as e:
            self.capturing = False
            raise RuntimeError(f"Failed to start camera: {e}")
    
    def stop_camera(self):
        """Stop camera capture"""
        try:
            if self.camera_process:
                # Kill the ROS2 camera process
                subprocess.run([
                    "pkill", "-f", "ros2 launch realsense2_camera rs_launch.py"
                ], check=False)
                self.camera_process = None
            
            self.capturing = False
            return True
            
        except Exception:
            self.capturing = False
            return False
    
    def capture_image(self, output_path: str = None):
        """Capture a single image"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")
        
        try:
            # For now, capture using OpenCV as fallback
            cap = cv2.VideoCapture(0)
            
            if not cap.isOpened():
                raise RuntimeError("Cannot open camera for capture")
            
            ret, frame = cap.read()
            cap.release()
            
            if not ret:
                raise RuntimeError("Failed to capture frame")
            
            if output_path is None:
                # Generate default output path
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = f"capture_{timestamp}.png"
            
            cv2.imwrite(output_path, frame)
            return output_path
            
        except Exception as e:
            raise RuntimeError(f"Failed to capture image: {e}")
    
    def test_connection(self):
        """Test if connection is working"""
        if not self.connected:
            return False
        
        try:
            # Test by trying to open camera
            cap = cv2.VideoCapture(0)
            is_open = cap.isOpened()
            cap.release()
            return is_open
        except Exception:
            return False


class CameraSystem(BaseSystem):
    """Camera system implementation using GUI connection functions"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.camera_controller = CameraController()
        self.logger = logging.getLogger(__name__)
        
    def initialize(self) -> bool:
        """Initialize the camera system"""
        try:
            self.logger.info(f"Initializing camera system: {self.name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize camera system: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to the camera"""
        try:
            device_id = self.config.get('device_id', 'Intel Corp')
            
            self.logger.info(f"Attempting to connect to camera: {device_id}")
            
            success = self.camera_controller.connect(device_id)
            if success:
                self.logger.info("Camera connected successfully")
                return True
            else:
                self.logger.error("Failed to connect to camera")
                return False
                
        except Exception as e:
            self.logger.error(f"Camera connection error: {e}")
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from the camera"""
        try:
            success = self.camera_controller.disconnect()
            if success:
                self.logger.info("Camera disconnected successfully")
            return success
        except Exception as e:
            self.logger.error(f"Camera disconnection error: {e}")
            return False
    
    def test_connection(self) -> Dict[str, Any]:
        """Test camera connection"""
        try:
            if self.camera_controller.test_connection():
                return {
                    'status': 'available',
                    'message': 'Camera connected and ready',
                    'details': {
                        'device_id': self.camera_controller.device_id,
                        'capturing': self.camera_controller.capturing
                    }
                }
            else:
                # Try to connect
                if self.connect():
                    return {
                        'status': 'available',
                        'message': 'Camera connection established',
                        'details': {
                            'device_id': self.camera_controller.device_id,
                            'capturing': self.camera_controller.capturing
                        }
                    }
                else:
                    return {
                        'status': 'not_available',
                        'message': 'Camera connection failed - check if camera is connected'
                    }
        except Exception as e:
            self.logger.error(f"Camera connection test failed: {e}")
            return {
                'status': 'error',
                'message': f'Camera test error: {str(e)}'
            }
    
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the camera"""
        step_name = step_config.get('name', 'unknown')
        self.logger.info(f"Executing camera step: {step_name}")
        
        try:
            # Ensure connection
            if not self.camera_controller.connected:
                if not self.connect():
                    raise ConnectionError("Cannot connect to camera")
            
            # Execute the step based on step configuration
            action = step_config.get('action', 'capture')
            
            if action == 'capture':
                return self._capture_image(step_config)
            elif action == 'start_recording':
                return self._start_recording(step_config)
            elif action == 'stop_recording':
                return self._stop_recording(step_config)
            else:
                raise ValueError(f"Unknown camera action: {action}")
                
        except Exception as e:
            self.logger.error(f"Camera step execution failed: {e}")
            raise
    
    def _capture_image(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Capture a single image"""
        self.logger.info("Capturing image")
        
        # Start camera if not already capturing
        if not self.camera_controller.capturing:
            self.camera_controller.start_camera()
            time.sleep(2)  # Wait for camera to stabilize
        
        # Generate output path
        output_dir = step_config.get('output_dir', './output/images')
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        image_name = step_config.get('image_name', f'capture_{timestamp}.png')
        output_path = os.path.join(output_dir, image_name)
        
        try:
            saved_path = self.camera_controller.capture_image(output_path)
            
            return {
                'status': 'success',
                'message': 'Image captured successfully',
                'data': {
                    'image_path': saved_path,
                    'timestamp': self.get_timestamp()
                }
            }
            
        except Exception as e:
            raise RuntimeError(f"Image capture failed: {e}")
    
    def _start_recording(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Start video recording"""
        self.logger.info("Starting video recording")
        
        success = self.camera_controller.start_camera()
        if not success:
            raise RuntimeError("Failed to start camera for recording")
        
        return {
            'status': 'success',
            'message': 'Video recording started',
            'data': {
                'action': 'start_recording',
                'timestamp': self.get_timestamp()
            }
        }
    
    def _stop_recording(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Stop video recording"""
        self.logger.info("Stopping video recording")
        
        success = self.camera_controller.stop_camera()
        if not success:
            raise RuntimeError("Failed to stop camera recording")
        
        return {
            'status': 'success',
            'message': 'Video recording stopped',
            'data': {
                'action': 'stop_recording',
                'timestamp': self.get_timestamp()
            }
        }
    
    def shutdown(self):
        """Shutdown the camera system"""
        try:
            # Stop recording if active
            if self.camera_controller.capturing:
                self.camera_controller.stop_camera()
            
            # Disconnect
            self.disconnect()
            self.logger.info("Camera system shut down")
        except Exception as e:
            self.logger.error(f"Error shutting down camera system: {e}")
