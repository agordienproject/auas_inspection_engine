"""
Camera System using Intel RealSense SDK for Windows (No ROS2)
"""
import logging
import time
import cv2
import os
import datetime
import threading
import numpy as np
from typing import Dict, Any, Optional

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[WARN] Intel RealSense SDK not available, camera system will use OpenCV fallback")

from systems.base_system import BaseSystem


class RealSenseCameraController:
    """Controller class for Intel RealSense camera operations"""
    
    def __init__(self):
        self.connected = False
        self.capturing = False
        self.device_id = None
        self.pipeline = None
        self.config = None
        self.current_color_frame = None
        self.current_depth_frame = None
        self.logger = logging.getLogger(__name__)
        
    def connect(self, device_id: str = "Intel Corp"):
        """Connect to Intel RealSense camera"""
        try:
            self.device_id = device_id
            
            if not REALSENSE_AVAILABLE:
                # Fallback to OpenCV
                return self._connect_opencv()
            
            # Initialize RealSense pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Test camera availability
            ctx = rs.context()
            devices = ctx.query_devices()
            
            if len(devices) == 0:
                self.logger.warning("No RealSense devices found")
                return self._connect_opencv()
            
            # Start streaming
            profile = self.pipeline.start(self.config)
            self.connected = True
            self.logger.info(f"Successfully connected to RealSense camera: {devices[0].get_info(rs.camera_info.name)}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to RealSense camera: {e}")
            return self._connect_opencv()
    
    def _connect_opencv(self):
        """Fallback connection using OpenCV"""
        try:
            # Try to find any available camera
            for i in range(5):  # Check first 5 camera indices
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    cap.release()
                    self.connected = True
                    self.camera_index = i
                    self.logger.info(f"Connected to camera using OpenCV (index {i})")
                    return True
            
            self.logger.error("No cameras found using OpenCV")
            return False
            
        except Exception as e:
            self.logger.error(f"OpenCV camera connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from camera"""
        try:
            self.stop_camera()
            if self.pipeline:
                self.pipeline.stop()
                self.pipeline = None
            
            self.connected = False
            return True
        except Exception as e:
            self.logger.error(f"Error disconnecting camera: {e}")
            self.connected = False
            return False
    
    def start_camera(self):
        """Start camera capture"""
        if not self.connected:
            raise ConnectionError("Camera not connected")
        
        try:
            if REALSENSE_AVAILABLE and self.pipeline:
                # RealSense is already streaming from connect()
                self.capturing = True
                self.logger.info("RealSense camera streaming started")
                return True
            else:
                # Use OpenCV fallback
                return self._start_camera_opencv()
                
        except Exception as e:
            self.logger.error(f"Failed to start camera: {e}")
            return False
    
    def _start_camera_opencv(self):
        """Start camera using OpenCV fallback"""
        try:
            self.cap = cv2.VideoCapture(getattr(self, 'camera_index', 0))
            if self.cap.isOpened():
                self.capturing = True
                self.logger.info("OpenCV camera streaming started")
                return True
            return False
        except Exception as e:
            self.logger.error(f"Failed to start OpenCV camera: {e}")
            return False
    
    def stop_camera(self):
        """Stop camera capture"""
        try:
            self.capturing = False
            if hasattr(self, 'cap'):
                self.cap.release()
                delattr(self, 'cap')
            self.logger.info("Camera streaming stopped")
            return True
        except Exception as e:
            self.logger.error(f"Error stopping camera: {e}")
            return False
    
    def capture_image(self, filename: str = None):
        """Capture a single image"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")
        
        try:
            if REALSENSE_AVAILABLE and self.pipeline:
                # Capture using RealSense
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    raise RuntimeError("Failed to capture color frame")
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                if filename:
                    cv2.imwrite(filename, color_image)
                    self.logger.info(f"Image saved to {filename}")
                
                return color_image
            else:
                # Use OpenCV fallback
                if hasattr(self, 'cap'):
                    ret, frame = self.cap.read()
                    if ret:
                        if filename:
                            cv2.imwrite(filename, frame)
                            self.logger.info(f"Image saved to {filename}")
                        return frame
                    else:
                        raise RuntimeError("Failed to capture frame")
                else:
                    raise RuntimeError("Camera not initialized")
                    
        except Exception as e:
            self.logger.error(f"Failed to capture image: {e}")
            raise
    
    def capture_depth_image(self, filename: str = None):
        """Capture depth image (only available with RealSense)"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")
        
        if not REALSENSE_AVAILABLE or not self.pipeline:
            raise RuntimeError("Depth capture requires Intel RealSense camera")
        
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                raise RuntimeError("Failed to capture depth frame")
            
            # Convert to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            
            if filename:
                # Save as 16-bit PNG to preserve depth data
                cv2.imwrite(filename, depth_image)
                self.logger.info(f"Depth image saved to {filename}")
            
            return depth_image
            
        except Exception as e:
            self.logger.error(f"Failed to capture depth image: {e}")
            raise
    
    def get_camera_info(self):
        """Get camera information"""
        if REALSENSE_AVAILABLE and self.pipeline:
            try:
                ctx = rs.context()
                devices = ctx.query_devices()
                if len(devices) > 0:
                    device = devices[0]
                    return {
                        'name': device.get_info(rs.camera_info.name),
                        'serial': device.get_info(rs.camera_info.serial_number),
                        'firmware': device.get_info(rs.camera_info.firmware_version),
                        'type': 'Intel RealSense'
                    }
            except Exception:
                pass
        
        return {
            'name': 'Generic Camera',
            'type': 'OpenCV',
            'index': getattr(self, 'camera_index', 0)
        }


class CameraSystem(BaseSystem):
    """Camera system for capturing images and depth data"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.controller = RealSenseCameraController()
        self.logger = logging.getLogger(__name__)
        self.current_inspection_folder = None  # Will be set by system manager
        
    def set_inspection_folder(self, folder_path: str):
        """Set the current inspection folder path for output"""
        self.current_inspection_folder = folder_path
        self.logger.info(f"Camera system inspection folder set to: {folder_path}")
        
    def connect(self) -> bool:
        """Connect to camera system"""
        try:
            device_id = self.config.get('device_id', 'Intel Corp')
            success = self.controller.connect(device_id)
            if success:
                self.connected = True
                self.logger.info("Camera system connected successfully")
            return success
        except Exception as e:
            self.logger.error(f"Failed to connect to camera: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from camera system"""
        try:
            success = self.controller.disconnect()
            self.connected = False
            self.logger.info("Camera system disconnected")
            return success
        except Exception as e:
            self.logger.error(f"Error disconnecting camera: {e}")
            return False
    
    def test_connection(self) -> bool:
        """Test camera connection"""
        if not self.connected:
            return False
        
        try:
            # Try to start and immediately stop camera
            if self.controller.start_camera():
                self.controller.stop_camera()
                return True
            return False
        except Exception as e:
            self.logger.error(f"Camera connection test failed: {e}")
            return False
    
    def execute_action(self, action: str, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """Execute camera action"""
        if parameters is None:
            parameters = {}
        
        try:
            if action == "capture_image":
                return self._capture_image(parameters)
            elif action == "capture_depth":
                return self._capture_depth(parameters)
            elif action == "start_stream":
                return self._start_stream(parameters)
            elif action == "stop_stream":
                return self._stop_stream(parameters)
            elif action == "get_info":
                return self._get_camera_info()
            else:
                raise ValueError(f"Unknown camera action: {action}")
                
        except Exception as e:
            self.logger.error(f"Camera action '{action}' failed: {e}")
            return {"success": False, "error": str(e)}
    
    def _capture_image(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Capture a single image"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.controller.start_camera()
            
            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"camera_image_{timestamp}.jpg"
            
            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            
            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
            
            # Capture image
            image = self.controller.capture_image(filename)
            
            return {
                "success": True,
                "filename": filename,
                "image_shape": image.shape if image is not None else None,
                "message": f"Image captured successfully: {filename}"
            }
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _capture_depth(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Capture depth image"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.controller.start_camera()
            
            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"depth_image_{timestamp}.png"
            
            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            
            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
            
            # Capture depth image
            depth_image = self.controller.capture_depth_image(filename)
            
            return {
                "success": True,
                "filename": filename,
                "depth_shape": depth_image.shape if depth_image is not None else None,
                "message": f"Depth image captured successfully: {filename}"
            }
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _start_stream(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Start camera streaming"""
        try:
            success = self.controller.start_camera()
            return {
                "success": success,
                "message": "Camera streaming started" if success else "Failed to start camera streaming"
            }
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _stop_stream(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Stop camera streaming"""
        try:
            success = self.controller.stop_camera()
            return {
                "success": success,
                "message": "Camera streaming stopped" if success else "Failed to stop camera streaming"
            }
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _get_camera_info(self) -> Dict[str, Any]:
        """Get camera information"""
        try:
            info = self.controller.get_camera_info()
            return {
                "success": True,
                "camera_info": info
            }
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def cleanup(self):
        """Clean up camera resources"""
        try:
            self.controller.stop_camera()
            self.controller.disconnect()
            self.logger.info("Camera system cleanup completed")
        except Exception as e:
            self.logger.error(f"Error during camera cleanup: {e}")
