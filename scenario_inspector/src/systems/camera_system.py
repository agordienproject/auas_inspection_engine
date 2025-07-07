"""
Camera System using ROS2 connection functions from GUI application
"""
import logging
import subprocess
import time
import cv2
import os
import datetime
import threading
import signal
from typing import Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARN] ROS2 not available, camera system will use fallback mode")

from systems.base_system import BaseSystem


class ROS2CameraNode(Node):
    """ROS2 Node for camera operations"""
    
    def __init__(self):
        super().__init__('camera_system_node')
        self.bridge = CvBridge()
        self.current_image = None
        self.image_subscription = None
        self.logger = self.get_logger()
        
    def start_image_subscription(self):
        """Start subscribing to camera images"""
        try:
            self.image_subscription = self.create_subscription(
                Image,
                '/camera/camera/color/image_raw',
                self.image_callback,
                10
            )
            self.logger.info("Started camera image subscription")
        except Exception as e:
            self.logger.error(f"Failed to start image subscription: {e}")
            raise
    
    def stop_image_subscription(self):
        """Stop subscribing to camera images"""
        if self.image_subscription:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
            self.logger.info("Stopped camera image subscription")
    
    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
        except Exception as e:
            self.logger.error(f"Failed to convert image: {e}")


class CameraController:
    """Controller class for camera operations using ROS2"""
    
    def __init__(self):
        self.connected = False
        self.camera_process = None
        self.capturing = False
        self.device_id = None
        self.ros2_node = None
        self.ros2_thread = None
        self.ros2_initialized = False
        
    def connect(self, device_id: str = "Intel Corp"):
        """Connect to camera system"""
        try:
            self.device_id = device_id
            
            if not ROS2_AVAILABLE:
                # Fallback to OpenCV
                return self._connect_opencv()
            
            # Initialize ROS2 if not already done
            if not self.ros2_initialized:
                if not rclpy.ok():
                    rclpy.init()
                self.ros2_initialized = True
            
            # Test camera availability using lsusb (like in utils.py)
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            if "Intel Corp." in result.stdout or "RealSense" in result.stdout:
                self.connected = True
                return True
            else:
                print(f"[WARN] Camera device '{device_id}' not found in USB devices")
                return False
                
        except Exception as e:
            self.connected = False
            raise e
    
    def _connect_opencv(self):
        """Fallback connection using OpenCV"""
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                self.connected = True
                return True
        return False
    
    def disconnect(self):
        """Disconnect from camera"""
        try:
            self.stop_camera()
            if self.ros2_node:
                self.ros2_node.destroy_node()
                self.ros2_node = None
            
            self.connected = False
            return True
        except Exception:
            self.connected = False
            return False
    
    def start_camera(self):
        """Start camera capture using ROS2"""
        if not self.connected:
            raise ConnectionError("Camera not connected")
        
        try:
            if not ROS2_AVAILABLE:
                return self._start_camera_opencv()
            
            # Launch RealSense camera via ROS2 (similar to GUI implementation)
            cmd = [
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"
            ]
            
            self.camera_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            
            # Give time for camera to start
            time.sleep(3)
            
            # Create ROS2 node for image subscription
            if not self.ros2_node:
                self.ros2_node = ROS2CameraNode()
                
                # Start spinning the node in a separate thread
                self.ros2_thread = threading.Thread(target=self._spin_ros2_node)
                self.ros2_thread.daemon = True
                self.ros2_thread.start()
            
            # Start image subscription
            self.ros2_node.start_image_subscription()
            
            # Wait a bit more for subscription to be ready
            time.sleep(2)
            
            self.capturing = True
            return True
            
        except Exception as e:
            self.capturing = False
            raise RuntimeError(f"Failed to start camera: {e}")
    
    def _start_camera_opencv(self):
        """Fallback start using OpenCV"""
        self.capturing = True
        return True
    
    def _spin_ros2_node(self):
        """Spin the ROS2 node in a separate thread"""
        try:
            rclpy.spin(self.ros2_node)
        except Exception as e:
            print(f"[ERROR] ROS2 spinning error: {e}")
    
    def stop_camera(self):
        """Stop camera capture"""
        try:
            # Stop ROS2 subscription
            if self.ros2_node:
                self.ros2_node.stop_image_subscription()
            
            # Kill the ROS2 camera process
            if self.camera_process:
                try:
                    # Kill the process group
                    os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass  # Process already terminated
                
                # Also try pkill as fallback
                subprocess.run([
                    "pkill", "-f", "ros2 launch realsense2_camera rs_launch.py"
                ], check=False)
                
                self.camera_process = None
            
            self.capturing = False
            return True
            
        except Exception as e:
            print(f"[WARN] Error stopping camera: {e}")
            self.capturing = False
            return False
    
    def capture_image(self, output_path: str = None):
        """Capture a single image from ROS2 camera feed"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")
        
        try:
            if not ROS2_AVAILABLE or not self.ros2_node:
                # Fallback to OpenCV capture
                return self._capture_image_opencv(output_path)
            
            # Wait for image from ROS2 subscription
            max_wait_time = 5.0  # seconds
            wait_interval = 0.1
            waited = 0.0
            
            while self.ros2_node.current_image is None and waited < max_wait_time:
                time.sleep(wait_interval)
                waited += wait_interval
            
            if self.ros2_node.current_image is None:
                raise RuntimeError("No image received from camera within timeout")
            
            # Get the current image
            frame = self.ros2_node.current_image.copy()
            
            if output_path is None:
                # Generate default output path
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = f"capture_{timestamp}.png"
            
            # Save the image
            cv2.imwrite(output_path, frame)
            return output_path
            
        except Exception as e:
            raise RuntimeError(f"Failed to capture image: {e}")
    
    def _capture_image_opencv(self, output_path: str = None):
        """Fallback image capture using OpenCV"""
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera for capture")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            raise RuntimeError("Failed to capture frame")
        
        if output_path is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f"capture_{timestamp}.png"
        
        cv2.imwrite(output_path, frame)
        return output_path
    
    def test_connection(self):
        """Test if connection is working"""
        if not self.connected:
            return False
        
        try:
            if not ROS2_AVAILABLE:
                # Fallback test using OpenCV
                cap = cv2.VideoCapture(0)
                is_open = cap.isOpened()
                cap.release()
                return is_open
            
            # Test using lsusb like the GUI implementation
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            for line in result.stdout.splitlines():
                if "Intel Corp." in line or "RealSense" in line:
                    return True
            return False
            
        except Exception as e:
            print(f"[ERROR] Camera connection test failed: {e}")
            return False


class CameraSystem(BaseSystem):
    """Camera system implementation using GUI connection functions"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.camera_controller = CameraController()
        self.logger = logging.getLogger(__name__)
        self.current_inspection_folder = None  # Will be set by system manager
        
    def set_inspection_folder(self, folder_path: str):
        """Set the current inspection folder path for output"""
        self.current_inspection_folder = folder_path
        self.logger.info(f"Camera system inspection folder set to: {folder_path}")
        
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
            action = step_config.get('action', 'take_photo')
            
            if action == 'take_photo':
                return self._take_photo(step_config)
            elif action == 'take_video':
                return self._take_video(step_config)
            elif action == 'show_camera':
                return self._show_camera(step_config)
            # Legacy support for old action names
            elif action == 'capture':
                return self._take_photo(step_config)
            elif action == 'start_recording':
                return self._take_video(step_config)
            elif action == 'stop_recording':
                return self._stop_recording(step_config)
            else:
                raise ValueError(f"Unknown camera action: {action}")
                
        except Exception as e:
            self.logger.error(f"Camera step execution failed: {e}")
            raise
    
    def _get_output_path(self, step_config: Dict[str, Any], default_filename: str) -> str:
        """Determine the output path based on step configuration"""
        path_config = step_config.get('path', 'classic')
        
        if path_config == 'classic':
            # Use the inspection folder if available
            if self.current_inspection_folder:
                output_dir = os.path.join(self.current_inspection_folder, step_config.get('name', 'camera_data'))
            else:
                # Fallback to default output directory
                output_dir = os.path.join('./output', 'camera_data')
        else:
            # Use custom path
            output_dir = path_config
        
        os.makedirs(output_dir, exist_ok=True)
        return os.path.join(output_dir, default_filename)
    
    def _take_photo(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Take a single photo using ROS2 camera feed"""
        self.logger.info("Taking photo with ROS2 camera")
        
        # Start camera if not already capturing
        if not self.camera_controller.capturing:
            self.logger.info("Starting camera for photo capture")
            self.camera_controller.start_camera()
            time.sleep(3)  # Wait for camera to stabilize and ROS2 to connect
        
        # Generate output path
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f'photo_{timestamp}.jpg'
        output_path = self._get_output_path(step_config, default_filename)
        
        try:
            saved_path = self.camera_controller.capture_image(output_path)
            
            result = {
                'status': 'success',
                'message': 'Photo captured successfully using ROS2 camera',
                'data': {
                    'image_path': saved_path,
                    'timestamp': self.get_timestamp(),
                    'action': 'take_photo',
                    'ros2_enabled': ROS2_AVAILABLE,
                    'device_id': self.camera_controller.device_id
                }
            }
            
            # Add file saving info if specified
            if step_config.get('saving_file', False):
                result['file_saved'] = True
                result['output_path'] = saved_path
            
            self.logger.info(f"Photo saved to: {saved_path}")
            return result
            
        except Exception as e:
            self.logger.error(f"Photo capture failed: {e}")
            raise RuntimeError(f"Photo capture failed: {e}")
    
    def _take_video(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Take a video with specified recording time"""
        self.logger.info("Starting video recording")
        
        recording_time = step_config.get('recording_time', 10)  # Default 10 seconds
        parameters = step_config.get('parameters', {})
        resolution = parameters.get('resolution', '1920x1080')
        fps = parameters.get('fps', 30)
        
        # Generate output path
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f'video_{timestamp}.mp4'
        output_path = self._get_output_path(step_config, default_filename)
        
        try:
            # Start camera
            success = self.camera_controller.start_camera()
            if not success:
                raise RuntimeError("Failed to start camera for recording")
            
            # Record video for specified time using OpenCV
            saved_path = self._record_video_opencv(output_path, recording_time, resolution, fps)
            
            result = {
                'status': 'success',
                'message': f'Video recorded successfully for {recording_time} seconds',
                'data': {
                    'video_path': saved_path,
                    'recording_time': recording_time,
                    'resolution': resolution,
                    'fps': fps,
                    'timestamp': self.get_timestamp(),
                    'action': 'take_video'
                }
            }
            
            # Add file saving info if specified
            if step_config.get('saving_file', False):
                result['file_saved'] = True
                result['output_path'] = saved_path
            
            return result
            
        except Exception as e:
            raise RuntimeError(f"Video recording failed: {e}")
    
    def _show_camera(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Enable camera in the GUI (show live feed)"""
        self.logger.info("Showing camera in GUI")
        
        try:
            # Start camera if not already capturing
            if not self.camera_controller.capturing:
                success = self.camera_controller.start_camera()
                if not success:
                    raise RuntimeError("Failed to start camera for GUI display")
            
            # For GUI integration, this would typically signal the GUI to show camera feed
            # For now, we'll just ensure the camera is running
            
            return {
                'status': 'success',
                'message': 'Camera enabled in GUI',
                'data': {
                    'action': 'show_camera',
                    'camera_status': 'active',
                    'timestamp': self.get_timestamp()
                }
            }
            
        except Exception as e:
            raise RuntimeError(f"Failed to show camera in GUI: {e}")
    
    def _record_video_opencv(self, output_path: str, duration: int, resolution: str, fps: int) -> str:
        """Record video using ROS2 camera feed or OpenCV fallback"""
        try:
            # Parse resolution
            width, height = map(int, resolution.split('x'))
            
            # Define codec and create VideoWriter
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
            
            start_time = time.time()
            frame_count = 0
            expected_frames = duration * fps
            
            self.logger.info(f"Recording video: {duration}s at {fps}fps ({expected_frames} frames)")
            
            if ROS2_AVAILABLE and self.camera_controller.ros2_node:
                # Use ROS2 camera feed
                while (time.time() - start_time) < duration:
                    if self.camera_controller.ros2_node.current_image is not None:
                        frame = self.camera_controller.ros2_node.current_image.copy()
                        
                        # Resize frame to target resolution
                        frame_resized = cv2.resize(frame, (width, height))
                        out.write(frame_resized)
                        frame_count += 1
                        
                        # Display progress
                        if frame_count % (fps * 2) == 0:  # Every 2 seconds
                            elapsed = time.time() - start_time
                            self.logger.info(f"Recording... {elapsed:.1f}s / {duration}s")
                    
                    # Control frame rate
                    time.sleep(1.0 / fps)
            else:
                # Fallback to OpenCV
                cap = cv2.VideoCapture(0)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                
                if not cap.isOpened():
                    raise RuntimeError("Cannot open camera for video recording")
                
                while (time.time() - start_time) < duration:
                    ret, frame = cap.read()
                    if not ret:
                        self.logger.warning("Failed to read frame, continuing...")
                        continue
                    
                    out.write(frame)
                    frame_count += 1
                    
                    # Display progress
                    if frame_count % (fps * 2) == 0:  # Every 2 seconds
                        elapsed = time.time() - start_time
                        self.logger.info(f"Recording... {elapsed:.1f}s / {duration}s")
                
                cap.release()
            
            # Release video writer
            out.release()
            
            self.logger.info(f"Video recording completed: {frame_count} frames, saved to {output_path}")
            return output_path
            
        except Exception as e:
            self.logger.error(f"Video recording error: {e}")
            raise
    
    # Legacy method for backward compatibility
    def _capture_image(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Legacy method - redirects to _take_photo"""
        return self._take_photo(step_config)
    
    def _start_recording(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Legacy method - start video recording"""
        self.logger.info("Starting video recording (legacy method)")
        
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
        """Legacy method - stop video recording"""
        self.logger.info("Stopping video recording (legacy method)")
        
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
            
            # Shutdown ROS2 if initialized
            if self.camera_controller.ros2_initialized and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception as e:
                    self.logger.warning(f"Error shutting down ROS2: {e}")
            
            self.logger.info("Camera system shut down")
        except Exception as e:
            self.logger.error(f"Error shutting down camera system: {e}")
