"""
Camera System using Intel RealSense SDK for Windows (No ROS2)
"""
import os
import datetime
import time
import logging
from typing import Dict, Any
from systems.base_system import BaseSystem
import cv2
import numpy as np

# Module-level logger
_logger = logging.getLogger(__name__)

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    _logger.warning("Intel RealSense SDK not available, camera system will use OpenCV fallback")

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
        self.logger.debug(
            "RealSenseCameraController initialized with default state: connected=%s, capturing=%s",
            self.connected,
            self.capturing,
        )

        # Video recording variables
        self.is_recording = False
        self.video_writer = None
        self.video_filename = None
        self.video_fps = 30
        self.video_quality = "medium"
        self.video_duration = None
        self.video_start_time = None
        self._frames_written_count = 0

    def connect(self, device_id: str = "Intel Corp", resolution: str = "high"):
        """Connect to Intel RealSense camera"""
        try:
            self.logger.debug("Attempting RealSense connect: device_id=%s, resolution=%s", device_id, resolution)
            self.device_id = device_id

            if not REALSENSE_AVAILABLE:
                # If RealSense SDK is not available, we can't connect to RealSense cameras
                self.logger.error("RealSense SDK not available - cannot connect to Intel RealSense camera")
                return False

            # Initialize RealSense pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # Test camera availability and validate device
            ctx = rs.context()
            devices = ctx.query_devices()
            self.logger.debug("Detected %d RealSense device(s)", len(devices))

            if len(devices) == 0:
                self.logger.error("No RealSense devices found")
                return False

            # Check if any of the detected devices match our required device_id
            target_device = None
            for device in devices:
                device_name = device.get_info(rs.camera_info.name)
                self.logger.debug("Found RealSense device: %s", device_name)

                # Check if this device matches our target device_id
                if device_id in device_name:
                    target_device = device
                    self.logger.debug("Target device found: %s", device_name)
                    break

            if target_device is None:
                self.logger.error("Required camera device not found: %s", device_id)
                self.logger.error("Available devices: %s", [dev.get_info(1) for dev in devices])
                return False

            # Configure streams based on resolution setting
            self.logger.debug("Configuring streams for resolution='%s'", resolution)
            if resolution == "ultra":
                # Ultra high resolution - 1920x1080
                self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            elif resolution == "high":
                # High resolution - 1280x720
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            elif resolution == "medium":
                # Medium resolution - 848x480
                self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
            else:
                # Standard resolution - 640x480
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # Enable the specific device we want to use
            self.config.enable_device(target_device.get_info(rs.camera_info.serial_number))

            # Start streaming
            profile = self.pipeline.start(self.config)
            self.logger.debug("RealSense pipeline started")

            # Configure camera settings for better quality
            try:
                # Get the color sensor
                color_sensor = profile.get_device().first_color_sensor()

                # Enable auto-exposure initially
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)

                # Set high gain for better low-light performance
                if color_sensor.supports(rs.option.gain):
                    color_sensor.set_option(rs.option.gain, 32)  # Adjust as needed

                # Set manual white balance for more consistent colors
                if color_sensor.supports(rs.option.enable_auto_white_balance):
                    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
                    if color_sensor.supports(rs.option.white_balance):
                        color_sensor.set_option(rs.option.white_balance, 4600)  # Daylight temperature

                self.logger.debug("Applied camera quality settings (auto_exposure=on, gain=32, white_balance=4600)")
            except Exception as e:
                self.logger.warning("Could not set camera quality settings: %s", e)

            self.connected = True
            self.logger.info("Successfully connected to RealSense camera: %s", target_device.get_info(rs.camera_info.name))
            return True

        except Exception as e:
            self.logger.error("Failed to connect to RealSense camera: %s", e)
            return False

    def _connect_opencv(self, resolution: str = "high"):
        """Fallback connection using OpenCV"""
        try:
            self.logger.debug("Attempting OpenCV camera connect with desired resolution='%s'", resolution)
            # Try to find any available camera
            for i in range(5):  # Check first 5 camera indices
                self.logger.debug("Probing camera index %d", i)
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    # Set resolution based on setting
                    if resolution == "ultra":
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                    elif resolution == "high":
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    elif resolution == "medium":
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    else:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

                    cap.release()
                    self.connected = True
                    self.camera_index = i
                    self.camera_resolution = resolution
                    self.logger.info("Connected to camera using OpenCV (index %d, resolution: %s)", i, resolution)
                    return True

            self.logger.error("No cameras found using OpenCV")
            return False

        except Exception as e:
            self.logger.error("OpenCV camera connection failed: %s", e)
            return False

    def disconnect(self):
        """Disconnect from camera"""
        try:
            self.logger.debug("Disconnecting camera. connected=%s, pipeline=%s", self.connected, 'set' if self.pipeline else 'none')
            self.stop_camera()
            if self.pipeline:
                self.pipeline.stop()
                self.pipeline = None

            self.connected = False
            self.logger.debug("Camera disconnected successfully")
            return True
        except Exception as e:
            self.logger.error("Error disconnecting camera: %s", e)
            self.connected = False
            return False

    def start_camera(self):
        """Start camera capture"""
        if not self.connected:
            raise ConnectionError("Camera not connected")

        try:
            self.logger.debug("Starting camera capture. REALSENSE_AVAILABLE=%s, pipeline=%s", REALSENSE_AVAILABLE, 'set' if self.pipeline else 'none')
            if REALSENSE_AVAILABLE and self.pipeline:
                # RealSense is already streaming from connect()
                self.capturing = True
                self.logger.info("RealSense camera streaming started")
                return True
            else:
                # Use OpenCV fallback
                self.logger.debug("Starting OpenCV fallback capture")
                return self._start_camera_opencv()

        except Exception as e:
            self.logger.error("Failed to start camera: %s", e)
            return False

    def _start_camera_opencv(self):
        """Start camera using OpenCV fallback"""
        try:
            self.logger.debug("Opening OpenCV VideoCapture(index=%s) with resolution=%s", getattr(self, 'camera_index', 0), getattr(self, 'camera_resolution', 'high'))
            self.cap = cv2.VideoCapture(getattr(self, 'camera_index', 0))
            if self.cap.isOpened():
                # Apply resolution settings
                resolution = getattr(self, 'camera_resolution', 'high')
                if resolution == "ultra":
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                elif resolution == "high":
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                elif resolution == "medium":
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                else:
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

                # Set additional quality settings
                self.cap.set(cv2.CAP_PROP_FPS, 30)

                self.capturing = True
                actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                self.logger.info("OpenCV camera streaming started - Resolution: %dx%d", actual_width, actual_height)
                return True
            return False
        except Exception as e:
            self.logger.error("Failed to start OpenCV camera: %s", e)
            return False

    def stop_camera(self):
        """Stop camera capture"""
        try:
            self.logger.debug("Stopping camera. capturing=%s, cap=%s", self.capturing, 'set' if hasattr(self, 'cap') else 'none')
            self.capturing = False
            if hasattr(self, 'cap'):
                self.cap.release()
                delattr(self, 'cap')
            self.logger.info("Camera streaming stopped")
            return True
        except Exception as e:
            self.logger.error("Error stopping camera: %s", e)
            return False

    def capture_image(self, filename: str = None, enhance_quality: bool = False, denoise: bool = False):
        """Capture a single image with optional quality enhancements"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")

        try:
            self.logger.debug("capture_image called: filename=%s, enhance_quality=%s, denoise=%s", filename, enhance_quality, denoise)
            if REALSENSE_AVAILABLE and self.pipeline:
                # Capture multiple frames and average them for better quality
                frames_to_average = 3 if enhance_quality else 1
                self.logger.debug("Capturing via RealSense. frames_to_average=%d", frames_to_average)
                images = []

                for _ in range(frames_to_average):
                    frames = self.pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()

                    if not color_frame:
                        raise RuntimeError("Failed to capture color frame")

                    # Convert to numpy array
                    color_image = np.asanyarray(color_frame.get_data())
                    images.append(color_image)

                    if frames_to_average > 1:
                        time.sleep(0.1)  # Small delay between captures

                # Average multiple frames to reduce noise
                if len(images) > 1:
                    color_image = np.mean(images, axis=0).astype(np.uint8)
                    self.logger.debug("Averaged %d frames for noise reduction", len(images))
                else:
                    color_image = images[0]

            else:
                # Use OpenCV fallback with frame averaging
                if hasattr(self, 'cap'):
                    frames_to_average = 3 if enhance_quality else 1
                    self.logger.debug("Capturing via OpenCV. frames_to_average=%d", frames_to_average)
                    images = []

                    for _ in range(frames_to_average):
                        ret, frame = self.cap.read()
                        if ret:
                            images.append(frame)
                            if frames_to_average > 1:
                                time.sleep(0.1)

                    if not images:
                        raise RuntimeError("Failed to capture frame")

                    # Average frames if multiple captured
                    if len(images) > 1:
                        color_image = np.mean(images, axis=0).astype(np.uint8)
                    else:
                        color_image = images[0]
                else:
                    raise RuntimeError("Camera not initialized")

            # Do not apply any image enhancements
            self.logger.debug("Image captured. shape=%s", getattr(color_image, 'shape', None))

            if filename:
                cv2.imwrite(filename, color_image)
                self.logger.info("Image saved to %s", filename)

            return color_image

        except Exception as e:
            self.logger.error("Failed to capture image: %s", e)
            raise


    def capture_depth_image(self, filename: str = None):
        """Capture depth image (only available with RealSense)"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")

        if not REALSENSE_AVAILABLE or not self.pipeline:
            raise RuntimeError("Depth capture requires Intel RealSense camera. Current camera does not support depth sensing.")

        try:
            self.logger.debug("Capturing depth image. filename=%s", filename)
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                raise RuntimeError("Failed to capture depth frame")

            # Convert to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())

            # Log depth information
            valid_pixels = np.count_nonzero(depth_image)
            total_pixels = depth_image.size
            valid_percentage = (valid_pixels / total_pixels) * 100

            self.logger.info("Depth frame captured: %s, %d/%d valid pixels (%.1f%%)", 
                             depth_image.shape, valid_pixels, total_pixels, valid_percentage)

            if filename:
                # Save as 16-bit PNG to preserve depth data
                cv2.imwrite(filename, depth_image)
                self.logger.info("Depth image saved to %s", filename)

            return depth_image

        except Exception as e:
            self.logger.error("Failed to capture depth image: %s", e)
            raise

    def start_video_recording(self, filename: str, fps: int = 30, quality: str = "high",
                            codec: str = "mp4v", duration: float = None):
        """Start video recording with specified quality and FPS"""
        if not self.capturing:
            raise RuntimeError("Camera not capturing")

        try:
            self.logger.debug("Starting video recording: file=%s, fps=%s, quality=%s, codec=%s, duration=%s", filename, fps, quality, codec, duration)
            # Determine resolution based on current camera setup
            if REALSENSE_AVAILABLE and self.pipeline:
                # For RealSense, we need to get the actual stream resolution
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if color_frame:
                    width = color_frame.get_width()
                    height = color_frame.get_height()
                else:
                    width, height = 1920, 1080  # Default ultra resolution
            else:
                # For OpenCV, get from camera properties
                if hasattr(self, 'cap'):
                    width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                else:
                    width, height = 1920, 1080  # Default

            # Define codec options
            codec_options = {
                'mp4v': cv2.VideoWriter_fourcc(*'mp4v'),  # Good quality, widely supported
                'xvid': cv2.VideoWriter_fourcc(*'XVID'),  # High compression
                'h264': cv2.VideoWriter_fourcc(*'H264'),  # Modern codec, best quality
                'mjpg': cv2.VideoWriter_fourcc(*'MJPG'),  # Motion JPEG, good for high fps
            }

            fourcc = codec_options.get(codec.lower(), cv2.VideoWriter_fourcc(*'mp4v'))
            self.logger.debug("VideoWriter params resolved: size=%sx%s, fourcc=%s", width, height, codec)

            # Create VideoWriter
            self.video_writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))

            if not self.video_writer.isOpened():
                raise RuntimeError(f"Failed to open video writer for {filename}")

            self.video_filename = filename
            self.video_fps = fps
            self.video_quality = quality
            self.video_duration = duration
            self.video_start_time = time.time()
            self.is_recording = True
            self._frames_written_count = 0

            self.logger.info("Started video recording: %s (%dx%d @ %dfps, %s quality)", filename, width, height, fps, quality)
            return True

        except Exception as e:
            self.logger.error("Failed to start video recording: %s", e)
            return False

    def record_video_frame(self):
        """Record a single frame to the video file"""
        if not self.is_recording or not hasattr(self, 'video_writer'):
            return False

        try:
            # Check duration limit
            if self.video_duration and (time.time() - self.video_start_time) >= self.video_duration:
                self.stop_video_recording()
                return False

            # Capture frame
            if REALSENSE_AVAILABLE and self.pipeline:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    return False

                frame = np.asanyarray(color_frame.get_data())

            else:
                # OpenCV fallback
                if hasattr(self, 'cap'):
                    ret, frame = self.cap.read()
                    if not ret:
                        return False
                else:
                    return False

            # Write frame to video
            self.video_writer.write(frame)
            self._frames_written_count += 1
            # Throttle debug logs to every 30 frames
            if self._frames_written_count % max(1, int(self.video_fps)) == 0:
                self.logger.debug("Video recording progress: frames=%d, elapsed=%.2fs", self._frames_written_count, time.time() - self.video_start_time)
            return True

        except Exception as e:
            self.logger.error("Failed to record video frame: %s", e)
            return False

    def stop_video_recording(self):
        """Stop video recording"""
        try:
            self.logger.debug("Stopping video recording. frames_written=%s", getattr(self, '_frames_written_count', 0))
            if hasattr(self, 'video_writer') and self.video_writer:
                self.video_writer.release()
                delattr(self, 'video_writer')

            self.is_recording = False
            duration = time.time() - getattr(self, 'video_start_time', time.time())

            self.logger.info("Video recording stopped. Duration: %.1fs", duration)
            return True

        except Exception as e:
            self.logger.error("Error stopping video recording: %s", e)
            return False

    def get_camera_info(self):
        """Get camera information"""
        self.logger.debug(
            "Querying camera info. Using %s backend",
            'RealSense' if (REALSENSE_AVAILABLE and self.pipeline) else 'OpenCV',
        )
        if REALSENSE_AVAILABLE and self.pipeline:
            try:
                ctx = rs.context()
                devices = ctx.query_devices()
                if len(devices) > 0:
                    device = devices[0]
                    self.logger.debug(
                        "Camera info: name=%s, serial=%s, firmware=%s",
                        device.get_info(rs.camera_info.name),
                        device.get_info(rs.camera_info.serial_number),
                        device.get_info(rs.camera_info.firmware_version),
                    )
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

    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.controller = RealSenseCameraController()
        self.logger = logging.getLogger(__name__)
        self.connected = False
        self.logger.debug("CameraSystem created. name=%s, config_keys=%s", name, list(config.keys()))

    def initialize(self) -> bool:
        """Initialize the camera system"""
        try:
            device_id = self.config.get('device_id', 'Intel Corp')
            resolution = self.config.get('resolution', 'high')  # Default to high quality
            self.logger.debug("Initializing camera: device_id=%s, resolution=%s", device_id, resolution)
            success = self.controller.connect(device_id, resolution)
            if success:
                self.connected = True
                self.is_initialized = True
                self.logger.info("Camera system initialized successfully with %s resolution", resolution)
            else:
                self.logger.debug("Camera controller connect returned False")
            return success
        except Exception as e:
            self.logger.error("Failed to initialize camera: %s", e)
            self.connected = False
            self.is_initialized = False
            return False

    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a camera step with given configuration"""
        if not self.is_initialized:
            return {"success": False, "error": "Camera system not initialized"}

        try:
            action = step_config.get('action', 'capture_image')
            parameters = step_config.get('parameters', {})
            self.logger.debug("execute_step: action=%s, param_keys=%s", action, list(parameters.keys()) if isinstance(parameters, dict) else type(parameters))

            return self.execute_action(action, parameters)

        except Exception as e:
            self.logger.error("Camera step execution failed: %s", e)
            return {"success": False, "error": str(e)}

    def test_connection(self) -> Dict[str, Any]:
        """Test camera connection"""
        try:
            required_device_id = self.config.get('device_id', 'Intel Corp')
            self.logger.debug("Testing camera connection. required_device_id=%s, currently_connected=%s", required_device_id, self.connected)

            if not self.connected:
                # Try to initialize if not connected
                self.logger.debug("Not connected; attempting initialize() for test")
                if not self.initialize():
                    # Check if RealSense SDK is available
                    if not REALSENSE_AVAILABLE:
                        self.logger.debug("RealSense SDK not available during test_connection")
                        return {
                            "status": "not_available",
                            "message": "Intel RealSense SDK not available",
                            "details": {"device_id": required_device_id, "sdk_available": False}
                        }

                    # Check if any RealSense devices are connected
                    try:
                        ctx = rs.context()
                        devices = ctx.query_devices()
                        if len(devices) == 0:
                            self.logger.debug("No RealSense devices detected during test_connection")
                            return {
                                "status": "not_available",
                                "message": "No Intel RealSense cameras detected",
                                "details": {"device_id": required_device_id}
                            }
                        else:
                            available_devices = [dev.get_info(rs.camera_info.name) for dev in devices]
                            self.logger.debug("Available RealSense devices: %s", available_devices)
                            return {
                                "status": "not_available",
                                "message": f"Required camera not found: {required_device_id}",
                                "details": {
                                    "required_device": required_device_id,
                                    "available_devices": available_devices
                                }
                            }
                    except:
                        self.logger.debug("Exception while detecting RealSense devices during test_connection", exc_info=True)
                        return {
                            "status": "not_available",
                            "message": "Failed to detect RealSense cameras",
                            "details": {"device_id": required_device_id}
                        }

            # Try to start and immediately stop camera to test
            self.logger.debug("Attempting quick start/stop to verify streaming")
            if self.controller.start_camera():
                self.controller.stop_camera()
                camera_info = self.controller.get_camera_info()
                return {
                    "status": "available",
                    "message": f"Camera connection successful (name: {camera_info.get('name', 'Unknown')})",
                    "details": camera_info
                }
            else:
                self.logger.debug("Controller.start_camera() returned False during test_connection")
                return {
                    "status": "error",
                    "message": "Camera connected but failed to start streaming",
                    "details": {}
                }

        except Exception as e:
            self.logger.error("Camera connection test failed: %s", e)
            return {
                "status": "error",
                "message": str(e),
                "details": {}
            }

    def set_inspection_folder(self, folder_path: str):
        """Set the current inspection folder path for output"""
        self.current_inspection_folder = folder_path
        self.logger.info("Camera system inspection folder set to: %s", folder_path)
        self.logger.debug("Inspection folder absolute=%s, exists=%s", os.path.abspath(folder_path), os.path.exists(folder_path))

    def execute_action(self, action: str, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """Execute camera action"""
        if parameters is None:
            parameters = {}

        try:
            self.logger.debug("execute_action called: action=%s, params=%s", action, {k: parameters[k] for k in list(parameters)[:5]} if isinstance(parameters, dict) else type(parameters))
            if action == "capture_image":
                return self._capture_image(parameters)
            elif action == "capture_depth":
                return self._capture_depth(parameters)
            elif action == "start_stream":
                return self._start_stream(parameters)
            elif action == "stop_stream":
                return self._stop_stream(parameters)
            elif action == "record_video":
                return self._record_video(parameters)
            elif action == "start_video_recording":
                return self._start_video_recording(parameters)
            elif action == "stop_video_recording":
                return self._stop_video_recording(parameters)
            elif action == "take_video":  # Legacy support
                return self._record_video(parameters)
            elif action == "get_info":
                return self._get_camera_info()
            else:
                raise ValueError(f"Unknown camera action: {action}")

        except Exception as e:
            self.logger.error("Camera action '%s' failed: %s", action, e)
            return {"success": False, "error": str(e)}

    def _capture_image(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Capture a single image"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.logger.debug("_capture_image: starting camera since not capturing")
                self.controller.start_camera()

            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"camera_image_{timestamp}.jpg"
            self.logger.debug("_capture_image: requested filename=%s", filename)

            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            self.logger.debug("_capture_image: resolved filename=%s", filename)

            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.logger.debug("_capture_image: created output directory %s", output_dir)

            # Get enhancement settings
            enhance_quality = parameters.get('enhance_quality', False)
            denoise = parameters.get('denoise', False)
            self.logger.debug("_capture_image: enhance_quality=%s, denoise=%s", enhance_quality, denoise)

            # Capture image with optional enhancements
            image = self.controller.capture_image(filename=None, enhance_quality=enhance_quality, denoise=denoise)
            self.logger.debug("_capture_image: captured image shape=%s", getattr(image, 'shape', None))

            # Apply additional image quality settings if specified
            quality = parameters.get('quality', 95)  # JPEG quality (0-100)
            self.logger.debug("_capture_image: jpeg_quality=%s", quality)

            if filename.lower().endswith('.jpg') or filename.lower().endswith('.jpeg'):
                # Save with specified quality
                cv2.imwrite(filename, image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            elif filename.lower().endswith('.png'):
                # PNG compression level (0-9, where 9 is best compression)
                compression = parameters.get('compression', 1)  # Low compression for better quality
                self.logger.debug("_capture_image: png_compression=%s", compression)
                cv2.imwrite(filename, image, [cv2.IMWRITE_PNG_COMPRESSION, compression])
            else:
                # Default save
                cv2.imwrite(filename, image)

            # Calculate file size for reporting
            file_size = os.path.getsize(filename) if os.path.exists(filename) else 0
            self.logger.debug("_capture_image: file_size_mb=%.2f", file_size / (1024*1024) if file_size else 0)

            return {
                "success": True,
                "filename": filename,
                "image_shape": image.shape if image is not None else None,
                "quality": quality,
                "file_size_mb": round(file_size / (1024*1024), 2),
                "enhanced": enhance_quality or denoise,
                "message": f"{'Enhanced ' if enhance_quality or denoise else ''}high quality image captured successfully: {filename}"
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def _capture_depth(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Capture depth image"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.logger.debug("_capture_depth: starting camera since not capturing")
                self.controller.start_camera()

            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"depth_image_{timestamp}.png"
            self.logger.debug("_capture_depth: requested filename=%s", filename)

            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            self.logger.debug("_capture_depth: resolved filename=%s", filename)

            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.logger.debug("_capture_depth: created output directory %s", output_dir)

            # Get additional parameters
            create_colormap = parameters.get('create_colormap', True)  # Generate colorized depth image
            save_raw = parameters.get('save_raw', True)  # Save raw 16-bit depth data
            self.logger.debug("_capture_depth: create_colormap=%s, save_raw=%s", create_colormap, save_raw)

            # Capture depth image
            depth_image = self.controller.capture_depth_image(filename if save_raw else None)

            results = {
                "success": True,
                "depth_shape": depth_image.shape if depth_image is not None else None,
                "message": f"Depth image captured successfully: {filename}"
            }

            # Create colorized version for visualization
            if create_colormap and depth_image is not None:
                colormap_filename = filename.replace('.png', '_colormap.png')
                colorized_depth = self._create_depth_colormap(depth_image)
                cv2.imwrite(colormap_filename, colorized_depth)
                results["colormap_filename"] = colormap_filename
                results["message"] += f" and colormap: {colormap_filename}"
                self.logger.debug("_capture_depth: colormap saved to %s", colormap_filename)

            # Add depth statistics
            if depth_image is not None:
                # Filter out zero values (invalid depth)
                valid_depth = depth_image[depth_image > 0]
                if len(valid_depth) > 0:
                    results["depth_stats"] = {
                        "min_depth_mm": int(np.min(valid_depth)),
                        "max_depth_mm": int(np.max(valid_depth)),
                        "mean_depth_mm": int(np.mean(valid_depth)),
                        "valid_pixels": len(valid_depth),
                        "total_pixels": depth_image.size
                    }
                    self.logger.debug("_capture_depth: stats=%s", results["depth_stats"])

            # Calculate file size
            if os.path.exists(filename):
                file_size = os.path.getsize(filename)
                results["file_size_mb"] = round(file_size / (1024*1024), 2)
                self.logger.debug("_capture_depth: file_size_mb=%.2f", results["file_size_mb"])

            results["filename"] = filename
            return results

        except Exception as e:
            return {"success": False, "error": str(e)}

    def _create_depth_colormap(self, depth_image):
        """Create a colorized depth image for visualization"""
        try:
            # Normalize depth image to 0-255 range
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.logger.debug("_create_depth_colormap: depth_image shape=%s", getattr(depth_image, 'shape', None))

            # Apply colormap (COLORMAP_JET gives a nice rainbow effect)
            colorized = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # Set invalid depth pixels (0) to black
            mask = depth_image == 0
            colorized[mask] = [0, 0, 0]

            return colorized

        except Exception as e:
            self.logger.error("Failed to create depth colormap: %s", e)
            # Return a simple grayscale version as fallback
            return cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    def _start_stream(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Start camera streaming"""
        try:
            self.logger.debug("_start_stream called")
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
            self.logger.debug("_stop_stream called")
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
            self.logger.debug("_get_camera_info called")
            info = self.controller.get_camera_info()
            return {
                "success": True,
                "camera_info": info
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _record_video(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Record a video with specified duration, FPS, and quality"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.logger.debug("_record_video: starting camera since not capturing")
                self.controller.start_camera()

            # Get parameters
            duration = parameters.get('duration', parameters.get('duration', 10))  # Support both names
            fps = parameters.get('fps', 30)
            quality = parameters.get('quality', 'medium')  # low, medium, high, ultra
            codec = parameters.get('codec', 'mp4v')  # mp4v, h264, xvid, mjpg
            self.logger.debug("_record_video params: duration=%s, fps=%s, quality=%s, codec=%s", duration, fps, quality, codec)

            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"camera_video_{timestamp}.mp4"
            self.logger.debug("_record_video requested filename=%s", filename)

            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            self.logger.debug("_record_video resolved filename=%s", filename)

            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.logger.debug("_record_video created output directory %s", output_dir)

            # Start video recording
            success = self.controller.start_video_recording(filename, fps, quality, codec, duration)

            if not success:
                return {"success": False, "error": "Failed to start video recording"}

            # Record frames for the specified duration
            start_time = time.time()
            frames_recorded = 0
            target_frame_time = 1.0 / fps

            self.logger.info("Recording video for %s seconds at %s FPS...", duration, fps)

            while (time.time() - start_time) < duration:
                frame_start = time.time()

                if self.controller.record_video_frame():
                    frames_recorded += 1
                else:
                    break

                # Control frame rate
                elapsed = time.time() - frame_start
                if elapsed < target_frame_time:
                    time.sleep(target_frame_time - elapsed)

            # Stop recording
            self.controller.stop_video_recording()

            # Calculate file size
            file_size = os.path.getsize(filename) if os.path.exists(filename) else 0
            actual_duration = time.time() - start_time
            actual_fps = frames_recorded / actual_duration if actual_duration > 0 else 0
            self.logger.debug("_record_video finished: frames_recorded=%d, actual_duration=%.2fs, actual_fps=%.2f, file_size_mb=%.2f", frames_recorded, actual_duration, actual_fps, file_size / (1024*1024) if file_size else 0)

            return {
                "success": True,
                "filename": filename,
                "duration": round(actual_duration, 2),
                "fps_requested": fps,
                "fps_actual": round(actual_fps, 1),
                "frames_recorded": frames_recorded,
                "quality": quality,
                "codec": codec,
                "file_size_mb": round(file_size / (1024*1024), 2),
                "message": f"Video recorded successfully: {filename} ({actual_duration:.1f}s @ {actual_fps:.1f}fps)"
            }

        except Exception as e:
            # Ensure video recording is stopped on error
            if hasattr(self.controller, 'is_recording') and self.controller.is_recording:
                self.controller.stop_video_recording()
            return {"success": False, "error": str(e)}

    def _start_video_recording(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Start video recording (for manual control)"""
        try:
            # Start camera if not already streaming
            if not self.controller.capturing:
                self.logger.debug("_start_video_recording: starting camera since not capturing")
                self.controller.start_camera()

            # Get parameters
            fps = parameters.get('fps', 30)
            quality = parameters.get('quality', 'medium')
            codec = parameters.get('codec', 'mp4v')
            self.logger.debug("_start_video_recording params: fps=%s, quality=%s, codec=%s", fps, quality, codec)

            # Generate filename if not provided
            filename = parameters.get('filename')
            if not filename:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"camera_video_{timestamp}.mp4"
            self.logger.debug("_start_video_recording requested filename=%s", filename)

            # Use current inspection folder if available
            if self.current_inspection_folder and not os.path.isabs(filename):
                filename = os.path.join(self.current_inspection_folder, filename)
            self.logger.debug("_start_video_recording resolved filename=%s", filename)

            # Ensure output directory exists
            output_dir = os.path.dirname(filename)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.logger.debug("_start_video_recording created output directory %s", output_dir)

            # Start video recording
            success = self.controller.start_video_recording(filename, fps, quality, codec)

            return {
                "success": success,
                "filename": filename if success else None,
                "message": f"Video recording {'started' if success else 'failed'}: {filename}"
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def _stop_video_recording(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Stop video recording (for manual control)"""
        try:
            self.logger.debug("_stop_video_recording called")
            success = self.controller.stop_video_recording()
            return {
                "success": success,
                "message": "Video recording stopped" if success else "Failed to stop video recording"
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def cleanup(self):
        """Clean up camera resources"""
        try:
            self.logger.debug("CameraSystem.cleanup called")
            self.controller.stop_camera()
            self.controller.disconnect()
            self.logger.info("Camera system cleanup completed")
        except Exception as e:
            self.logger.error("Error during camera cleanup: %s", e)
