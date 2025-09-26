"""
Scan Control System for laser scanning operations
"""

import logging
import time
import os
from typing import Dict, Any
from systems.base_system import BaseSystem
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../python_bindings')))

# Robust import for pyllt (dev and PyInstaller-frozen)
def _import_pyllt():
    log = logging.getLogger(__name__)
    try:
        log.debug("Trying direct import of pyllt")
        import pyllt
        log.debug("Direct import of pyllt succeeded")
        return pyllt
    except ImportError:
        # Try to find libs/python_bindings up to 3 levels up from exe or script
        if hasattr(sys, '_MEIPASS'):
            base_dir = os.path.dirname(sys.executable)
        else:
            base_dir = os.path.dirname(getattr(sys.modules['__main__'], '__file__', os.path.abspath(__file__)))
        log.debug("Searching for python_bindings starting from base_dir=%s", base_dir)
        found = False
        for _ in range(4):  # Check current + 3 levels up
            candidate = os.path.join(base_dir, 'libs', 'python_bindings')
            log.debug("Checking candidate path: %s", candidate)
            if os.path.isdir(candidate):
                if candidate not in sys.path:
                    sys.path.insert(0, candidate)
                log.info("Found python_bindings at %s; added to sys.path", candidate)
                found = True
                break
            parent = os.path.dirname(base_dir)
            if parent == base_dir:
                break
            base_dir = parent
        if not found:
            log.error('Cannot find libs/python_bindings directory in any of 3 parent folders')
            raise ImportError('Cannot find libs/python_bindings directory in any of 3 parent folders')
        try:
            log.debug("Attempting import of pyllt after updating sys.path")
            import pyllt
            log.debug("Import of pyllt after sys.path update succeeded")
            return pyllt
        except ImportError:
            log.error('Cannot find pyllt after adding libs/python_bindings to sys.path')
            raise ImportError('Cannot find pyllt after adding libs/python_bindings to sys.path')
llt = _import_pyllt()
import ctypes as ct

class ScanControlSystem(BaseSystem):
    def set_inspection_folder(self, folder_path: str):
        """Set the current inspection folder path for output"""
        self.current_inspection_folder = folder_path
        self.logger.info("ScanControl system inspection folder set to: %s", folder_path)
        self.logger.debug("Inspection folder absolute=%s, exists=%s", os.path.abspath(folder_path), os.path.exists(folder_path))
    """Scan Control System for laser scanning operations"""
    def __init__(self, name: str, config: Dict[str, Any]):
        self.logger = logging.getLogger(__name__)
        self.logger.debug("[DEBUG] ScanControlSystem.__init__() called with name='%s', config=%s", name, config)
        super().__init__(name, config)
        self.hLLT = None
        self.resolution = None
        self.scanner_type = None
        self.connected = False
        # Debug initial state
        self.logger.debug("[DEBUG] ScanControlSystem created. name=%s, config_keys=%s", name, list(config.keys()))
        self.logger.debug(
            "ScanControlSystem created. name=%s, config_keys=%s", name, list(config.keys())
        )

    def test_connection(self) -> Dict[str, Any]:
        self.logger.debug("[DEBUG] ScanControlSystem.test_connection() called")
        try:
            self.logger.info("Testing connection to ScanControl system...")
            self.logger.debug("[DEBUG] Testing connection to ScanControl system...")
            self.logger.debug("test_connection: starting initialize() for probe")
            self.logger.debug("[DEBUG] test_connection: starting initialize() for probe")
            # Try to initialize and connect, then disconnect
            if not self.initialize():
                self.logger.debug("[DEBUG] test_connection: initialize() returned False")
                self.logger.debug("test_connection: initialize() returned False")
                return {'status': 'error', 'message': 'Initialization failed'}
            self.logger.debug("[DEBUG] test_connection: initialize() succeeded; proceeding to cleanup()")
            self.logger.debug("test_connection: initialize() succeeded; proceeding to cleanup()")
            self.cleanup()
            self.logger.debug("[DEBUG] Connection test passed successfully")
            # Use 'available' to align with UI green check expectations
            return {
                'status': 'available',
                'message': 'ScanControl system connection test passed.',
                'details': {'ip': self.config.get('ip', 'unknown')}
            }
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in test_connection(): %s", e)
            self.logger.error("ScanControl connection test failed: %s", e)
            return {'status': 'error', 'message': str(e)}

    def initialize(self) -> bool:
        self.logger.debug("[DEBUG] ScanControlSystem.initialize() called")
        # Always clean up any previous connection before initializing
        self.cleanup()
        try:
            ip_address = self.config.get('ip', 'unknown')
            self.logger.debug("[DEBUG] Initializing ScanControl at IP: %s", ip_address)
            self.logger.info("Initializing ScanControl at %s", ip_address)
            self.logger.debug("[DEBUG] initialize: creating LLT device handle (Ethernet)")
            self.logger.debug("initialize: creating LLT device handle (Ethernet)")
            # 1. Create device handle for Ethernet
            self.hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
            self.logger.debug("[DEBUG] LLT device handle created: %s", self.hLLT)
            # 2. Find available interfaces
            available_interfaces = (ct.c_uint * 6)()
            self.logger.debug("[DEBUG] Getting device interfaces...")
            res = llt.get_device_interfaces_fast(self.hLLT, available_interfaces, len(available_interfaces))
            try:
                iface_list = [available_interfaces[i] for i in range(len(available_interfaces))]
            except Exception:
                iface_list = []
            self.logger.debug("[DEBUG] initialize: interfaces status=%s, candidates=%s", res, iface_list)
            self.logger.debug("initialize: interfaces status=%s, candidates=%s", res, iface_list)
            # 3. Set first found interface
            self.logger.debug("[DEBUG] Setting device interface to: %s", available_interfaces[0])
            # Pass integer for third argument, not string IP
            llt.set_device_interface(self.hLLT, available_interfaces[0], 0)
            self.logger.debug("[DEBUG] initialize: set_device_interface to %s", available_interfaces[0])
            self.logger.debug("initialize: set_device_interface to %s", available_interfaces[0])
            # 4. Connect
            self.logger.debug("[DEBUG] Attempting to connect to device...")
            conn_code = llt.connect(self.hLLT)
            self.logger.debug("[DEBUG] initialize: connect() returned code=%s", conn_code)
            self.logger.debug("initialize: connect() returned code=%s", conn_code)
            if conn_code != 1:
                self.logger.debug("[DEBUG] Failed to connect to ScanControl device, code: %s", conn_code)
                self.logger.error("Failed to connect to ScanControl device")
                self.is_initialized = False
                return False
            self.logger.debug("[DEBUG] Successfully connected to ScanControl device")
            # 5. Get scanner type
            self.logger.debug("[DEBUG] Getting scanner type...")
            scanner_type = ct.c_int(0)
            llt.get_llt_type(self.hLLT, ct.byref(scanner_type))
            self.scanner_type = scanner_type.value
            self.logger.debug("[DEBUG] initialize: scanner_type=%s", self.scanner_type)
            self.logger.debug("initialize: scanner_type=%s", self.scanner_type)
            # 6. Get and set resolution (use highest by default)
            self.logger.debug("[DEBUG] Getting available resolutions...")
            available_resolutions = (ct.c_uint * 4)()
            llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
            self.resolution = available_resolutions[0]
            self.logger.debug("[DEBUG] Setting resolution to: %s", self.resolution)
            llt.set_resolution(self.hLLT, self.resolution)
            try:
                res_list = [available_resolutions[i] for i in range(len(available_resolutions))]
            except Exception:
                res_list = []
            self.logger.debug("[DEBUG] initialize: available_resolutions=%s, selected=%s", res_list, self.resolution)
            self.logger.debug("initialize: available_resolutions=%s, selected=%s", res_list, self.resolution)
            # 7. Set profile config to PROFILE
            self.logger.debug("[DEBUG] Setting profile config to PROFILE...")
            llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
            self.logger.debug("[DEBUG] initialize: set_profile_config to PROFILE")
            self.logger.debug("initialize: set_profile_config to PROFILE")
            self.is_initialized = True
            self.logger.debug("[DEBUG] ScanControl system initialized successfully")
            self.logger.info("ScanControl system initialized successfully")
            return True
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in initialize(): %s", e)
            self.logger.error("Failed to initialize ScanControl: %s", e)
            self.is_initialized = False
            return False

    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_initialized:
            self.logger.debug("[DEBUG] execute_step called but system not initialized")
            raise RuntimeError("ScanControl system not initialized")
        step_name = step_config.get('name', 'unknown')
        action = step_config.get('action', 'recording_data')
        parameters = step_config.get('parameters', {})
        recording_time = step_config.get('recording_time', 10)
        saving_file = step_config.get('saving_file', False)
        output_path = step_config.get('path', 'scan_data')
        # Use inspection folder if output_path is not absolute
        if hasattr(self, 'current_inspection_folder') and self.current_inspection_folder:
            if not os.path.isabs(output_path):
                output_path = os.path.join(self.current_inspection_folder, output_path)
        self.logger.debug("[DEBUG] ScanControlSystem.execute_step() called with step_name='%s'", step_name)
        self.logger.debug("[DEBUG] Step parameters: action=%s, recording_time=%s, saving_file=%s, output_path=%s",
              action, recording_time, saving_file, output_path)
        self.logger.info("Executing ScanControl step: %s", step_name)
        self.logger.debug(
            "execute_step: action=%s, recording_time=%s, saving_file=%s, output_path=%s, param_keys=%s",
            action,
            recording_time,
            saving_file,
            output_path,
            list(parameters.keys()) if isinstance(parameters, dict) else type(parameters),
        )
        try:
            if action == "recording_data":
                self.logger.debug("[DEBUG] Executing recording_data action")
                result = self._execute_data_recording(parameters, recording_time, saving_file, output_path)
            else:
                self.logger.debug("[DEBUG] Unsupported scan action %s", action)
                raise ValueError(f"Unsupported scan action: {action}")
        except Exception as e:
            self.logger.debug("[DEBUG] Exception in execute_step(): %s", e)
            self.logger.error("ScanControl step execution failed: %s", e)
            result = {'success': False, 'error': str(e), 'step_name': step_name}
        finally:
            self.cleanup()
        return result

    def _execute_data_recording(self, parameters: Dict[str, Any], recording_time: int, saving_file: bool, output_path: str) -> Dict[str, Any]:
        self.logger.debug("[DEBUG] ScanControlSystem._execute_data_recording() called")
        self.logger.debug(
            "[DEBUG] Recording parameters: recording_time=%s, saving_file=%s, output_path=%s",
            recording_time,
            saving_file,
            output_path,
        )
        self.logger.info("Starting data recording scan (real scanner)")
        self.logger.debug(
            "_execute_data_recording: recording_time=%s, saving_file=%s, output_path=%s",
            recording_time,
            saving_file,
            output_path,
        )

        # Parameters to control 3D stacking
        profile_spacing_mm = float(parameters.get('profile_spacing_mm', 1.0))  # distance per profile when using index mode
        gantry_speed_mm_s = float(parameters.get('gantry_speed_mm_s', 0.0))    # distance per second when using time mode
        y_origin_mm = float(parameters.get('y_origin_mm', 0.0))                # optional offset
        y_mode = str(parameters.get('y_mode', 'auto')).lower()                 # 'auto' | 'time' | 'index'
        intensity_min = int(parameters.get('intensity_min', 1))                # filter low intensity
        drop_z_leq_zero = bool(parameters.get('drop_z_leq_zero', True))

        start_time = time.time()
        self.logger.debug("[DEBUG] Starting profile transfer...")
        llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)

        profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        lost_profiles = ct.c_int()

        profiles = []  # raw 2D for legacy CSV
        pts_x, pts_y, pts_z, pts_i = [], [], [], []  # stacked 3D

        self.logger.debug("[DEBUG] Beginning profile capture loop for ~%ss (continuous)", recording_time)
        end_time = start_time + float(recording_time)
        profile_index = 0
        last_log_time = start_time
        while time.time() < end_time:
            # Fetch latest profile from device buffer
            llt.get_actual_profile(
                self.hLLT,
                profile_buffer,
                len(profile_buffer),
                llt.TProfileConfig.PROFILE,
                ct.byref(lost_profiles),
            )

            # Convert to x, z, intensities
            x = (ct.c_double * self.resolution)()
            z = (ct.c_double * self.resolution)()
            intensities = (ct.c_ushort * self.resolution)()
            snull = ct.POINTER(ct.c_ushort)()
            inull = ct.POINTER(ct.c_uint)()
            llt.convert_profile_2_values(
                self.hLLT,
                profile_buffer,
                self.resolution,
                llt.TProfileConfig.PROFILE,
                self.scanner_type,
                0,
                1,
                snull,
                intensities,
                snull,
                x,
                z,
                inull,
                inull,
            )

            # Store raw profile for 2D CSV
            x_row = [x[j] for j in range(self.resolution)]
            z_row = [z[j] for j in range(self.resolution)]
            i_row = [int(intensities[j]) for j in range(self.resolution)]
            profiles.append({'x': x_row, 'z': z_row, 'intensities': i_row})

            # Map to Y
            now = time.time()
            elapsed = now - start_time
            use_time_mode = (y_mode == 'time') or (y_mode == 'auto' and gantry_speed_mm_s > 0)
            if use_time_mode:
                y_val = y_origin_mm + float(elapsed) * gantry_speed_mm_s
            else:
                y_val = y_origin_mm + float(profile_index) * profile_spacing_mm

            # Accumulate 3D points with filters
            for j in range(self.resolution):
                xi = float(x[j])
                zi = float(z[j])
                ii = int(intensities[j])
                if drop_z_leq_zero and zi <= 0:
                    continue
                if ii < intensity_min:
                    continue
                pts_x.append(xi)
                pts_y.append(y_val)
                pts_z.append(zi)
                pts_i.append(ii)

            profile_index += 1

            # Telemetry (approx capture rate)
            if now - last_log_time >= 1.0:
                rate = profile_index / max(1e-6, (now - start_time))
                self.logger.debug(
                    "[DEBUG] Capture rate ~%.1f Hz, profiles=%s, points=%s",
                    rate,
                    profile_index,
                    len(pts_x),
                )
                last_log_time = now

            # Very short sleep to avoid tight busy-loop
            time.sleep(0.001)

        # Stop transfer
        self.logger.debug(
            "[DEBUG] Stopping profile transfer; captured %s profiles, lost_profiles flag=%s",
            profile_index,
            lost_profiles.value,
        )
        llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        actual_duration = time.time() - start_time
        est_rate_hz = profile_index / max(1e-6, actual_duration)
        y_mode_used = 'time' if use_time_mode else 'index'
        self.logger.debug(
            "[DEBUG] Data recording finished: profiles=%s, points=%s, duration=%.2fs, est_rate=%.1f Hz, y_mode=%s",
            len(profiles),
            len(pts_x),
            actual_duration,
            est_rate_hz,
            y_mode_used,
        )

        result = {
            'success': True,
            'step_name': 'laser_scan',
            'mode': 'recording_data',
            'duration': actual_duration,
            'parameters_used': parameters,
            'profiles_captured': len(profiles),
            'points_captured': len(pts_x),
            'estimated_profile_rate_hz': est_rate_hz,
            'y_mode_used': y_mode_used,
            'file_saved': False,
            'output_path': None,
        }

        # Save outputs
        self.logger.debug("[DEBUG] Saving scan data to file...")
        try:
            self.logger.debug("[DEBUG] Creating output directory: %s", output_path)
            os.makedirs(output_path, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            # 2D CSV (legacy)
            file_path = os.path.join(output_path, f"scan_data_{timestamp}.csv")
            self.logger.debug("[DEBUG] Writing 2D CSV: %s", file_path)
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write("# x,z,intensity\n")
                for profile in profiles:
                    for j in range(self.resolution):
                        f.write("%s,%s,%s\n" % (profile['x'][j], profile['z'][j], profile['intensities'][j]))
            result['file_saved'] = True
            result['output_path'] = file_path
            try:
                size_mb = os.path.getsize(file_path) / (1024 * 1024)
            except Exception:
                size_mb = 0
            self.logger.debug("[DEBUG] Scan data saved successfully, file size: %.2f MB", size_mb)
            self.logger.info("Scan data saved to: %s", file_path)
            self.logger.debug("Saved file size: %.2f MB", size_mb)

            # 3D CSV
            file_path_3d = os.path.join(output_path, f"scan_points_{timestamp}.csv")
            self.logger.debug("[DEBUG] Writing 3D CSV: %s (points=%s)", file_path_3d, len(pts_x))
            with open(file_path_3d, 'w', encoding='utf-8') as f3:
                f3.write("x,y,z,intensity\n")
                for xi, yi, zi, ii in zip(pts_x, pts_y, pts_z, pts_i):
                    f3.write(f"{xi},{yi},{zi},{ii}\n")
            result['output_path_3d_csv'] = file_path_3d

            # PLY (ASCII) with grayscale mapped from intensity
            file_path_ply = os.path.join(output_path, f"scan_points_{timestamp}.ply")
            self._write_ply_ascii(file_path_ply, pts_x, pts_y, pts_z, pts_i)
            result['output_path_ply'] = file_path_ply
        except Exception as e:
            self.logger.debug("[DEBUG] Failed to save scan data: %s", e)
            self.logger.error("Failed to save scan data: %s", e)
            result['save_error'] = str(e)
        else:
            self.logger.debug("[DEBUG] Saving file not requested, skipping...")

        self.logger.debug("[DEBUG] Data recording scan completed successfully")
        self.logger.info("Data recording scan completed successfully")
        return result

    def _write_ply_ascii(self, path: str, xs, ys, zs, intensities) -> None:
        try:
            n = len(xs)
            if n == 0:
                self.logger.warning("No points to write to PLY: %s", path)
                return
            imin = min(intensities) if intensities else 0
            imax = max(intensities) if intensities else 1
            span = float(imax - imin) if imax > imin else 1.0
            with open(path, 'w', encoding='ascii') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {n}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
                f.write("end_header\n")
                for xi, yi, zi, ii in zip(xs, ys, zs, intensities):
                    norm = (float(ii) - imin) / span
                    c = int(max(0, min(255, round(norm * 255.0))))
                    f.write(f"{xi:.6f} {yi:.6f} {zi:.6f} {c} {c} {c}\n")
            self.logger.info("PLY written: %s (points=%s)", path, n)
        except Exception as e:
            self.logger.error("Failed to write PLY: %s", e)

    def _execute_calibration(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute calibration scan"""
        self.logger.debug("[DEBUG] ScanControlSystem._execute_calibration() called")
        self.logger.info("Starting calibration scan")
        try:
            self.logger.debug("[DEBUG] _execute_calibration: param_keys=%s",
                  list(parameters.keys()) if isinstance(parameters, dict) else type(parameters))
            self.logger.debug(
                "_execute_calibration: param_keys=%s",
                list(parameters.keys()) if isinstance(parameters, dict) else type(parameters),
            )
        except Exception:
            pass

        # Simulate calibration process
        self.logger.debug("[DEBUG] Simulating calibration process for 5 seconds...")
        time.sleep(5)

        self.logger.debug("[DEBUG] Calibration scan completed successfully")
        return {
            'success': True,
            'step_name': 'laser_calibration',
            'mode': 'calibration',
            'calibration_status': 'completed',
            'accuracy': 99.5  # Simulate calibration accuracy
        }

    def _execute_test_scan(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Execute test scan"""
        self.logger.debug("[DEBUG] ScanControlSystem._execute_test_scan() called")
        self.logger.info("Starting test scan")
        try:
            self.logger.debug("[DEBUG] _execute_test_scan: param_keys=%s",
                  list(parameters.keys()) if isinstance(parameters, dict) else type(parameters))
            self.logger.debug(
                "_execute_test_scan: param_keys=%s",
                list(parameters.keys()) if isinstance(parameters, dict) else type(parameters),
            )
        except Exception:
            pass

        # Simulate test scan
        self.logger.debug("[DEBUG] Simulating test scan for 2 seconds...")
        time.sleep(2)

        self.logger.debug("[DEBUG] Test scan completed successfully")
        return {
            'success': True,
            'step_name': 'laser_test',
            'mode': 'test_scan',
            'test_results': 'passed',
            'signal_quality': 'good'
        }

    def validate_scan_parameters(self, parameters: Dict[str, Any]) -> bool:
        """Validate scan parameters"""
        # For now, always return True (use defaults)
        self.logger.debug("[DEBUG] ScanControlSystem.validate_scan_parameters() called")
        self.logger.debug("[DEBUG] validate_scan_parameters called with keys=%s",
              list(parameters.keys()) if isinstance(parameters, dict) else type(parameters))
        self.logger.debug(
            "validate_scan_parameters called with keys=%s",
            list(parameters.keys()) if isinstance(parameters, dict) else type(parameters),
        )
        return True

    def cleanup(self) -> None:
        self.logger.debug("[DEBUG] ScanControlSystem.cleanup() called")
        self.logger.info("Cleaning up ScanControl system")
        self.logger.debug("[DEBUG] cleanup: hLLT=%s, is_initialized=%s",
              'set' if self.hLLT is not None else 'none', self.is_initialized)
        self.logger.debug(
            "cleanup: hLLT=%s, is_initialized=%s",
            'set' if self.hLLT is not None else 'none',
            self.is_initialized,
        )
        try:
            if self.hLLT is not None:
                self.logger.debug("[DEBUG] Disconnecting and deleting LLT device...")
                llt.disconnect(self.hLLT)
                llt.del_device(self.hLLT)
                self.hLLT = None
                self.logger.debug("[DEBUG] LLT device cleanup completed")
        except Exception as e:
            self.logger.debug("[DEBUG] Error during cleanup: %s", e)
            self.logger.error("Error during cleanup: %s", e)
        self.is_initialized = False
        self.logger.debug("[DEBUG] ScanControl system cleanup completed")
        self.logger.info("ScanControl system cleanup completed")
