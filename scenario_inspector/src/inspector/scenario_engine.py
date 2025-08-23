"""
Scenario execution engine for running inspection programs
"""
import os
import yaml
import logging
from datetime import datetime
from typing import Dict, Any, List, Optional

from config.config_manager import ConfigManager
from systems.system_manager import SystemManager

class ScenarioEngine:
    """Main engine for executing inspection scenarios"""
    
    def __init__(self, config_manager, progress_callback=None):
        self.config_manager = config_manager
        self.progress_callback = progress_callback  # Callback for progress updates
        self.logger = logging.getLogger(__name__)
        
        # Initialize system manager
        self.system_manager = SystemManager(self.config_manager)
        
        # Current scenario state
        self.current_scenario = None
        self.current_inspection_folder = None
        self.current_inspection_date = None
        self.execution_results = []  # Track all step execution results
        self.inspection_logger = None  # Separate logger for this inspection
    
    def _emit_progress(self, message: str):
        """Safely emit progress updates if callback is available"""
        if self.progress_callback:
            self.progress_callback(message)
        else:
            self.logger.info(f"Progress: {message}")
        
        # Also log to inspection-specific log if available
        if self.inspection_logger:
            self.inspection_logger.info(f"Progress: {message}")
    
    def _setup_inspection_logger(self, inspection_folder: str, program_name: str):
        """Set up a separate logger for this inspection"""
        try:
            # Create inspection log file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = f"inspection_{program_name}_{timestamp}.log"
            log_path = os.path.join(inspection_folder, log_filename)
            
            # Create custom logger for this inspection
            inspection_logger_name = f"inspection_{timestamp}"
            self.inspection_logger = logging.getLogger(inspection_logger_name)
            self.inspection_logger.setLevel(logging.DEBUG)
            
            # Remove any existing handlers to avoid duplicates
            for handler in self.inspection_logger.handlers[:]:
                self.inspection_logger.removeHandler(handler)
            
            # Create file handler for inspection log
            file_handler = logging.FileHandler(log_path, encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)
            
            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            file_handler.setFormatter(formatter)
            
            # Add handler to logger
            self.inspection_logger.addHandler(file_handler)
            
            # Log the start of the inspection
            self.inspection_logger.info("=" * 60)
            self.inspection_logger.info(f"INSPECTION LOG - {program_name}")
            self.inspection_logger.info(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            self.inspection_logger.info("=" * 60)
            
            self.logger.info(f"Inspection log created: {log_path}")
            return log_path
            
        except Exception as e:
            self.logger.error(f"Failed to setup inspection logger: {e}")
            return None
    
    def _log_to_inspection(self, level: str, message: str):
        """Log message to inspection-specific log"""
        if self.inspection_logger:
            if level.upper() == 'DEBUG':
                self.inspection_logger.debug(message)
            elif level.upper() == 'INFO':
                self.inspection_logger.info(message)
            elif level.upper() == 'WARNING':
                self.inspection_logger.warning(message)
            elif level.upper() == 'ERROR':
                self.inspection_logger.error(message)
            else:
                self.inspection_logger.info(message)
    
    def load_scenario_from_data(self, program_data: Dict[str, Any], piece_info: Dict[str, Any]) -> bool:
        """Load scenario from program data and piece info (used by GUI)"""
        try:
            self.current_scenario = {
                'program': program_data.get('program', {}),
                'piece_info': piece_info
            }
            
            # Validate scenario structure
            if not self._validate_scenario(self.current_scenario):
                self.logger.error("Invalid scenario structure")
                return False
            
            program_name = self.current_scenario['program'].get('name', 'Unknown')
            self.logger.info(f"Loaded scenario from data: {program_name}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load scenario from data: {e}")
            return False
    
    def load_scenario_from_file(self, scenario_file_path: str) -> bool:
        """Load scenario from YAML file"""
        try:
            with open(scenario_file_path, 'r') as file:
                self.current_scenario = yaml.safe_load(file)
            
            # Validate scenario structure
            if not self._validate_scenario(self.current_scenario):
                self.logger.error("Invalid scenario structure")
                return False
            
            self.logger.info(f"Loaded scenario: {self.current_scenario['program']['name']}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load scenario from {scenario_file_path}: {e}")
            return False
    
    def _validate_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Validate scenario structure"""
        try:
            # Check required top-level keys
            if 'program' not in scenario:
                self.logger.error("Missing 'program' section in scenario")
                return False
            
            program = scenario['program']
            required_program_keys = ['name', 'stages']
            for key in required_program_keys:
                if key not in program:
                    self.logger.error(f"Missing required program key: {key}")
                    return False
            
            # Validate stages
            stages = program['stages']
            if not isinstance(stages, list) or len(stages) == 0:
                self.logger.error("Program must have at least one stage")
                return False
            
            # Validate each stage
            for stage in stages:
                if not self._validate_stage(stage):
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Scenario validation error: {e}")
            return False
    
    def _validate_stage(self, stage: Dict[str, Any]) -> bool:
        """Validate stage structure"""
        required_keys = ['stage', 'name', 'steps']
        for key in required_keys:
            if key not in stage:
                self.logger.error(f"Missing required stage key: {key}")
                return False
        
        # Validate steps
        steps = stage['steps']
        if not isinstance(steps, list) or len(steps) == 0:
            self.logger.error("Stage must have at least one step")
            return False
        
        # Validate each step
        for step in steps:
            if not self._validate_step(step):
                return False
        
        return True
    
    def _validate_step(self, step: Dict[str, Any]) -> bool:
        """Validate step structure"""
        required_keys = ['step', 'name', 'system']
        for key in required_keys:
            if key not in step:
                self.logger.error(f"Missing required step key: {key}")
                return False
        
        # Note: System existence validation is handled by SystemManager during execution
        return True
    
    def execute_scenario_from_gui(self, program_data: Dict[str, Any], piece_info: Dict[str, Any]) -> bool:
        """Execute scenario loaded from GUI data with progress tracking"""
        try:
            # Load scenario data
            if not self.load_scenario_from_data(program_data, piece_info):
                return False
            
            self._emit_progress("Starting program execution...")
            
            # Create inspection folder
            program_data_for_folder = {
                'name': program_data.get('program', {}).get('name', 'unknown_program'),
                'piece_info': piece_info
            }
            
            self.current_inspection_folder = self.system_manager.create_inspection_folder(program_data_for_folder)
            self.current_inspection_date = datetime.now()
            self._emit_progress(f"Created inspection folder: {os.path.basename(self.current_inspection_folder)}")
            
            # Set up inspection-specific logger
            program_name = program_data.get('program', {}).get('name', 'unknown_program')
            log_path = self._setup_inspection_logger(self.current_inspection_folder, program_name)
            if log_path:
                self._emit_progress(f"Inspection log created: {os.path.basename(log_path)}")
            
            # Log initial inspection info
            self._log_to_inspection('INFO', f"Program: {program_name}")
            self._log_to_inspection('INFO', f"Piece: {piece_info.get('name_piece', 'Unknown')}")
            self._log_to_inspection('INFO', f"Reference: {piece_info.get('ref_piece', 'Unknown')}")
            self._log_to_inspection('INFO', f"Inspection folder: {self.current_inspection_folder}")
            
            # Get program stages
            stages = program_data.get('program', {}).get('stages', [])
            total_steps = sum(len(stage.get('steps', [])) for stage in stages)
            completed_steps = 0
            
            # Execute all stages
            for stage in stages:
                stage_name = stage.get('name', f"Stage {stage.get('stage', 'Unknown')}")
                together = stage.get('together', False)
                pause_after = stage.get('pause', False)

                self._emit_progress(f"Executing stage: {stage_name} {'(parallel)' if together else '(sequential)'}")

                # Execute steps in stage
                steps = stage.get('steps', [])

                if together and len(steps) > 1:
                    # Execute steps in parallel
                    completed_steps += self.execute_steps_parallel(steps, stage_name, completed_steps, total_steps)
                else:
                    # Execute steps sequentially 
                    completed_steps += self.execute_steps_sequential(steps, completed_steps, total_steps)

                # Calculate progress
                progress_percent = int((completed_steps / total_steps) * 100) if total_steps > 0 else 100
                self._emit_progress(f"Stage '{stage_name}' completed. Progress: {progress_percent}% ({completed_steps}/{total_steps})")
                # Emit progress percentage for progress bar update
                if hasattr(self, 'progress_callback_percentage') and callable(self.progress_callback_percentage):
                    self.progress_callback_percentage(progress_percent)
                self._log_to_inspection('INFO', f"Stage '{stage_name}' completed. {len(steps)} steps executed.")

                # PAUSE: If pause is set, yield control and wait for GUI to resume
                if pause_after:
                    # Emit progress bar update before pausing
                    if hasattr(self, 'progress_callback_percentage') and callable(self.progress_callback_percentage):
                        self.progress_callback_percentage(progress_percent)
                    elif hasattr(self.progress_callback, 'emit'):
                        try:
                            self.progress_callback.emit(progress_percent)
                        except Exception:
                            pass
                    if hasattr(self, 'pause_callback') and callable(self.pause_callback):
                        self._emit_progress(f"Paused after stage '{stage_name}'. Waiting for user to continue...")
                        self.pause_callback(stage_name)
                    else:
                        self._emit_progress(f"Paused after stage '{stage_name}', but no pause_callback set. Skipping pause.")
            
            # Create inspection report
            self._emit_progress("Creating inspection report...")
            self._log_to_inspection('INFO', "Creating inspection report and finalizing inspection...")
            self.create_inspection_report()
            
            # Log inspection completion
            total_successful = sum(1 for result in self.execution_results if result.get('success', False))
            total_failed = len(self.execution_results) - total_successful
            success_rate = (total_successful / len(self.execution_results) * 100) if self.execution_results else 0
            
            self._log_to_inspection('INFO', "=" * 50)
            self._log_to_inspection('INFO', "INSPECTION COMPLETED")
            self._log_to_inspection('INFO', f"Total steps: {len(self.execution_results)}")
            self._log_to_inspection('INFO', f"Successful: {total_successful}")
            self._log_to_inspection('INFO', f"Failed: {total_failed}")
            self._log_to_inspection('INFO', f"Success rate: {success_rate:.1f}%")
            self._log_to_inspection('INFO', f"Completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            self._log_to_inspection('INFO', "=" * 50)
            
            self._emit_progress("Program execution completed successfully!")
            return True
            
        except Exception as e:
            self.logger.error(f"Scenario execution error: {e}")
            self._log_to_inspection('ERROR', f"Inspection execution failed: {str(e)}")
            self._emit_progress(f"Execution failed: {str(e)}")
            return False
        finally:
            # Cleanup inspection logger
            if self.inspection_logger:
                for handler in self.inspection_logger.handlers[:]:
                    handler.close()
                    self.inspection_logger.removeHandler(handler)
                self.inspection_logger = None
            
            # Cleanup systems
            self.system_manager.shutdown_all_systems()
    
    def execute_steps_sequential(self, steps, current_completed, total_steps):
        """Execute steps one after another"""
        completed = 0
        for step in steps:
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            
            self._emit_progress(f"Executing step: {step_name} on system: {system}")
            self._log_to_inspection('INFO', f"Starting step: {step_name} on system: {system}")
            
            start_time = datetime.now()
            try:
                result = self.system_manager.execute_step(step)
                end_time = datetime.now()
                duration = (end_time - start_time).total_seconds()
                
                # Log successful execution
                self._log_to_inspection('INFO', f"Step '{step_name}' completed successfully in {duration:.2f} seconds")
                
                # Store execution result
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': True,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'result': result
                }
                self.execution_results.append(step_result)
                
                completed += 1
                
            except Exception as e:
                end_time = datetime.now()
                duration = (end_time - start_time).total_seconds()
                
                # Log failed execution
                self._log_to_inspection('ERROR', f"Step '{step_name}' failed after {duration:.2f} seconds: {str(e)}")
                
                # Store failed execution result
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': False,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'error': str(e)
                }
                self.execution_results.append(step_result)
                
                self._emit_progress(f"Step {step_name} failed: {str(e)}")
                raise
                
        return completed
    
    def execute_steps_parallel(self, steps, stage_name, current_completed, total_steps):
        """Execute steps in parallel using threading"""
        from concurrent.futures import ThreadPoolExecutor, as_completed
        
        completed = 0
        
        def execute_single_step(step):
            """Execute a single step - to be run in thread"""
            step_name = step.get('name', f"Step {step.get('step', 'Unknown')}")
            system = step.get('system')
            start_time = datetime.now()
            
            try:
                self._emit_progress(f"[Parallel] Starting step: {step_name} on system: {system}")
                result = self.system_manager.execute_step(step)
                end_time = datetime.now()
                
                self._emit_progress(f"[Parallel] Completed step: {step_name}")
                
                # Return step result data
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': True,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'result': result
                }
                return step_result
                
            except Exception as e:
                end_time = datetime.now()
                error_msg = f"Step {step_name} failed: {str(e)}"
                self._emit_progress(f"[Parallel] {error_msg}")
                
                # Return failed step result data
                step_result = {
                    'name': step_name,
                    'system': system,
                    'success': False,
                    'start_time': start_time,
                    'end_time': end_time,
                    'duration': (end_time - start_time).total_seconds(),
                    'error': str(e)
                }
                return step_result
        
        # Execute steps in parallel using ThreadPoolExecutor
        self._emit_progress(f"Starting {len(steps)} parallel steps in stage: {stage_name}")
        
        with ThreadPoolExecutor(max_workers=len(steps)) as executor:
            # Submit all steps for execution
            future_to_step = {executor.submit(execute_single_step, step): step for step in steps}
            
            # Wait for all steps to complete
            for future in as_completed(future_to_step):
                step = future_to_step[future]
                try:
                    step_result = future.result()
                    
                    # Store the execution result
                    self.execution_results.append(step_result)
                    
                    if step_result['success']:
                        completed += 1
                    else:
                        # If any step fails, we should stop execution
                        self._emit_progress(f"Parallel execution failed: {step_result.get('error', 'Unknown error')}")
                        raise Exception(step_result.get('error', 'Unknown error'))
                        
                except Exception as e:
                    step_name = step.get('name', 'Unknown')
                    self._emit_progress(f"Exception in parallel step {step_name}: {str(e)}")
                    raise
        
        self._emit_progress(f"All {len(steps)} parallel steps completed successfully")
        return completed
    
    def create_inspection_report(self):
        """Create inspection report using the file manager"""
        if not self.current_inspection_folder or not self.execution_results:
            self.logger.warning("Cannot create inspection report: missing inspection folder or execution results")
            self.logger.debug(f"Inspection folder: {self.current_inspection_folder}")
            self.logger.debug(f"Execution results count: {len(self.execution_results) if self.execution_results else 0}")
            return
        
        try:
            # Prepare report data
            program = self.current_scenario.get('program', {})
            piece_info = self.current_scenario.get('piece_info', {})
            program_name = program.get('name', 'Unknown')
            piece_name = piece_info.get('name_piece', 'Unknown')
            ref_piece = piece_info.get('ref_piece', 'Unknown')
            
            # Use guest user info by default
            inspector_name = "Guest User"
            
            self.logger.debug(f"Creating report for {len(self.execution_results)} steps")
            
            report_data = {
                'program_name': program_name,
                'piece_name': piece_name,
                'ref_piece': ref_piece,
                'inspection_date': self.current_inspection_date.strftime('%Y-%m-%d %H:%M:%S') if self.current_inspection_date else 'Unknown',
                'inspector': inspector_name,
                'steps': self.execution_results
            }
            
            # Create the report using the file manager
            report_path = self.system_manager.file_manager.create_inspection_report(
                self.current_inspection_folder, report_data
            )
            
            if report_path:
                self._emit_progress(f"Inspection report created: {os.path.basename(report_path)}")
                self.logger.info(f"Inspection report created at: {report_path}")
            else:
                self.logger.warning("Failed to create inspection report")
                
        except Exception as e:
            self.logger.error(f"Error creating inspection report: {e}")
            self._emit_progress(f"Warning: Failed to create inspection report: {str(e)}")

