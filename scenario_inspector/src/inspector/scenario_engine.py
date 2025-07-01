"""
Scenario execution engine for running inspection programs
"""
import yaml
import logging
from datetime import datetime
from typing import Dict, Any, List, Optional
from dataclasses import asdict

from config.config_manager import ConfigManager
from database.connection import DatabaseConnection
from database.models import Inspection, User
from systems.base_system import BaseSystem
from systems.scan_control import ScanControlSystem
from systems.camera_system import CameraSystem
from utils.file_manager import FileManager

class ScenarioEngine:
    """Main engine for executing inspection scenarios"""
    
    def __init__(self, user: User):
        self.user = user
        self.logger = logging.getLogger(__name__)
        self.config_manager = ConfigManager()
        self.db_connection = DatabaseConnection()
        
        # Initialize file manager
        output_config = self.config_manager.get_output_config()
        self.file_manager = FileManager(output_config['base_directory'])
        
        # Initialize systems
        self.systems = {}
        self._initialize_systems()
        
        # Current scenario state
        self.current_scenario = None
        self.current_inspection_id = None
        self.current_inspection_folder = None
    
    def _initialize_systems(self):
        """Initialize all available systems"""
        systems_config = self.config_manager.get_systems_config()
        
        for system_name, system_config in systems_config.items():
            try:
                if system_name == "scanControl":
                    self.systems[system_name] = ScanControlSystem(system_name, system_config)
                elif system_name == "camera":
                    self.systems[system_name] = CameraSystem(system_name, system_config)
                else:
                    self.logger.warning(f"Unknown system type: {system_name}")
                    
            except Exception as e:
                self.logger.error(f"Failed to initialize system {system_name}: {e}")
    
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
        
        # Check if system exists
        system_name = step['system']
        if system_name not in self.systems:
            self.logger.error(f"Unknown system: {system_name}")
            return False
        
        return True
    
    def execute_scenario(self) -> bool:
        """Execute the loaded scenario"""
        if not self.current_scenario:
            self.logger.error("No scenario loaded")
            return False
        
        program = self.current_scenario['program']
        program_name = program['name']
        
        # Get piece information
        piece_info = program.get('piece_info', {})
        piece_name = piece_info.get('name_piece', 'Unknown')
        ref_piece = piece_info.get('ref_piece', 'Unknown')
        
        self.logger.info(f"Starting execution of scenario: {program_name}")
        
        try:
            # Create inspection folder
            self.current_inspection_folder = self.file_manager.create_inspection_folder(
                program_name, piece_name, ref_piece
            )
            
            # Create inspection record in database
            inspection = Inspection(
                name_piece=piece_name,
                ref_piece=ref_piece,
                name_program=program_name,
                state='IN_PROGRESS',
                inspection_date=datetime.now().date(),
                inspection_path=self.current_inspection_folder,
                inspection_status='RUNNING',
                creation_date=datetime.now(),
                user_creation=self.user.id_user
            )
            
            self.current_inspection_id = self.db_connection.create_inspection(inspection)
            if not self.current_inspection_id:
                self.logger.error("Failed to create inspection record")
                return False
            
            # Execute all stages
            execution_results = []
            overall_success = True
            
            for stage in program['stages']:
                stage_result = self._execute_stage(stage)
                execution_results.append(stage_result)
                if not stage_result['success']:
                    overall_success = False
            
            # Update inspection status
            final_status = 'COMPLETED' if overall_success else 'FAILED'
            inspection.id_inspection = self.current_inspection_id
            inspection.inspection_status = final_status
            inspection.state = 'GOOD' if overall_success else 'NOT_GOOD'
            inspection.modification_date = datetime.now()
            inspection.user_modification = self.user.id_user
            
            self.db_connection.update_inspection(inspection)
            
            # Create inspection report
            report_data = {
                'program_name': program_name,
                'piece_name': piece_name,
                'ref_piece': ref_piece,
                'inspection_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'inspector': f"{self.user.first_name} {self.user.last_name}",
                'steps': execution_results
            }
            
            self.file_manager.create_inspection_report(
                self.current_inspection_folder, report_data
            )
            
            self.logger.info(f"Scenario execution completed. Status: {final_status}")
            return overall_success
            
        except Exception as e:
            self.logger.error(f"Scenario execution failed: {e}")
            
            # Update inspection as failed
            if self.current_inspection_id:
                inspection.id_inspection = self.current_inspection_id
                inspection.inspection_status = 'FAILED'
                inspection.state = 'ERROR'
                inspection.details = f"Execution error: {str(e)}"
                inspection.modification_date = datetime.now()
                inspection.user_modification = self.user.id_user
                self.db_connection.update_inspection(inspection)
            
            return False
    
    def _execute_stage(self, stage: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a single stage"""
        stage_name = stage['name']
        steps = stage['steps']
        together = stage.get('together', False)
        
        self.logger.info(f"Executing stage: {stage_name}")
        
        stage_result = {
            'stage_name': stage_name,
            'success': True,
            'steps': [],
            'start_time': datetime.now(),
            'end_time': None
        }
        
        try:
            if together:
                # Execute steps in parallel (simplified sequential for now)
                self.logger.info("Executing steps together (sequential for now)")
            
            # Execute each step
            for step in steps:
                step_result = self._execute_step(step)
                stage_result['steps'].append(step_result)
                
                if not step_result['success']:
                    stage_result['success'] = False
                    self.logger.error(f"Step failed: {step_result.get('name', 'unknown')}")
            
            stage_result['end_time'] = datetime.now()
            stage_result['duration'] = (stage_result['end_time'] - stage_result['start_time']).total_seconds()
            
            self.logger.info(f"Stage {stage_name} completed. Success: {stage_result['success']}")
            return stage_result
            
        except Exception as e:
            self.logger.error(f"Stage execution error: {e}")
            stage_result['success'] = False
            stage_result['error'] = str(e)
            stage_result['end_time'] = datetime.now()
            return stage_result
    
    def _execute_step(self, step: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a single step"""
        step_name = step['name']
        system_name = step['system']
        
        self.logger.info(f"Executing step: {step_name} on system {system_name}")
        
        step_result = {
            'name': step_name,
            'system': system_name,
            'success': False,
            'start_time': datetime.now(),
            'end_time': None
        }
        
        try:
            # Get system
            system = self.systems[system_name]
            
            # Initialize system if not already done
            if not system.is_initialized:
                if not system.initialize():
                    raise RuntimeError(f"Failed to initialize system {system_name}")
            
            # Execute step
            execution_result = system.execute_step(step)
            step_result.update(execution_result)
            
            # Organize step data in output folder
            if step_result.get('success') and self.current_inspection_folder:
                organized_path = self.file_manager.organize_step_data(
                    self.current_inspection_folder, step_name, step_result
                )
                step_result['organized_path'] = organized_path
            
            step_result['end_time'] = datetime.now()
            step_result['duration'] = (step_result['end_time'] - step_result['start_time']).total_seconds()
            
            self.logger.info(f"Step {step_name} completed. Success: {step_result['success']}")
            return step_result
            
        except Exception as e:
            self.logger.error(f"Step execution error: {e}")
            step_result['success'] = False
            step_result['error'] = str(e)
            step_result['end_time'] = datetime.now()
            return step_result
    
    def cleanup_systems(self):
        """Cleanup all systems"""
        self.logger.info("Cleaning up all systems")
        for system_name, system in self.systems.items():
            try:
                system.cleanup()
            except Exception as e:
                self.logger.error(f"Error cleaning up system {system_name}: {e}")
    
    def get_available_programs(self) -> List[str]:
        """Get list of available program files"""
        try:
            programs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'programs')
            programs = []
            
            if os.path.exists(programs_dir):
                for file in os.listdir(programs_dir):
                    if file.endswith('.yaml') or file.endswith('.yml'):
                        programs.append(os.path.join(programs_dir, file))
            
            return programs
            
        except Exception as e:
            self.logger.error(f"Error getting available programs: {e}")
            return []
