"""
Scenario execution engine for running inspection programs
"""
import os
import yaml
import logging
from datetime import datetime
from typing import Dict, Any, List, Optional
from dataclasses import asdict

from config.config_manager import ConfigManager
from database.connection import DatabaseConnection
from database.models import Inspection, User
from systems.system_manager import SystemManager
from utils.file_manager import FileManager

class ScenarioEngine:
    """Main engine for executing inspection scenarios"""
    
    def __init__(self, user: User):
        self.user = user
        self.logger = logging.getLogger(__name__)
        self.config_manager = ConfigManager()
        self.db_connection = DatabaseConnection()
        
        # Initialize system manager instead of managing systems directly
        self.system_manager = SystemManager(self.config_manager)
        
        # Current scenario state
        self.current_scenario = None
        self.current_inspection_id = None
        self.current_inspection_folder = None
    
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
            # Create inspection folder using SystemManager
            program_data = {
                'name': program_name,
                'piece_info': {
                    'name_piece': piece_name,
                    'ref_piece': ref_piece
                }
            }
            self.current_inspection_folder = self.system_manager.create_inspection_folder(program_data)
            
            # Execute all stages
            execution_results = []
            overall_success = True
            
            for stage in program['stages']:
                stage_result = self._execute_stage(stage)
                execution_results.append(stage_result)
                if not stage_result['success']:
                    overall_success = False
            
            # Create inspection report
            report_data = {
                'program_name': program_name,
                'piece_name': piece_name,
                'ref_piece': ref_piece,
                'inspection_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'inspector': f"{self.user.first_name} {self.user.last_name}",
                'steps': execution_results
            }
            
            self.system_manager.file_manager.create_inspection_report(
                self.current_inspection_folder, report_data
            )
            
            # Insert inspection data into database
            inspection = Inspection(
                name_piece=piece_name,
                ref_piece=ref_piece,
                name_program=program_name,
                state='GOOD' if overall_success else 'NOT_GOOD',
                inspection_date=datetime.now().date(),
                inspection_path=self.current_inspection_folder,
                inspection_status='COMPLETED' if overall_success else 'FAILED',
                creation_date=datetime.now(),
                user_creation=self.user.id_user,
                details=f"Executed {len(execution_results)} steps. Success rate: {sum(1 for r in execution_results if r.get('success', False))}/{len(execution_results)}"
            )
            
            if self.db_connection.insert_inspection(inspection):
                self.logger.info("Inspection data saved to database")
            else:
                self.logger.warning("Failed to save inspection data to database")
            
            self.logger.info(f"Scenario execution completed. Status: {'SUCCESS' if overall_success else 'FAILED'}")
            return overall_success
            
        except Exception as e:
            self.logger.error(f"Scenario execution failed: {e}")
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
            # Execute step using SystemManager
            execution_result = self.system_manager.execute_step(step)
            step_result.update(execution_result)
            
            # Organize step data in output folder
            if step_result.get('success') and self.current_inspection_folder:
                organized_path = self.system_manager.file_manager.organize_step_data(
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
    

