"""
System Manager for coordinating different inspection systems
"""
import logging
from typing import Dict, Any, Optional
from config.config_manager import ConfigManager
from systems.base_system import BaseSystem
from systems.scanner_system import ScannerSystem
from systems.gantry_system import GantrySystem
from systems.table_system import TableSystem
from systems.camera_system import CameraSystem
from utils.file_manager import FileManager


class SystemManager:
    """Manages all inspection systems and coordinates their execution"""
    
    def __init__(self, config_manager: ConfigManager):
        self.config_manager = config_manager
        self.logger = logging.getLogger(__name__)
        self.systems: Dict[str, BaseSystem] = {}
        self.current_inspection_folder = None
        
        # Initialize file manager
        output_config = config_manager.get_output_config()
        output_dir = output_config.get('base_directory', './output')
        self.file_manager = FileManager(output_dir)
        
        self._initialize_systems()
        
    def _initialize_systems(self):
        """Initialize all available systems"""
        systems_config = self.config_manager.get_systems_config()
        
        for system_name, system_config in systems_config.items():
            try:
                system = self._create_system(system_name, system_config)
                if system and system.initialize():
                    self.systems[system_name] = system
                    self.logger.info(f"System {system_name} initialized successfully")
                else:
                    self.logger.warning(f"Failed to initialize system {system_name}")
            except Exception as e:
                self.logger.error(f"Error initializing system {system_name}: {e}")
                
    def _create_system(self, system_name: str, system_config: Dict[str, Any]) -> Optional[BaseSystem]:
        """Create a system instance based on its type"""
        system_type = system_config.get('type', '').lower()
        
        if system_type == 'scanner_system' or system_name.lower() == 'scancontrol':
            return ScannerSystem(system_name, system_config)
        elif system_type == 'camera_system' or system_name.lower() == 'camera':
            return CameraSystem(system_name, system_config)
        elif system_type == 'rotating_table' or system_name.lower() == 'table':
            return TableSystem(system_name, system_config)
        elif system_type == 'gantry_system' or system_name.lower() == 'gantry':
            return GantrySystem(system_name, system_config)
        else:
            self.logger.warning(f"Unknown system type: {system_type} for system: {system_name}")
            return None
    
    def create_inspection_folder(self, program_data: Dict[str, Any]) -> str:
        """Create inspection folder for the current program execution"""
        try:
            # Extract program information
            program_name = program_data.get('name', 'unknown_program')
            piece_info = program_data.get('piece_info', {})
            piece_name = piece_info.get('name_piece', 'unknown_piece')
            ref_piece = piece_info.get('ref_piece', 'unknown_ref')
            
            # Create inspection folder
            inspection_folder = self.file_manager.create_inspection_folder(
                program_name, piece_name, ref_piece
            )
            
            # Set the current inspection folder
            self.current_inspection_folder = inspection_folder
            
            # Notify all systems about the inspection folder
            self._set_inspection_folder_for_systems(inspection_folder)
            
            self.logger.info(f"Created inspection folder: {inspection_folder}")
            return inspection_folder
            
        except Exception as e:
            self.logger.error(f"Failed to create inspection folder: {e}")
            raise
    
    def _set_inspection_folder_for_systems(self, folder_path: str):
        """Set the inspection folder path for all systems that support it"""
        for system_name, system in self.systems.items():
            try:
                if hasattr(system, 'set_inspection_folder'):
                    system.set_inspection_folder(folder_path)
                    self.logger.debug(f"Set inspection folder for {system_name}")
            except Exception as e:
                self.logger.warning(f"Failed to set inspection folder for {system_name}: {e}")
    
    def get_current_inspection_folder(self) -> Optional[str]:
        """Get the current inspection folder path"""
        return self.current_inspection_folder
            
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step on the appropriate system"""
        system_name = step_config.get('system')
        
        if not system_name:
            raise ValueError("Step configuration missing 'system' field")
            
        if system_name not in self.systems:
            raise ValueError(f"System '{system_name}' not available or not initialized")
            
        system = self.systems[system_name]
        self.logger.info(f"Executing step on system: {system_name}")
        
        try:
            result = system.execute_step(step_config)
            self.logger.info(f"Step executed successfully on {system_name}")
            return result
        except Exception as e:
            self.logger.error(f"Step execution failed on {system_name}: {e}")
            raise
            
    def get_system_status(self, system_name: str) -> Dict[str, Any]:
        """Get status of a specific system by testing its connection"""
        if system_name not in self.systems:
            return {'status': 'not_available', 'message': 'System not initialized'}
            
        try:
            # Test actual connection to the system using GUI connection functions
            system = self.systems[system_name]
            return system.test_connection()
        except Exception as e:
            self.logger.error(f"Error testing system {system_name}: {e}")
            return {'status': 'error', 'message': f'Connection test failed: {str(e)}'}
        
    def get_all_systems_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all systems by testing their connections"""
        status = {}
        systems_config = self.config_manager.get_systems_config()
        
        for system_name in systems_config.keys():
            self.logger.info(f"Testing connection for system: {system_name}")
            status[system_name] = self.get_system_status(system_name)
            
        return status
        
    def shutdown_all_systems(self):
        """Shutdown all systems gracefully"""
        for system_name, system in self.systems.items():
            try:
                if hasattr(system, 'shutdown'):
                    system.shutdown()
                    self.logger.info(f"System {system_name} shut down")
            except Exception as e:
                self.logger.error(f"Error shutting down system {system_name}: {e}")
                
        self.systems.clear()
