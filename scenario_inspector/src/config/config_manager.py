"""
Configuration Manager for Scenario Inspector
"""
import yaml
import os
from typing import Dict, Any

class ConfigManager:
    """Manages application configuration from YAML files"""
    
    def __init__(self, config_path: str = None):
        if config_path is None:
            # Default to config/app_config.yaml relative to the project root
            src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            project_dir = os.path.dirname(src_dir)
            config_path = os.path.join(project_dir, 'config', 'app_config.yaml')
        
        self.config_path = config_path
        self._config = None
        self.load_config()
    
    def load_config(self) -> None:
        """Load configuration from YAML file"""
        try:
            with open(self.config_path, 'r') as file:
                self._config = yaml.safe_load(file)
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in configuration file: {e}")
    
    def get_config(self) -> Dict[str, Any]:
        """Get the complete configuration dictionary"""
        if self._config is None:
            self.load_config()
        return self._config
    
    def get_database_config(self) -> Dict[str, Any]:
        """Get database configuration"""
        return self.get_config().get('database', {})
    
    def get_systems_config(self) -> Dict[str, Any]:
        """Get systems configuration"""
        return self.get_config().get('systems', {})
    
    def get_system_config(self, system_name: str) -> Dict[str, Any]:
        """Get configuration for a specific system"""
        systems = self.get_systems_config()
        return systems.get(system_name, {})
    
    def get_ftp_config(self) -> Dict[str, Any]:
        """Get FTP configuration"""
        return self.get_config().get('ftp', {})
    
    def get_output_config(self) -> Dict[str, Any]:
        """Get output configuration"""
        return self.get_config().get('output', {})
    
    def get_logging_config(self) -> Dict[str, Any]:
        """Get logging configuration"""
        return self.get_config().get('logging', {})
    
    def get_security_config(self) -> Dict[str, Any]:
        """Get security configuration"""
        return self.get_config().get('security', {})
    
    def get_gui_config(self) -> Dict[str, Any]:
        """Get GUI configuration"""
        return self.get_config().get('gui', {})
    
    def reload_config(self) -> None:
        """Reload configuration from file"""
        self.load_config()
    
    def save_config(self, new_config: Dict[str, Any]) -> None:
        """Save configuration to file"""
        try:
            with open(self.config_path, 'w') as file:
                yaml.dump(new_config, file, default_flow_style=False, indent=2)
            self._config = new_config
        except Exception as e:
            raise IOError(f"Failed to save configuration: {e}")
    
    def update_config_section(self, section: str, new_values: Dict[str, Any]) -> None:
        """Update a specific section of the configuration"""
        if self._config is None:
            self.load_config()
        
        if section not in self._config:
            self._config[section] = {}
        
        self._config[section].update(new_values)
        self.save_config(self._config)

    def get_global_parameters(self):
        return self.config_data.get('global_parameters', {})

    def get_inspection_stages(self):
        return self.config_data.get('inspection_stages', [])

    def get_output_directory(self):
        return self.config_data.get('output_directory', os.path.join(os.getcwd(), 'output'))