"""
Base system class for all inspection systems
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from datetime import datetime
import logging

class BaseSystem(ABC):
    """Base class for all inspection systems"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        self.name = name
        self.config = config
        self.logger = logging.getLogger(f"system.{name}")
        self.is_initialized = False
        
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the system"""
        pass
        
    @abstractmethod
    def execute_step(self, step_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a step with given configuration
        
        Args:
            step_config: Configuration for the step to execute
            
        Returns:
            Dict containing execution results and metadata
        """
        pass
        
    @abstractmethod
    def test_connection(self) -> Dict[str, Any]:
        """Test connection to the system
        
        Returns:
            Dict containing connection status and details
        """
        pass
        
    def cleanup(self) -> None:
        """Cleanup system resources"""
        pass
        
    def shutdown(self) -> None:
        """Shutdown the system gracefully"""
        self.cleanup()
        
    def validate_parameters(self, parameters: Dict[str, Any]) -> bool:
        """Validate step parameters"""
        return True
    
    def get_status(self) -> Dict[str, Any]:
        """Get current system status"""
        return {
            "name": self.name,
            "initialized": self.is_initialized,
            "config": self.config
        }
    
    def get_timestamp(self) -> str:
        """Get current timestamp"""
        return datetime.now().isoformat()
