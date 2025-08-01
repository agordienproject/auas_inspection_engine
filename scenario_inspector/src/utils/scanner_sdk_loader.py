"""
Utility functions for loading the scanCONTROL SDK
"""
import sys
import os
import ctypes as ct


class ScannerSDKLoader:
    """Utility class for loading the scanCONTROL SDK"""
    
    _instance = None
    _sdk_loaded = False
    _llt_module = None
    _sdk_path = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ScannerSDKLoader, cls).__new__(cls)
        return cls._instance
    
    @classmethod
    def load_sdk(cls, llt_path: str) -> bool:
        """Load the scanCONTROL SDK from the specified path"""
        if cls._sdk_loaded and cls._sdk_path == llt_path:
            return True  # Already loaded from this path
        
        try:
            # Add scanCONTROL SDK path to Python path
            if llt_path and os.path.exists(llt_path):
                if llt_path not in sys.path:
                    sys.path.insert(0, llt_path)
                
                import pylinllt as llt_module
                cls._llt_module = llt_module
                cls._sdk_loaded = True
                cls._sdk_path = llt_path
                print(f"Successfully loaded scanCONTROL SDK from: {llt_path}")
                return True
            else:
                print(f"Warning: scanCONTROL SDK path not found: {llt_path}")
                return False
                
        except ImportError as e:
            print(f"Warning: scanCONTROL SDK not available at {llt_path}: {e}")
            cls._sdk_loaded = False
            return False
    
    @classmethod
    def is_sdk_available(cls) -> bool:
        """Check if SDK is available"""
        return cls._sdk_loaded
    
    @classmethod
    def get_llt_module(cls):
        """Get the loaded LLT module"""
        return cls._llt_module if cls._sdk_loaded else None
    
    @classmethod
    def get_sdk_path(cls) -> str:
        """Get the current SDK path"""
        return cls._sdk_path or "Not loaded"


def load_scanner_sdk_from_config(config: dict, default_path: str = None) -> bool:
    """Load scanner SDK using configuration"""
    llt_path = config.get('llt_path')
    
    if not llt_path and default_path:
        llt_path = default_path
    
    if llt_path:
        return ScannerSDKLoader.load_sdk(llt_path)
    else:
        print("Warning: No llt_path specified in configuration")
        return False


def get_scanner_sdk():
    """Get the scanner SDK module if available"""
    return ScannerSDKLoader.get_llt_module()


def is_scanner_sdk_available() -> bool:
    """Check if scanner SDK is available"""
    return ScannerSDKLoader.is_sdk_available()
