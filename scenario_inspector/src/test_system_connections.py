#!/usr/bin/env python3
"""
Test script to verify system connections using the updated SystemManager
"""
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from config.config_manager import ConfigManager
from systems.system_manager import SystemManager
import logging

def setup_logging():
    """Setup basic logging"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

def test_system_connections():
    """Test all system connections"""
    print("=" * 60)
    print("AUAS Inspection Engine - System Connection Test")
    print("=" * 60)
    
    try:
        # Initialize configuration manager
        print("\n1. Loading configuration...")
        config_manager = ConfigManager()
        systems_config = config_manager.get_systems_config()
        
        print(f"Found {len(systems_config)} systems in configuration:")
        for system_name, system_config in systems_config.items():
            connection_type = system_config.get('connection_type', 'unknown')
            system_type = system_config.get('type', 'unknown')
            print(f"  - {system_name}: {system_type} ({connection_type})")
        
        # Initialize system manager
        print("\n2. Initializing System Manager...")
        system_manager = SystemManager(config_manager)
        
        # Test all system connections
        print("\n3. Testing system connections...")
        print("-" * 40)
        
        all_status = system_manager.get_all_systems_status()
        
        available_systems = 0
        total_systems = len(all_status)
        
        for system_name, status_info in all_status.items():
            status = status_info.get('status', 'unknown')
            message = status_info.get('message', 'No message')
            details = status_info.get('details', {})
            
            # Get system config for display
            system_config = config_manager.get_system_config(system_name)
            connection_type = system_config.get('connection_type', 'unknown')
            
            # Format connection info
            if connection_type == 'tcp':
                ip = system_config.get('ip', 'N/A')
                port = system_config.get('port', 'N/A')
                conn_info = f"{ip}:{port}"
            elif connection_type == 'serial':
                port = system_config.get('port', 'N/A')
                baudrate = system_config.get('baudrate', 'N/A')
                conn_info = f"{port} @ {baudrate}"
            elif connection_type == 'usb':
                device_id = system_config.get('device_id', 'N/A')
                conn_info = device_id
            else:
                conn_info = "N/A"
            
            # Display status
            if status == 'available':
                status_icon = "✅"
                available_systems += 1
            elif status == 'not_available':
                status_icon = "❌"
            elif status == 'error':
                status_icon = "⚠️"
            else:
                status_icon = "❓"
            
            print(f"{status_icon} {system_name:12} | {connection_type:8} | {conn_info:20} | {message}")
            
            # Show details if available
            if details:
                for key, value in details.items():
                    print(f"{'':15}   └─ {key}: {value}")
        
        print("-" * 40)
        print(f"\nSummary: {available_systems}/{total_systems} systems available")
        
        if available_systems == total_systems:
            print("🎉 All systems are connected and ready!")
        elif available_systems > 0:
            print(f"⚠️  {total_systems - available_systems} system(s) not available")
        else:
            print("❌ No systems are available")
        
        print("\n4. Testing system shutdown...")
        system_manager.shutdown_all_systems()
        print("✅ System manager shut down successfully")
        
        return available_systems == total_systems
        
    except Exception as e:
        print(f"\n❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function"""
    setup_logging()
    
    success = test_system_connections()
    
    print("\n" + "=" * 60)
    if success:
        print("✅ System connection test PASSED")
        exit_code = 0
    else:
        print("❌ System connection test FAILED")
        exit_code = 1
    
    print("=" * 60)
    return exit_code

if __name__ == "__main__":
    exit(main())
