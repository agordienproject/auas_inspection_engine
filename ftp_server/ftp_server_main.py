#!/usr/bin/env python3
"""
Main executable entry point for AUAS FTP Server
This file will be used to create the standalone executable
"""
import os
import sys
import logging
from pathlib import Path

# Add the current directory to Python path
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

try:
    from pyftpdlib.authorizers import DummyAuthorizer
    from pyftpdlib.handlers import FTPHandler
    from pyftpdlib.servers import FTPServer
except ImportError:
    print("❌ pyftpdlib not found. Please install dependencies first.")
    print("Run: pip install pyftpdlib")
    input("Press Enter to exit...")
    sys.exit(1)


class AUASFTPServer:
    """Simple FTP Server for AUAS Inspection Engine"""
    
    def __init__(self, 
                 host="127.0.0.1", 
                 port=21, 
                 base_path=None):
        self.host = host
        self.port = port
        
        # Default base path - use a folder next to the executable
        if base_path is None:
            if getattr(sys, 'frozen', False):
                # Running as executable
                exe_dir = Path(sys.executable).parent
                self.base_path = exe_dir / "FTP_Data"
            else:
                # Running as script
                self.base_path = Path(r"C:\Users\Agordien\Documents\projects\AUAS\FTP")
        else:
            self.base_path = Path(base_path)
            
        self.server = None
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def setup_directories(self):
        """Create necessary directories"""
        try:
            self.base_path.mkdir(parents=True, exist_ok=True)
            (self.base_path / "inspections").mkdir(exist_ok=True)
            
            # Create a test file
            test_file = self.base_path / "server_info.txt"
            with open(test_file, 'w') as f:
                f.write("AUAS FTP Server - Ready for inspection uploads\n")
                f.write(f"Server started at: {self.host}:{self.port}\n")
                f.write(f"Base directory: {self.base_path}\n")
                
            self.logger.info(f"✅ FTP directories created at: {self.base_path}")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to create directories: {e}")
            raise
    
    def create_server(self):
        """Create and configure FTP server"""
        try:
            # Create authorizer and add users
            authorizer = DummyAuthorizer()
            
            # Add the inspection_engine user with read/write permissions
            authorizer.add_user(
                username="inspection_engine",
                password="admin",
                homedir=str(self.base_path),
                perm="elradfmwMT"  # Full permissions
            )
            
            # Optional: Add anonymous user with read-only access
            authorizer.add_anonymous(
                homedir=str(self.base_path),
                perm="elr"  # Read-only
            )
            
            # Create FTP handler
            handler = FTPHandler
            handler.authorizer = authorizer
            
            # Configure handler settings
            handler.banner = "AUAS Inspection Engine FTP Server Ready"
            handler.max_cons = 256
            handler.max_cons_per_ip = 10
            
            # Create server
            self.server = FTPServer((self.host, self.port), handler)
            
            self.logger.info(f"✅ FTP Server configured")
            self.logger.info(f"📡 Host: {self.host}")
            self.logger.info(f"🔌 Port: {self.port}")
            self.logger.info(f"📁 Base Path: {self.base_path}")
            self.logger.info(f"👤 User: inspection_engine / admin")
            self.logger.info(f"🌐 Anonymous: enabled (read-only)")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to create server: {e}")
            raise
    
    def start(self):
        """Start the FTP server"""
        try:
            print("🏗️ AUAS FTP Server Starting...")
            print("=" * 50)
            
            self.setup_directories()
            self.create_server()
            
            print(f"\n🚀 FTP Server is running!")
            print(f"📡 Address: ftp://{self.host}:{self.port}")
            print(f"📁 Data folder: {self.base_path}")
            print(f"👤 Username: inspection_engine")
            print(f"🔑 Password: admin")
            print(f"🌐 Anonymous access: enabled (read-only)")
            print("\n" + "=" * 50)
            print("Press Ctrl+C to stop the server")
            print("=" * 50)
            
            # Start serving
            self.server.serve_forever()
            
        except KeyboardInterrupt:
            print("\n🛑 Server stopped by user")
            return 0
        except OSError as e:
            if "Address already in use" in str(e):
                print(f"❌ Port {self.port} is already in use!")
                print("Either stop the existing FTP server or change the port.")
                input("Press Enter to exit...")
                return 1
            else:
                print(f"❌ Network error: {e}")
                input("Press Enter to exit...")
                return 1
        except Exception as e:
            print(f"❌ Server error: {e}")
            input("Press Enter to exit...")
            return 1
    
    def stop(self):
        """Stop the FTP server"""
        if self.server:
            self.server.close_all()
            self.logger.info("🛑 FTP Server stopped")


def main():
    """Main entry point"""
    server = AUASFTPServer()
    return server.start()


if __name__ == "__main__":
    sys.exit(main())
