#!/usr/bin/env python3
"""
Simple Python FTP Server for AUAS Inspection Engine
Based on pyftpdlib - simple, reliable, and easy to configure
"""
import os
import sys
import logging
from pathlib import Path

# Load environment variables
from dotenv import load_dotenv

load_dotenv(dotenv_path=Path(__file__).parent.parent / ".env")
import os

try:
    from pyftpdlib.authorizers import DummyAuthorizer
    from pyftpdlib.handlers import FTPHandler
    from pyftpdlib.servers import FTPServer
except ImportError:
    print("❌ pyftpdlib not installed. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyftpdlib"])
    from pyftpdlib.authorizers import DummyAuthorizer
    from pyftpdlib.handlers import FTPHandler
    from pyftpdlib.servers import FTPServer



class AUASFTPServer:
    """Simple FTP Server for AUAS Inspection Engine"""

    def __init__(self, host=None, port=None, base_path=None):
        # Load from environment variables if not provided
        self.host = host or os.getenv("FTP_HOST", "127.0.0.1")
        self.port = int(port or os.getenv("FTP_PORT", 21))
        self.base_path = Path(base_path or os.getenv("FTP_BASE_PATH", r"C:\\Users\\Agordien\\Documents\\projects\\AUAS\\FTP"))
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
            self.setup_directories()
            self.create_server()
            
            self.logger.info("🚀 Starting FTP Server...")
            self.logger.info("🔗 Connect with: ftp://127.0.0.1:21")
            self.logger.info("⚠️  Press Ctrl+C to stop")
            
            self.server.serve_forever()
            
        except KeyboardInterrupt:
            self.logger.info("🛑 Server stopped by user")
        except Exception as e:
            self.logger.error(f"❌ Server error: {e}")
        finally:
            if self.server:
                self.server.close_all()
    
    def stop(self):
        """Stop the FTP server"""
        if self.server:
            self.server.close_all()
            self.logger.info("🛑 FTP Server stopped")



def main():
    """Main function to start FTP server"""
    print("🏗️ AUAS Inspection Engine - FTP Server")
    print("=" * 50)

    # Create and start server (parameters will be loaded from .env by default)
    ftp_server = AUASFTPServer()

    try:
        ftp_server.start()
    except Exception as e:
        print(f"❌ Failed to start server: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
