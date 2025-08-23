"""
FTP Manager for AUAS Inspection Engine
Simple implementation for uploading inspection data
"""
import ftplib
import os
import logging
from datetime import datetime
from typing import Dict, Any, Tuple

class FTPManager:
    """Simple FTP manager for uploading inspection data"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.ftp_connection = None

    def _parse_server_url(self) -> Tuple[str, int]:
        """Parse FTP server URL to extract host and port"""
        server = self.config.get('server', 'ftp://localhost')

        # Remove ftp:// if present
        if server.startswith('ftp://'):
            server = server[6:]

        # Check if port is specified
        if ':' in server:
            host, port_str = server.split(':')
            try:
                port = int(port_str)
            except ValueError:
                port = 21  # Default FTP port
        else:
            host = server
            port = 21

        return host, port

    def test_connection(self) -> Dict[str, Any]:
        """Test simple FTP connection"""
        try:
            host, port = self._parse_server_url()
            username = self.config.get('username', 'anonymous')
            password = self.config.get('password', '')

            self.logger.debug("Testing FTP connection to %s, %s", host, port)
            self.logger.debug("Username: %s", username)

            # Create simple FTP connection
            ftp = ftplib.FTP()

            # Connect to server
            ftp.connect(host, port, timeout=30)

            # Set passive mode
            passive_mode = self.config.get('passive_mode', True)
            ftp.set_pasv(passive_mode)
            self.logger.debug("Passive mode set to: %s", passive_mode)

            # Login
            ftp.login(username, password)

            # Test by listing current directory
            files = ftp.nlst()
            self.logger.debug("Successfully connected. Files in root: %d items", len(files))

            # Test base path if specified
            base_path = self.config.get('base_path', '/')
            if base_path and base_path != '/':
                try:
                    ftp.cwd(base_path)
                    self.logger.debug("Successfully accessed base path: %s", base_path)
                except ftplib.error_perm as e:
                    self.logger.debug("Base path %s not accessible: %s", base_path, e)
                    # Try to create the base path
                    try:
                        ftp.mkd(base_path)
                        ftp.cwd(base_path)
                        self.logger.debug("Created and accessed base path: %s", base_path)
                    except ftplib.error_perm:
                        ftp.quit()
                        return {
                            "success": False,
                            "message": f"Cannot access or create base path: {base_path}"
                        }

            ftp.quit()

            return {
                "success": True,
                "message": f"Successfully connected to FTP server {host}:{port}",
                "details": {
                    "host": host,
                    "port": port,
                    "base_path": base_path,
                    "files_in_root": len(files),
                    "encryption": "None (Simple FTP)"
                }
            }

        except ftplib.error_perm as e:
            error_msg = f"FTP permission error: {str(e)}"
            self.logger.debug(error_msg)
            return {
                "success": False,
                "message": error_msg
            }
        except ftplib.error_temp as e:
            error_msg = f"FTP temporary error: {str(e)}"
            self.logger.debug(error_msg)
            return {
                "success": False,
                "message": error_msg
            }
        except ConnectionRefusedError:
            error_msg = "Connection refused. FTP server may not be running."
            self.logger.debug(error_msg)
            return {
                "success": False,
                "message": error_msg
            }
        except Exception as e:
            error_msg = f"FTP connection error: {str(e)}"
            self.logger.debug(error_msg)
            return {
                "success": False,
                "message": error_msg
            }

    def _connect(self) -> bool:
        """Establish simple FTP connection"""
        try:
            if self.ftp_connection:
                try:
                    # Test if connection is still alive
                    self.ftp_connection.pwd()
                    return True
                except Exception:
                    # Connection is dead, close it
                    try:
                        self.ftp_connection.quit()
                    except Exception:
                        pass
                    self.ftp_connection = None

            host, port = self._parse_server_url()
            username = self.config.get('username', 'anonymous')
            password = self.config.get('password', '')

            # Create simple FTP connection
            self.ftp_connection = ftplib.FTP()

            # Connect to server
            self.ftp_connection.connect(host, port, timeout=30)

            # Set passive mode
            passive_mode = self.config.get('passive_mode', True)
            self.ftp_connection.set_pasv(passive_mode)
            self.logger.debug("Passive mode set to: %s", passive_mode)

            # Login
            self.ftp_connection.login(username, password)

            # Change to base path if specified
            base_path = self.config.get('base_path', '/')
            if base_path and base_path != '/':
                try:
                    self.ftp_connection.cwd(base_path)
                except ftplib.error_perm:
                    # Try to create the base path
                    self.ftp_connection.mkd(base_path)
                    self.ftp_connection.cwd(base_path)

            return True

        except Exception as e:
            self.logger.error("Failed to connect to FTP server: %s", e)
            self.ftp_connection = None
            return False

    def _disconnect(self):
        """Close FTP connection"""
        if self.ftp_connection:
            try:
                self.ftp_connection.quit()
            except:
                pass
            self.ftp_connection = None

    def _create_directory_if_not_exists(self, directory: str) -> bool:
        """Create directory on FTP server if it doesn't exist"""
        try:
            # Try to change to the directory
            self.ftp_connection.cwd(directory)
            return True
        except ftplib.error_perm:
            # Directory doesn't exist, try to create it
            try:
                self.ftp_connection.mkd(directory)
                self.ftp_connection.cwd(directory)
                self.logger.debug("Created FTP directory: %s", directory)
                return True
            except ftplib.error_perm as e:
                self.logger.error("Failed to create FTP directory %s: %s", directory, e)
                return False

    def upload_inspection_folder(self, local_folder_path: str, inspection_date: datetime = None) -> Dict[str, Any]:
        """Upload inspection folder to FTP server with date organization"""
        try:
            if not os.path.exists(local_folder_path):
                return {
                    "success": False,
                    "message": f"Local folder does not exist: {local_folder_path}"
                }

            if not self._connect():
                return {
                    "success": False,
                    "message": "Failed to connect to FTP server"
                }

            # Use provided date or current date
            if inspection_date is None:
                inspection_date = datetime.now()

            # Create date folder (YYYY-MM-DD format)
            date_folder = inspection_date.strftime("%Y-%m-%d")

            # Get inspection folder name (last part of path)
            inspection_folder_name = os.path.basename(local_folder_path)

            self.logger.debug("Uploading %s to FTP as %s/%s", local_folder_path, date_folder, inspection_folder_name)

            # Navigate to base path
            base_path = self.config.get('base_path', '/')
            if base_path and base_path != '/':
                self.ftp_connection.cwd(base_path)

            # Create/navigate to date folder
            if not self._create_directory_if_not_exists(date_folder):
                self._disconnect()
                return {
                    "success": False,
                    "message": f"Failed to create date folder: {date_folder}"
                }

            # Create/navigate to inspection folder
            if not self._create_directory_if_not_exists(inspection_folder_name):
                self._disconnect()
                return {
                    "success": False,
                    "message": f"Failed to create inspection folder: {inspection_folder_name}"
                }

            # Upload all files in the folder
            uploaded_files = []
            failed_files = []

            for root, dirs, files in os.walk(local_folder_path):
                # Calculate relative path from the inspection folder
                rel_path = os.path.relpath(root, local_folder_path)

                # Create subdirectories on FTP if needed
                if rel_path != '.':
                    # Convert Windows path separators to forward slashes for FTP
                    ftp_rel_path = rel_path.replace('\\', '/')
                    try:
                        self.ftp_connection.mkd(ftp_rel_path)
                    except ftplib.error_perm:
                        # Directory might already exist
                        pass

                # Upload files
                for file in files:
                    local_file_path = os.path.join(root, file)

                    # Calculate FTP path
                    if rel_path == '.':
                        ftp_file_path = file
                    else:
                        ftp_file_path = rel_path.replace('\\', '/') + '/' + file

                    try:
                        with open(local_file_path, 'rb') as f:
                            self.ftp_connection.storbinary(f'STOR {ftp_file_path}', f)
                        uploaded_files.append(ftp_file_path)
                        self.logger.debug("Uploaded: %s", ftp_file_path)
                    except Exception as e:
                        failed_files.append(f"{ftp_file_path}: {str(e)}")
                        self.logger.error("Failed to upload %s: %s", ftp_file_path, e)

            self._disconnect()

            if failed_files:
                return {
                    "success": False,
                    "message": f"Upload completed with {len(failed_files)} failures",
                    "details": {
                        "uploaded_files": uploaded_files,
                        "failed_files": failed_files,
                        "ftp_path": f"{base_path}/{date_folder}/{inspection_folder_name}"
                    }
                }
            else:
                return {
                    "success": True,
                    "message": f"Successfully uploaded {len(uploaded_files)} files",
                    "details": {
                        "uploaded_files": uploaded_files,
                        "ftp_path": f"{base_path}/{date_folder}/{inspection_folder_name}"
                    }
                }

        except Exception as e:
            self._disconnect()
            error_msg = f"FTP upload error: {str(e)}"
            self.logger.error(error_msg)
            return {
                "success": False,
                "message": error_msg
            }
