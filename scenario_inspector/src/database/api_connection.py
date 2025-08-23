"""
API Connection module for AUAS Inspection Engine
Replaces direct database access with secure API calls
"""
import logging
from typing import Dict, Any, Optional
from datetime import datetime
import requests


class APIConnection:
    """API connection handler for secure data access"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.session = requests.Session()
        self.auth_token = None
        self.base_url = self._build_base_url()

    def _build_base_url(self) -> str:
        """Build the base URL from config"""
        url = self.config.get('url', '127.0.0.1:3000/api')
        use_https = self.config.get('use_https', False)

        if not url.startswith(('http://', 'https://')):
            protocol = 'https://' if use_https else 'http://'
            url = protocol + url

        return url

    def test_connection(self) -> Dict[str, Any]:
        """Test API connection"""
        try:
            timeout = self.config.get('timeout', 30)
            self.logger.debug("Testing API connection to: %s", self.base_url)
            self.logger.debug("Connection timeout: %ss", timeout)
            self.logger.debug("Session headers: %s", dict(self.session.headers))

            # Try to reach the API base endpoint
            response = self.session.get(
                self.base_url,
                timeout=timeout
            )

            self.logger.debug("API response status: %s", response.status_code)
            self.logger.debug("API response headers: %s", dict(response.headers))
            self.logger.debug("API response time: %.3fs", response.elapsed.total_seconds())

            if response.status_code == 200:
                self.logger.debug("API connection successful to %s", self.base_url)
                return {
                    "status": "available",
                    "message": "API connection successful",
                    "details": {
                        "url": self.base_url,
                        "response_time_ms": int(response.elapsed.total_seconds() * 1000)
                    }
                }
            else:
                self.logger.debug("API connection failed with status %s", response.status_code)
                self.logger.debug("Response content: %s...", response.text[:200])
                return {
                    "status": "error",
                    "message": "API returned status code: %s" % response.status_code,
                    "details": {"url": self.base_url, "status_code": response.status_code}
                }

        except requests.exceptions.ConnectionError as e:
            self.logger.debug("Connection error: %s", str(e))
            return {
                "status": "not_available",
                "message": "Could not connect to API server",
                "details": {"url": self.base_url, "error": str(e)}
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug("Timeout error: %s", str(e))
            return {
                "status": "error",
                "message": "API connection timeout",
                "details": {"url": self.base_url, "timeout": self.config.get('timeout', 30), "error": str(e)}
            }
        except Exception as e:
            self.logger.debug("Unexpected error during API connection test: %s", str(e))
            return {
                "status": "error",
                "message": "API connection error: %s" % str(e),
                "details": {"url": self.base_url, "error": str(e)}
            }

    def authenticate(self, email: str, password: str) -> Dict[str, Any]:
        """Authenticate with the API using email and password"""
        try:
            login_url = self.base_url + self.config.get('login_endpoint', '/auth/login')
            timeout = self.config.get('timeout', 30)

            self.logger.debug("Attempting authentication for email: %s", email)
            self.logger.debug("Login URL: %s", login_url)
            self.logger.debug("Request timeout: %ss", timeout)

            login_data = {
                "email": email,
                "password": "***HIDDEN***"  # Don't log actual password
            }
            self.logger.debug("Login payload structure: %s", login_data)

            # Prepare actual login data with real password
            actual_login_data = {
                "email": email,
                "password": password
            }

            self.logger.debug("Sending POST request to %s", login_url)
            response = self.session.post(
                login_url,
                json=actual_login_data,
                timeout=timeout
            )

            self.logger.debug("Authentication response status: %s", response.status_code)
            self.logger.debug("Authentication response headers: %s", dict(response.headers))
            self.logger.debug("Authentication response time: %.3fs", response.elapsed.total_seconds())

            if response.status_code == 200 or response.status_code == 201:
                auth_data = response.json()
                self.logger.debug("Authentication successful, response keys: %s", list(auth_data.keys()))

                # Store authentication token
                self.auth_token = auth_data.get('token')
                if self.auth_token:
                    self.logger.debug("Received auth token (length: %s)", len(self.auth_token))
                    # Add token to session headers for future requests
                    self.session.headers.update({
                        'Authorization': 'Bearer %s' % self.auth_token
                    })
                    self.logger.debug("Updated session headers with auth token")
                else:
                    self.logger.debug("No auth token received in response")

                user_data = auth_data.get('user', {})
                self.logger.debug("User data received: %s", list(user_data.keys()) if user_data else 'None')

                self.logger.info("Successfully authenticated user: %s", email)
                return {
                    "success": True,
                    "user_data": user_data,
                    "token": self.auth_token,
                    "message": "Authentication successful"
                }

            elif response.status_code == 401:
                self.logger.debug("Authentication failed: Invalid credentials (401)")
                try:
                    error_response = response.json()
                    self.logger.debug("Error response content: %s", error_response)
                except:
                    self.logger.debug("Error response text: %s", response.text)
                return {
                    "success": False,
                    "message": "Invalid email or password"
                }
            else:
                self.logger.debug("Authentication failed with status %s", response.status_code)
                try:
                    error_response = response.json()
                    self.logger.debug("Error response content: %s", error_response)
                except:
                    self.logger.debug("Error response text: %s", response.text)
                return {
                    "success": False,
                    "message": "Authentication failed: HTTP %s" % response.status_code
                }

        except requests.exceptions.ConnectionError as e:
            self.logger.debug("Connection error during authentication: %s", str(e))
            return {
                "success": False,
                "message": "Could not connect to authentication server",
                "error": str(e)
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug("Timeout error during authentication: %s", str(e))
            return {
                "success": False,
                "message": "Authentication request timed out",
                "error": str(e)
            }
        except Exception as e:
            self.logger.error("Authentication error: %s", e)
            self.logger.debug("Full authentication error details: %s", str(e))
            return {
                "success": False,
                "message": "Authentication error: %s" % str(e)
            }

    def save_inspection_data(self, inspection_data: Dict[str, Any]) -> Dict[str, Any]:
        """Save inspection data via API"""
        try:
            if not self.auth_token:
                self.logger.debug("Cannot save inspection data: No authentication token")
                return {
                    "success": False,
                    "message": "Not authenticated. Please login first."
                }

            inspection_url = self.base_url + self.config.get('inspection_endpoint', '/inspections')
            self.logger.debug("Saving inspection data to: %s", inspection_url)
            self.logger.debug("Data keys: %s", list(inspection_data.keys()))
            self.logger.debug("Auth token available: %s", bool(self.auth_token))

            # Add timestamp if not present
            if 'timestamp' not in inspection_data:
                inspection_data['timestamp'] = datetime.now().isoformat()
                self.logger.debug("Added timestamp to inspection data: %s", inspection_data['timestamp'])

            self.logger.debug("Current session headers: %s", dict(self.session.headers))
            self.logger.debug("Sending POST request to save inspection data")

            response = self.session.post(
                inspection_url,
                json=inspection_data,
                timeout=self.config.get('timeout', 30)
            )

            self.logger.debug("Save inspection response status: %s", response.status_code)
            self.logger.debug("Save inspection response time: %.3fs", response.elapsed.total_seconds())

            if response.status_code in [200, 201]:
                result_data = response.json()
                self.logger.debug("Response data keys: %s", list(result_data.keys()))
                self.logger.info("Inspection data saved successfully: %s", result_data.get('id', 'N/A'))
                return {
                    "success": True,
                    "inspection_id": result_data.get('id'),
                    "message": "Inspection data saved successfully"
                }

            elif response.status_code == 401:
                self.logger.debug("Save inspection failed: Authentication required (401)")
                try:
                    error_response = response.json()
                    self.logger.debug("401 Error response: %s", error_response)
                except:
                    self.logger.debug("401 Error response text: %s", response.text)
                return {
                    "success": False,
                    "message": "Authentication required. Please login again."
                }
            else:
                self.logger.debug("Save inspection failed with status %s", response.status_code)
                try:
                    error_response = response.json()
                    self.logger.debug("Error response: %s", error_response)
                except:
                    self.logger.debug("Error response text: %s", response.text)
                return {
                    "success": False,
                    "message": "Failed to save inspection data: HTTP %s" % response.status_code
                }

        except requests.exceptions.ConnectionError as e:
            self.logger.debug("Connection error while saving inspection data: %s", str(e))
            return {
                "success": False,
                "message": "Could not connect to API server",
                "error": str(e)
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug("Timeout error while saving inspection data: %s", str(e))
            return {
                "success": False,
                "message": "Request timed out while saving inspection data",
                "error": str(e)
            }
        except Exception as e:
            self.logger.error("Error saving inspection data: %s", e)
            return {
                "success": False,
                "message": "Error saving inspection data: %s" % str(e)
            }

    def get_inspection_data(self, inspection_id: Optional[str] = None) -> Dict[str, Any]:
        """Retrieve inspection data via API"""
        try:
            if not self.auth_token:
                return {
                    "success": False,
                    "message": "Not authenticated. Please login first."
                }

            inspection_url = self.base_url + self.config.get('inspection_endpoint', '/inspections')
            if inspection_id:
                inspection_url += "/%s" % inspection_id

            response = self.session.get(
                inspection_url,
                timeout=self.config.get('timeout', 30)
            )

            if response.status_code == 200:
                data = response.json()
                return {
                    "success": True,
                    "data": data,
                    "message": "Inspection data retrieved successfully"
                }

            elif response.status_code == 401:
                return {
                    "success": False,
                    "message": "Authentication expired. Please login again."
                }
            elif response.status_code == 404:
                return {
                    "success": False,
                    "message": "Inspection not found: %s" % inspection_id
                }
            else:
                return {
                    "success": False,
                    "message": "Failed to retrieve inspection data: HTTP %s" % response.status_code
                }

        except requests.exceptions.ConnectionError:
            return {
                "success": False,
                "message": "Could not connect to API server"
            }
        except requests.exceptions.Timeout:
            return {
                "success": False,
                "message": "Request timed out while retrieving inspection data"
            }
        except Exception as e:
            self.logger.error("Error retrieving inspection data: %s", e)
            return {
                "success": False,
                "message": "Error retrieving inspection data: %s" % str(e)
            }

    def logout(self):
        """Clear authentication and logout"""
        try:
            if self.auth_token:
                # Remove authorization header
                if 'Authorization' in self.session.headers:
                    del self.session.headers['Authorization']

                self.auth_token = None
                self.logger.info("Successfully logged out")

        except Exception as e:
            self.logger.error("Error during logout: %s", e)

    def is_authenticated(self) -> bool:
        """Check if currently authenticated"""
        return self.auth_token is not None

    def close(self):
        """Close the API connection"""
        try:
            self.logout()
            self.session.close()
        except Exception as e:
            self.logger.error("Error closing API connection: %s", e)
