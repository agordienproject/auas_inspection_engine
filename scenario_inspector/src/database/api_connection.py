"""
API Connection module for AUAS Inspection Engine
Replaces direct database access with secure API calls
"""
import requests
import logging
from typing import Dict, Any, Optional
from datetime import datetime


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
            self.logger.debug(f"Testing API connection to: {self.base_url}")
            self.logger.debug(f"Connection timeout: {timeout}s")
            self.logger.debug(f"Session headers: {dict(self.session.headers)}")
            
            # Try to reach the API base endpoint
            response = self.session.get(
                self.base_url, 
                timeout=timeout
            )
            
            self.logger.debug(f"API response status: {response.status_code}")
            self.logger.debug(f"API response headers: {dict(response.headers)}")
            self.logger.debug(f"API response time: {response.elapsed.total_seconds():.3f}s")
            
            if response.status_code == 200:
                self.logger.debug(f"API connection successful to {self.base_url}")
                return {
                    "status": "available",
                    "message": "API connection successful",
                    "details": {
                        "url": self.base_url,
                        "response_time_ms": int(response.elapsed.total_seconds() * 1000)
                    }
                }
            else:
                self.logger.debug(f"API connection failed with status {response.status_code}")
                self.logger.debug(f"Response content: {response.text[:200]}...")
                return {
                    "status": "error",
                    "message": f"API returned status code: {response.status_code}",
                    "details": {"url": self.base_url, "status_code": response.status_code}
                }
                
        except requests.exceptions.ConnectionError as e:
            self.logger.debug(f"Connection error: {str(e)}")
            return {
                "status": "not_available",
                "message": "Could not connect to API server",
                "details": {"url": self.base_url, "error": str(e)}
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug(f"Timeout error: {str(e)}")
            return {
                "status": "error",
                "message": "API connection timeout",
                "details": {"url": self.base_url, "timeout": self.config.get('timeout', 30), "error": str(e)}
            }
        except Exception as e:
            self.logger.debug(f"Unexpected error during API connection test: {str(e)}")
            return {
                "status": "error",
                "message": f"API connection error: {str(e)}",
                "details": {"url": self.base_url, "error": str(e)}
            }
    
    def authenticate(self, email: str, password: str) -> Dict[str, Any]:
        """Authenticate with the API using email and password"""
        try:
            login_url = self.base_url + self.config.get('login_endpoint', '/auth/login')
            timeout = self.config.get('timeout', 30)
            
            self.logger.debug(f"Attempting authentication for email: {email}")
            self.logger.debug(f"Login URL: {login_url}")
            self.logger.debug(f"Request timeout: {timeout}s")
            
            login_data = {
                "email": email,
                "password": "***HIDDEN***"  # Don't log actual password
            }
            self.logger.debug(f"Login payload structure: {login_data}")
            
            # Prepare actual login data with real password
            actual_login_data = {
                "email": email,
                "password": password
            }
            
            self.logger.debug(f"Sending POST request to {login_url}")
            response = self.session.post(
                login_url,
                json=actual_login_data,
                timeout=timeout
            )
            
            self.logger.debug(f"Authentication response status: {response.status_code}")
            self.logger.debug(f"Authentication response headers: {dict(response.headers)}")
            self.logger.debug(f"Authentication response time: {response.elapsed.total_seconds():.3f}s")
            
            if response.status_code == 200:
                auth_data = response.json()
                self.logger.debug(f"Authentication successful, response keys: {list(auth_data.keys())}")
                
                # Store authentication token
                self.auth_token = auth_data.get('token')
                if self.auth_token:
                    self.logger.debug(f"Received auth token (length: {len(self.auth_token)})")
                    # Add token to session headers for future requests
                    self.session.headers.update({
                        'Authorization': f'Bearer {self.auth_token}'
                    })
                    self.logger.debug("Updated session headers with auth token")
                else:
                    self.logger.debug("No auth token received in response")
                
                user_data = auth_data.get('user', {})
                self.logger.debug(f"User data received: {list(user_data.keys()) if user_data else 'None'}")
                
                self.logger.info(f"Successfully authenticated user: {email}")
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
                    self.logger.debug(f"Error response content: {error_response}")
                except:
                    self.logger.debug(f"Error response text: {response.text}")
                return {
                    "success": False,
                    "message": "Invalid email or password"
                }
            else:
                self.logger.debug(f"Authentication failed with status {response.status_code}")
                try:
                    error_response = response.json()
                    self.logger.debug(f"Error response content: {error_response}")
                except:
                    self.logger.debug(f"Error response text: {response.text}")
                return {
                    "success": False,
                    "message": f"Authentication failed: HTTP {response.status_code}"
                }
                
        except requests.exceptions.ConnectionError as e:
            self.logger.debug(f"Connection error during authentication: {str(e)}")
            return {
                "success": False,
                "message": "Could not connect to authentication server",
                "error": str(e)
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug(f"Timeout error during authentication: {str(e)}")
            return {
                "success": False,
                "message": "Authentication request timed out",
                "error": str(e)
            }
        except Exception as e:
            self.logger.error(f"Authentication error: {e}")
            self.logger.debug(f"Full authentication error details: {str(e)}")
            return {
                "success": False,
                "message": f"Authentication error: {str(e)}"
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
            self.logger.debug(f"Saving inspection data to: {inspection_url}")
            self.logger.debug(f"Data keys: {list(inspection_data.keys())}")
            self.logger.debug(f"Auth token available: {bool(self.auth_token)}")
            
            # Add timestamp if not present
            if 'timestamp' not in inspection_data:
                inspection_data['timestamp'] = datetime.now().isoformat()
                self.logger.debug(f"Added timestamp to inspection data: {inspection_data['timestamp']}")
            
            self.logger.debug(f"Current session headers: {dict(self.session.headers)}")
            self.logger.debug(f"Sending POST request to save inspection data")
            
            response = self.session.post(
                inspection_url,
                json=inspection_data,
                timeout=self.config.get('timeout', 30)
            )
            
            self.logger.debug(f"Save inspection response status: {response.status_code}")
            self.logger.debug(f"Save inspection response time: {response.elapsed.total_seconds():.3f}s")
            
            if response.status_code in [200, 201]:
                result_data = response.json()
                self.logger.debug(f"Response data keys: {list(result_data.keys())}")
                self.logger.info(f"Inspection data saved successfully: {result_data.get('id', 'N/A')}")
                return {
                    "success": True,
                    "inspection_id": result_data.get('id'),
                    "message": "Inspection data saved successfully"
                }
            
            elif response.status_code == 401:
                self.logger.debug("Save inspection failed: Authentication required (401)")
                try:
                    error_response = response.json()
                    self.logger.debug(f"401 Error response: {error_response}")
                except:
                    self.logger.debug(f"401 Error response text: {response.text}")
                return {
                    "success": False,
                    "message": "Authentication required. Please login again."
                }
            else:
                self.logger.debug(f"Save inspection failed with status {response.status_code}")
                try:
                    error_response = response.json()
                    self.logger.debug(f"Error response: {error_response}")
                except:
                    self.logger.debug(f"Error response text: {response.text}")
                return {
                    "success": False,
                    "message": f"Failed to save inspection data: HTTP {response.status_code}"
                }
                
        except requests.exceptions.ConnectionError as e:
            self.logger.debug(f"Connection error while saving inspection data: {str(e)}")
            return {
                "success": False,
                "message": "Could not connect to API server",
                "error": str(e)
            }
        except requests.exceptions.Timeout as e:
            self.logger.debug(f"Timeout error while saving inspection data: {str(e)}")
            return {
                "success": False,
                "message": "Request timed out while saving inspection data",
                "error": str(e)
            }
        except Exception as e:
            self.logger.error(f"Error saving inspection data: {e}")
            return {
                "success": False,
                "message": f"Error saving inspection data: {str(e)}"
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
                inspection_url += f"/{inspection_id}"
            
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
                    "message": f"Inspection not found: {inspection_id}"
                }
            else:
                return {
                    "success": False,
                    "message": f"Failed to retrieve inspection data: HTTP {response.status_code}"
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
            self.logger.error(f"Error retrieving inspection data: {e}")
            return {
                "success": False,
                "message": f"Error retrieving inspection data: {str(e)}"
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
            self.logger.error(f"Error during logout: {e}")
    
    def is_authenticated(self) -> bool:
        """Check if currently authenticated"""
        return self.auth_token is not None
    
    def close(self):
        """Close the API connection"""
        try:
            self.logout()
            self.session.close()
        except Exception as e:
            self.logger.error(f"Error closing API connection: {e}")
