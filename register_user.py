# register_user.py
import requests

def register_and_login():
    """Register a user and get JWT token"""

    # Try to register a new user with unique credentials
    register_data = {
        'username': 'admin_user',
        'password': 'secure_password123',
        'email': 'admin@example.com'
    }

    print("Registering user...")
    register_response = requests.post('http://localhost:8000/register', json=register_data)  # Changed to json= instead of data=

    login_username = 'admin_user'
    login_password = 'secure_password123'

    if register_response.status_code != 200:
        print(f"Registration failed or user already exists: {register_response.text}")
        # If registration failed, maybe user already exists, try to login with testuser
        # First try with the attempted new user in case it was created despite the error
        login_username = 'admin_user'
        login_password = 'secure_password123'
    else:
        print("User registered successfully")

    # Try to login with determined credentials
    import urllib.parse
    login_data = {
        'username': login_username,
        'password': login_password
    }

    form_data = urllib.parse.urlencode(login_data)
    login_response = requests.post('http://localhost:8000/token',
                                  data=form_data,
                                  headers={'Content-Type': 'application/x-www-form-urlencoded'})

    # If login with admin_user fails, try common defaults
    if login_response.status_code != 200:
        default_users = [
            ('admin_user', 'secure_password123'),  # Try the attempted new user again
            ('testuser', 'testpass'),              # Try the original test user
            ('admin', 'admin'),
            ('user', 'password'),
            ('test', 'test')
        ]

        for username, password in default_users:
            form_data = urllib.parse.urlencode({'username': username, 'password': password})
            temp_response = requests.post('http://localhost:8000/token',
                                          data=form_data,
                                          headers={'Content-Type': 'application/x-www-form-urlencoded'})
            if temp_response.status_code == 200:
                login_response = temp_response
                print(f"Login successful with {username}/{password}")
                break

    if login_response.status_code == 200:
        token_data = login_response.json()
        token = token_data['access_token']
        print(f"Successfully obtained JWT token: {token[:20]}...")  # Show partial token
        return token
    else:
        print(f"Login failed: {login_response.text}")
        return None

def try_login(username, password):
    """Helper function to try login"""
    import urllib.parse
    form_data = urllib.parse.urlencode({'username': username, 'password': password})
    return requests.post('http://localhost:8000/token',
                         data=form_data,
                         headers={'Content-Type': 'application/x-www-form-urlencoded'})

if __name__ == "__main__":
    token = register_and_login()
    if token:
        print("\nYou can now use this token for API requests")
        print("Token:", token)
    else:
        print("\nFailed to get token - make sure your backend server is running on http://localhost:8000")
        print("Possible issues: database not accessible, user already exists with different password, or server errors")