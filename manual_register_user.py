# manual_register_user.py
import requests
import urllib.parse

def try_different_users():
    """Try different login combinations to access the system"""
    
    # List of potential users to try
    user_attempts = [
        {'username': 'admin', 'password': 'admin'},
        {'username': 'user', 'password': 'password'},
        {'username': 'test', 'password': 'test'},
        {'username': 'admin_user', 'password': 'secure_password123'},
        {'username': 'testuser', 'password': 'testpass'}
    ]

    for user_data in user_attempts:
        username = user_data['username']
        password = user_data['password']
        
        print(f"Trying login with {username}/{password}...")
        
        # Format data for OAuth2 login endpoint
        form_data = urllib.parse.urlencode({'username': username, 'password': password})
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        try:
            response = requests.post('http://localhost:8000/token', 
                                   data=form_data, 
                                   headers=headers)
            
            if response.status_code == 200:
                token_data = response.json()
                token = token_data['access_token']
                print(f"SUCCESS! Got JWT token with {username}/{password}")
                print(f"Token: {token}")
                return token
            else:
                print(f"  -> Failed: {response.text}")
        except Exception as e:
            print(f"  -> Error: {e}")
    
    print("\nNone of the default users worked. You may need to:")
    print("1. Check your database to see what users exist")
    print("2. Manually create a user in the database")
    print("3. Try registering with different username/password that might not conflict")
    return None

if __name__ == "__main__":
    token = try_different_users()
    if token:
        print(f"\nYou can use this token in your requests:")
        print(f"Authorization: Bearer {token}")
    else:
        print(f"\nCould not authenticate. You may need to debug the database/user creation issue.")