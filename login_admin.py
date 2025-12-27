# login_admin.py
import requests
import urllib.parse

def login_with_created_user():
    """Try logging in with the specific user we created directly in the DB"""
    
    username = "admin"
    password = "admin123"  # Simple password we used
    
    print(f"Trying login with {username}/{password}...")
    
    # Format data for OAuth2 login endpoint
    form_data = urllib.parse.urlencode({'username': username, 'password': password})
    headers = {'Content-Type': 'application/x-www-form-urlencoded'}
    
    try:
        response = requests.post('http://localhost:8000/token', 
                               data=form_data, 
                               headers=headers)
        
        print(f"Response status: {response.status_code}")
        print(f"Response content: {response.text}")
        
        if response.status_code == 200:
            token_data = response.json()
            token = token_data['access_token']
            print(f"SUCCESS! Got JWT token")
            print(f"Token: {token}")
            return token
        else:
            print(f"Login failed with status {response.status_code}")
            return None
    except Exception as e:
        print(f"Error during login: {e}")
        return None

if __name__ == "__main__":
    token = login_with_created_user()
    if token:
        print(f"\nYou can use this token in your requests:")
        print(f"Authorization: Bearer {token}")
    else:
        print(f"\nCould not authenticate with admin/admin123.")
        print(f"The user exists in the database but there might be a bcrypt issue during authentication.")