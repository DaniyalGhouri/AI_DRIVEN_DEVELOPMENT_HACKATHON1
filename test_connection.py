# test_connection.py
import requests

def test_connection():
    """Test if the backend server is responding"""
    try:
        response = requests.get('http://localhost:8000/health')
        print("Health check response:", response.status_code, response.json() if response.status_code == 200 else response.text)
    except requests.exceptions.ConnectionError:
        print("Cannot connect to server - make sure backend is running on http://localhost:8000")
    except Exception as e:
        print(f"Error during health check: {e}")

if __name__ == "__main__":
    test_connection()