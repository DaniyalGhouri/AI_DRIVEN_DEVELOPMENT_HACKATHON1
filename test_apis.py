# test_apis.py
import requests
import json

def test_health():
    """Test the health endpoint"""
    try:
        response = requests.get('http://localhost:8000/health')
        print("Health check response:", response.status_code)
        print(json.dumps(response.json(), indent=2))
    except Exception as e:
        print(f"Health check failed: {e}")

def test_query_api():
    """Test the query API directly"""
    headers = {
        'Content-Type': 'application/json'
    }
    query_data = {
        "query": "What is ROS 2?",
        "module_context": "module-1-ros2"
    }
    
    try:
        response = requests.post('http://localhost:8000/query', headers=headers, json=query_data)
        print(f"\nQuery API response: {response.status_code}")
        print(f"Response: {response.text}")
    except Exception as e:
        print(f"\nQuery API failed: {e}")

def test_ingestion_status():
    """Test if our ingested content is accessible"""
    try:
        response = requests.get('http://localhost:8000/users/me/notes')  # This will fail without auth but shows connectivity
        print(f"\nNotes API (should require auth): {response.status_code}")
    except Exception as e:
        print(f"\nNotes API failed: {e}")

if __name__ == "__main__":
    print("Testing backend APIs...")
    test_health()
    test_query_api()
    test_ingestion_status()