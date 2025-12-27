# test_ingestion.py
import requests
import json

def test_ingestion_api():
    """Test if we can access the ingestion API without authentication"""
    
    # Test with a fake token to see if we get a different error
    fake_token = "fake_token"
    headers = {
        'Authorization': f'Bearer {fake_token}',
        'Content-Type': 'application/json'
    }
    
    # Try to send a minimal ingestion request
    ingest_data = {
        "files": []
    }
    
    try:
        response = requests.post('http://localhost:8000/ingest', 
                                headers=headers, 
                                data=json.dumps(ingest_data))
        print(f"Ingest response: {response.status_code}")
        print(f"Response: {response.text}")
    except Exception as e:
        print(f"Error testing ingestion: {e}")

def test_public_endpoints():
    """Test public endpoints that don't require authentication"""
    endpoints = [
        'http://localhost:8000/health',
    ]
    
    for endpoint in endpoints:
        try:
            response = requests.get(endpoint)
            print(f"{endpoint}: {response.status_code}")
            print(f"Response: {response.json() if response.status_code == 200 else response.text}")
        except Exception as e:
            print(f"Error accessing {endpoint}: {e}")

if __name__ == "__main__":
    print("Testing public endpoints...")
    test_public_endpoints()
    print("\n" + "="*50)
    print("Testing protected endpoint (will fail with fake token)...")
    test_ingestion_api()