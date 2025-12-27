# backend/run_server.py
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'backend', '.env'))

# Add the project root to the Python path so imports work
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_server():
    """Run the backend server with proper environment loading"""
    from backend.main import app
    import uvicorn
    
    # Check if required environment variables are set after loading .env
    qdrant_host = os.getenv("QDRANT_HOST")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    database_url = os.getenv("DATABASE_URL")
    
    if not qdrant_host or not qdrant_api_key:
        print("WARNING: QDRANT configuration not found in environment.")
    if not database_url:
        print("WARNING: DATABASE_URL not found in environment.")
    
    # Start the server
    uvicorn.run(
        "backend.main:app",
        host="0.0.0.0",
        port=8000,
        reload=False
    )

if __name__ == "__main__":
    run_server()