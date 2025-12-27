import os
import sys
import threading
import time

# Add project root to Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Change to project directory
os.chdir(project_root)

# Load environment variables from .env file in backend directory
backend_dir = os.path.join(project_root, "backend")
dotenv_path = os.path.join(backend_dir, ".env")

# Check if .env file exists
if os.path.exists(dotenv_path):
    print("Loading environment variables from .env file...")
    with open(dotenv_path, 'r') as f:
        env_vars = {}
        for line in f:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                key, value = line.split('=', 1)
                # Remove quotes if present
                value = value.strip().strip("'\"")
                env_vars[key] = value
                os.environ[key] = value
    print("Environment variables loaded successfully.")
else:
    print("ERROR: .env file not found in backend directory!")

print(f"Checking environment variables...")
print(f"QDRANT_HOST: {'SET' if os.getenv('QDRANT_HOST') else 'NOT SET'}")
print(f"DATABASE_URL: {'SET' if os.getenv('DATABASE_URL') else 'NOT SET'}")
print(f"GEMINI_API_KEY: {'SET' if os.getenv('GEMINI_API_KEY') else 'NOT SET'}")

# Now try to import and run the app
try:
    print("\nImporting backend application...")
    from backend.main import app
    print("Backend application imported successfully.")
    
    print("\nStarting backend server...")
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
except ImportError as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()
except Exception as e:
    print(f"Server startup error: {e}")
    import traceback
    traceback.print_exc()