import os
import sys
import subprocess
import threading
import time

def run_backend():
    """Function to run the backend server"""
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
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    # Remove quotes if present
                    value = value.strip().strip("'\"")
                    os.environ[key] = value
        print("Environment variables loaded successfully.")
    else:
        print("ERROR: .env file not found in backend directory!")
    
    print(f"QDRANT_HOST: {'SET' if os.getenv('QDRANT_HOST') else 'NOT SET'}")
    print(f"DATABASE_URL: {'SET' if os.getenv('DATABASE_URL') else 'NOT SET'}")
    print(f"GEMINI_API_KEY: {'SET' if os.getenv('GEMINI_API_KEY') else 'NOT SET'}")
    
    # Set PYTHONPATH to include current directory
    env = os.environ.copy()
    env['PYTHONPATH'] = f"{project_root};{env.get('PYTHONPATH', '')}"

    # Try to run the backend server
    try:
        print("\nStarting backend server with uvicorn...")
        subprocess.run([
            sys.executable, "-c",
            "from backend.main import app; import uvicorn; uvicorn.run(app, host='0.0.0.0', port=8000)"
        ], env=env)
    except Exception as e:
        print(f"Error running backend: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Run the backend in a separate thread/process
    backend_thread = threading.Thread(target=run_backend)
    backend_thread.daemon = True
    backend_thread.start()
    
    print("Backend server started in background. Press Ctrl+C to stop.")
    
    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        sys.exit(0)