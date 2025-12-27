#!/usr/bin/env python3
# start_server.py
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
dotenv_path = os.path.join(os.path.dirname(__file__), 'backend', '.env')
if os.path.exists(dotenv_path):
    load_dotenv(dotenv_path)
    print("Environment variables loaded from .env file")
else:
    print("Warning: .env file not found")

# Add the project root to Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Now start the server with the environment properly loaded
from backend.main import app
import uvicorn

print("Starting backend server...")
print(f"QDRANT_HOST: {os.getenv('QDRANT_HOST', 'NOT SET')}")
print(f"DATABASE_URL: {os.getenv('DATABASE_URL', 'NOT SET')[:50] if os.getenv('DATABASE_URL') else 'NOT SET'}...")

uvicorn.run(
    app,
    host="0.0.0.0",
    port=8000,
    reload=False  # Disable reload for stability
)