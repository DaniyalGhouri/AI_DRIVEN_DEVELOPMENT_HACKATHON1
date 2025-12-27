# ingest_textbook.py
import os
import requests
import json
from pathlib import Path
import time

def get_jwt_token():
    """Get JWT token by logging in"""
    login_data = {
        'username': 'testuser',
        'password': 'testpass'
    }
    response = requests.post('http://localhost:8000/token', data=login_data)
    if response.status_code == 200:
        token_data = response.json()
        return token_data['access_token']
    else:
        print(f"Login failed: {response.text}")
        return None

def ingest_textbook_content():
    """Ingest all textbook content into the RAG system"""
    access_token = get_jwt_token()
    if not access_token:
        print("Failed to get JWT token")
        return

    headers = {
        'Authorization': f'Bearer {access_token}',
        'Content-Type': 'application/json'
    }

    # Path to your textbook content
    textbook_path = Path("physical-ai-humanoid-robotics-textbook/docs")
    
    files_to_ingest = []
    
    # Find all markdown files in the textbook
    for md_file in textbook_path.rglob("*.md"):
        if 'node_modules' not in str(md_file) and '.git' not in str(md_file):
            try:
                content = md_file.read_text(encoding='utf-8')
                
                # Extract module information from the file path
                relative_path = str(md_file.relative_to(textbook_path))
                path_parts = relative_path.split('/')
                
                # Extract module_id from the path
                module_id = "unknown"
                if 'modules' in path_parts:
                    module_idx = path_parts.index('modules')
                    if module_idx + 1 < len(path_parts):
                        module_id = path_parts[module_idx + 1]
                
                file_info = {
                    "file_path": relative_path,
                    "content": content,
                    "module_id": module_id
                }
                
                # Add section/chapter IDs if they exist in the path
                if len(path_parts) >= 2:
                    file_info["chapter_id"] = path_parts[-2] if path_parts[-1] != 'index.md' else path_parts[-1]
                
                files_to_ingest.append(file_info)
                
                print(f"Prepared {relative_path} for ingestion")
                
            except Exception as e:
                print(f"Error reading {md_file}: {e}")
    
    print(f"Found {len(files_to_ingest)} files to ingest")
    
    if not files_to_ingest:
        print("No files found to ingest")
        return
    
    # Send the ingestion request (in batches to avoid timeout issues)
    batch_size = 5  # Process files in small batches
    for i in range(0, len(files_to_ingest), batch_size):
        batch = files_to_ingest[i:i + batch_size]
        
        ingest_payload = {
            "files": batch
        }
        
        print(f"Ingesting batch {i//batch_size + 1}/{(len(files_to_ingest)-1)//batch_size + 1}")
        
        try:
            response = requests.post('http://localhost:8000/ingest', headers=headers, json=ingest_payload)
            
            if response.status_code == 200:
                result = response.json()
                print(f"Batch {i//batch_size + 1} ingested successfully: {result}")
            else:
                print(f"Batch {i//batch_size + 1} failed: {response.status_code} - {response.text}")
                
        except Exception as e:
            print(f"Error ingesting batch: {e}")
        
        # Add a small delay between batches to prevent overwhelming the server
        time.sleep(2)

if __name__ == "__main__":
    print("Starting textbook content ingestion...")
    print("Make sure your backend server is running on http://localhost:8000")
    ingest_textbook_content()
    print("Ingestion process completed!")