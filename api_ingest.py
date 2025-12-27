# api_ingest.py
import requests
import json
from pathlib import Path

def ingest_content_with_token(jwt_token):
    """Use the API to ingest textbook content"""
    
    headers = {
        'Authorization': f'Bearer {jwt_token}',
        'Content-Type': 'application/json'
    }
    
    # Load textbook content
    textbook_path = Path("physical-ai-humanoid-robotics-textbook/docs")
    files_to_process = list(textbook_path.rglob("*.md"))
    
    print(f"Found {len(files_to_process)} markdown files to process")
    
    # Process files in batches to avoid timeout
    batch_size = 3  # Small batch size to avoid issues
    successful_ingests = 0
    failed_ingests = 0
    
    for i in range(0, len(files_to_process), batch_size):
        batch = files_to_process[i:i + batch_size]
        
        files_data = []
        
        for md_file in batch:
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
                    
                    files_data.append({
                        "file_path": relative_path,
                        "content": content[:4000],  # Limit content size to avoid issues
                        "module_id": module_id
                    })
                    
                    print(f"Added to batch: {relative_path} (module: {module_id})")
                    
                except Exception as e:
                    print(f"Error reading {md_file}: {e}")
                    failed_ingests += 1
        
        if files_data:
            ingest_payload = {
                "files": files_data
            }
            
            print(f"\nSending batch {i//batch_size + 1} with {len(files_data)} files...")
            
            try:
                response = requests.post('http://localhost:8000/ingest', 
                                       headers=headers, 
                                       json=ingest_payload, 
                                       timeout=60)  # 60 second timeout
                
                if response.status_code == 200:
                    result = response.json()
                    print(f"Batch {i//batch_size + 1} ingested successfully: {result}")
                    successful_ingests += len(files_data)
                else:
                    print(f"Batch {i//batch_size + 1} failed: {response.status_code} - {response.text}")
                    failed_ingests += len(files_data)
                    
            except requests.exceptions.Timeout:
                print(f"Batch {i//batch_size + 1} timed out")
                failed_ingests += len(files_data)
            except Exception as e:
                print(f"Batch {i//batch_size + 1} error: {e}")
                failed_ingests += len(files_data)
    
    print(f"\nIngestion completed!")
    print(f"Successful: {successful_ingests}")
    print(f"Failed: {failed_ingests}")
    print(f"Total processed: {successful_ingests + failed_ingests}")

if __name__ == "__main__":
    # Get token from user input
    token = input("Enter your JWT token: ")
    if token.strip():
        ingest_content_with_token(token.strip())
    else:
        print("No token provided. Exiting.")