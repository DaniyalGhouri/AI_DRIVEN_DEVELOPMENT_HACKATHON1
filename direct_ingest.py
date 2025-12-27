# direct_ingest.py
import sys
import os
import time  # Add time import for rate limiting
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

from backend.rag.embedding import generate_embeddings
from backend.rag.retriever import QdrantRetriever
from backend.agent.agent import RAGAgent
import uuid
from pathlib import Path
from dotenv import load_dotenv

def direct_ingest_content():
    """Directly ingest content bypassing the API and authentication"""
    print("Starting direct content ingestion...")

    # Load environment variables
    load_dotenv(os.path.join(os.path.dirname(__file__), 'backend', '.env'))
    
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    QDRANT_HOST = os.getenv("QDRANT_HOST")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    
    if not GEMINI_API_KEY:
        print("Error: GEMINI_API_KEY not found in environment")
        return
    
    if not QDRANT_HOST or not QDRANT_API_KEY:
        print("Error: QDRANT configuration not found in environment")
        return
    
    print("Connecting to Qdrant...")
    try:
        qdrant_retriever = QdrantRetriever(qdrant_host=QDRANT_HOST, qdrant_api_key=QDRANT_API_KEY)
        print("Connected to Qdrant successfully!")
    except Exception as e:
        print(f"Error connecting to Qdrant: {e}")
        return
    
    print("Initializing RAG Agent...")
    try:
        rag_agent = RAGAgent(gemini_api_key=GEMINI_API_KEY)
        print("RAG Agent initialized successfully!")
    except Exception as e:
        print(f"Error initializing RAG Agent: {e}")
        return
    
    # Load textbook content
    textbook_path = Path("physical-ai-humanoid-robotics-textbook/docs")
    files_to_process = list(textbook_path.rglob("*.md"))
    
    print(f"Found {len(files_to_process)} markdown files to process")
    
    successful_ingests = 0
    failed_ingests = 0
    
    for md_file in files_to_process:
        if 'node_modules' not in str(md_file) and '.git' not in str(md_file):
            try:
                print(f"Processing: {md_file}")
                
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
                
                # For direct ingestion, we'll break content into chunks
                # Simple chunking: split by paragraphs or max 1000 chars
                chunks = []
                if len(content) > 1000:
                    # Simple chunking - split approximately every 1000 characters
                    for i in range(0, len(content), 1000):
                        chunk = content[i:i+1000]
                        chunks.append({
                            "text": chunk,
                            "module_id": module_id,
                            "file_path": relative_path,
                            "chunk_index": len(chunks)
                        })
                else:
                    chunks.append({
                        "text": content,
                        "module_id": module_id,
                        "file_path": relative_path,
                        "chunk_index": 0
                    })
                
                # Process each chunk
                for chunk_idx, chunk_data in enumerate(chunks):
                    try:
                        # Add delay to respect rate limits (Gemini free tier: 15 requests/minute)
                        if chunk_idx > 0:
                            print(f"  Waiting for rate limit (4 seconds)...")
                            time.sleep(4)

                        # Generate embedding for this chunk
                        embedding = generate_embeddings([chunk_data["text"]], GEMINI_API_KEY, task_type="retrieval_document")[0]

                        # Prepare metadata for Qdrant
                        metadata = {
                            "id": str(uuid.uuid4()),
                            "text": chunk_data["text"],
                            "module_id": chunk_data["module_id"],
                            "file_path": chunk_data["file_path"],
                            "chunk_index": chunk_data["chunk_index"],
                            "version": "1.0",
                        }

                        # Insert into Qdrant
                        qdrant_retriever.upsert_vectors([embedding], [metadata])

                        print(f"  Successfully ingested chunk {chunk_data['chunk_index']} from {relative_path}")
                        successful_ingests += 1
                    except Exception as e:
                        error_msg = str(e)
                        if "429" in error_msg or "quota" in error_msg.lower() or "exceeded" in error_msg.lower():
                            print(f"  Rate limit hit: {e}")
                            print("  Waiting 60 seconds before continuing...")
                            time.sleep(60)
                            # Note: We don't retry here, just continue to next chunk
                        else:
                            print(f"  Error ingesting chunk: {e}")
                            failed_ingests += 1
                        
            except Exception as e:
                print(f"Error processing file {md_file}: {e}")
                failed_ingests += 1
    
    print(f"\nIngestion completed!")
    print(f"Successful: {successful_ingests}")
    print(f"Failed: {failed_ingests}")
    print(f"Total processed: {successful_ingests + failed_ingests}")

if __name__ == "__main__":
    direct_ingest_content()