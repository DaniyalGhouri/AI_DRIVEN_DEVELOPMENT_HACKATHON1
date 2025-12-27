import google.generativeai as genai
from typing import List, Optional
import time

def generate_embeddings(
    texts: List[str], 
    gemini_api_key: str, 
    task_type: str = "retrieval_document"
) -> List[List[float]]:
    """
    Generates embeddings for a list of texts using Google Gemini API.
    
    Args:
        texts: List of text strings to embed.
        gemini_api_key: API key for Google Gemini.
        task_type: "retrieval_document" (for storing) or "retrieval_query" (for searching).
        
    Returns:
        List of embedding vectors.
    """
    if not gemini_api_key:
        raise ValueError("GEMINI_API_KEY is required for generating embeddings.")
    
    genai.configure(api_key=gemini_api_key)
    
    embeddings = []
    # Batch processing to avoid hitting API limits and for efficiency
    batch_size = 10 
    
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        try:
            # model="models/embedding-001" is standard 768 dim
            result = genai.embed_content(
                model="models/embedding-001",
                content=batch_texts,
                task_type=task_type,
                title=None
            )
            
            # The result['embedding'] is a list of embeddings when input is a list
            if 'embedding' in result:
                 embeddings.extend(result['embedding'])
            else:
                 # Should not happen with list input, but safe fallback
                 print(f"Unexpected response structure: {result}")
                 
        except Exception as e:
            print(f"Error generating embeddings for batch {i}: {e}")
            raise e
            
    return embeddings

if __name__ == "__main__":
    # Example usage (requires GEMINI_API_KEY in env)
    import os
    key = os.getenv("GEMINI_API_KEY")
    if key:
        sample_texts = ["Hello world", "How are you?"]
        try:
            embs = generate_embeddings(sample_texts, key, "retrieval_document")
            print(f"Generated {len(embs)} embeddings of size {len(embs[0])}")
        except Exception as e:
            print(f"Failed: {e}")
    else:
        print("Set GEMINI_API_KEY to test.")