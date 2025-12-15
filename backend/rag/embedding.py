import os
from typing import List
import google.generativeai as genai

EMBEDDING_MODEL = "models/embedding-001"

def generate_embeddings(texts: List[str], gemini_api_key: str) -> List[List[float]]:
    """
    Generates embeddings for a list of texts using Google's embedding-001 model.
    """
    if not gemini_api_key:
        raise ValueError("GEMINI_API_KEY must be provided for embedding generation.")
    
    genai.configure(api_key=gemini_api_key)

    response = genai.embed_content(
        model=EMBEDDING_MODEL,
        content=texts,
        task_type="retrieval_document"
    )
    return [d for d in response['embedding']]

if __name__ == "__main__":
    print("Embedding example cannot run directly without API key.")
