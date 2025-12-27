# backend/rag/simple_embedding.py
from typing import List
import hashlib
import struct
import numpy as np

def simple_text_to_vector(text: str, vector_size: int = 768) -> List[float]:
    """
    A simple deterministic text-to-vector converter using hashing.
    This creates consistent embeddings based on the text content.
    """
    # Create a hash of the text
    text_hash = hashlib.sha256(text.encode('utf-8')).digest()
    
    # Convert the hash to a list of floats
    vector = []
    for i in range(0, len(text_hash), 4):
        # Take 4 bytes at a time and convert to a float
        chunk = text_hash[i:i+4]
        if len(chunk) < 4:
            # Pad with zeros if necessary
            chunk += b'\x00' * (4 - len(chunk))
        
        # Convert 4-byte chunk to integer then normalize to [-1, 1]
        int_val = struct.unpack('<I', chunk)[0]
        float_val = (int_val % 100000) / 50000.0 - 1.0  # Normalize to [-1, 1]
        vector.append(float_val)
    
    # If we don't have enough values, pad with zeros
    while len(vector) < vector_size:
        vector.append(0.0)
    
    # Truncate to the desired size
    return vector[:vector_size]

def generate_embeddings(texts: List[str], gemini_api_key: str = None) -> List[List[float]]:
    """
    Simple embedding generation using hash-based vectors.
    Much faster and doesn't require internet or external services.
    """
    embeddings = []
    for text in texts:
        embedding = simple_text_to_vector(text)
        embeddings.append(embedding)
    
    return embeddings

if __name__ == "__main__":
    # Example usage
    sample_texts = [
        "Introduction to ROS 2",
        "Robot Operating System concepts",
        "Humanoid robotics control systems"
    ]
    
    print("Generating sample embeddings using simple method...")
    embeddings = generate_embeddings(sample_texts)
    
    print(f"Generated {len(embeddings)} embeddings")
    print(f"Each embedding has dimension: {len(embeddings[0])}")
    print("Sample embedding (first 10 dimensions):", embeddings[0][:10])