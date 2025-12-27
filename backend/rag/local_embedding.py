# backend/rag/local_embedding.py
from typing import List
import numpy as np

# Import the sentence transformer
from sentence_transformers import SentenceTransformer

class LocalEmbeddingGenerator:
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize the local embedding generator with a sentence transformer model.
        
        Args:
            model_name: Name of the sentence transformer model to use
                       Common options: "all-MiniLM-L6-v2", "all-mpnet-base-v2"
        """
        print(f"Loading sentence transformer model: {model_name}")
        self.model = SentenceTransformer(model_name)
        print("Model loaded successfully!")
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using the local model.
        
        Args:
            texts: List of text strings to embed
            
        Returns:
            List of embedding vectors (each as a list of floats)
        """
        if not texts:
            return []
        
        # Generate embeddings using the local model
        embeddings = self.model.encode(texts)
        
        # Convert to list of lists (standard format)
        embeddings_list = embeddings.tolist()
        
        return embeddings_list

# Backward compatibility function to match the existing interface
def generate_embeddings(texts: List[str], gemini_api_key: str = None) -> List[List[float]]:
    """
    Generate embeddings using local sentence transformer model.
    Maintains compatibility with existing interface.
    
    Args:
        texts: List of text strings to embed
        gemini_api_key: Not used (kept for compatibility)
        
    Returns:
        List of embedding vectors
    """
    generator = LocalEmbeddingGenerator()
    return generator.generate_embeddings(texts)

if __name__ == "__main__":
    # Example usage
    sample_texts = [
        "Introduction to ROS 2",
        "Robot Operating System concepts",
        "Humanoid robotics control systems"
    ]
    
    print("Generating sample embeddings...")
    embeddings = generate_embeddings(sample_texts)
    
    print(f"Generated {len(embeddings)} embeddings")
    print(f"Each embedding has dimension: {len(embeddings[0])}")
    print("Sample embedding (first 10 dimensions):", embeddings[0][:10])