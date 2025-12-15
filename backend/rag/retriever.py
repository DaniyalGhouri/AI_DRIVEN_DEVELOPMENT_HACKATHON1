import os
import uuid
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue

COLLECTION_NAME = "textbook_chunks"

class QdrantRetriever:
    def __init__(self, qdrant_host: str, qdrant_api_key: str):
        if not qdrant_host or not qdrant_api_key:
            raise ValueError("QDRANT_HOST and QDRANT_API_KEY must be provided to QdrantRetriever.")
        
        self.client = QdrantClient(
            url=qdrant_host,
            api_key=qdrant_api_key,
            check_compatibility=False,
        )
        self.vector_size = 768 # Gemini embedding-001 dimension
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        collections = self.client.get_collections().collections
        if COLLECTION_NAME not in [c.name for c in collections]:
            self.client.recreate_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
            )
            print(f"Collection '{COLLECTION_NAME}' created.")
        else:
            print(f"Collection '{COLLECTION_NAME}' already exists.")

    def upsert_vectors(self, embeddings: List[List[float]], metadatas: List[Dict[str, Any]]):
        points = []
        for i, embedding in enumerate(embeddings):
            # Ensure each point has a unique ID, or use Qdrant to generate one
            point_id = str(uuid.uuid4())
            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=metadatas[i]
                )
            )
        
        self.client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=points
        )
        print(f"Upserted {len(points)} vectors to collection '{COLLECTION_NAME}'.")

    def search_vectors(
        self,
        query_embedding: List[float],
        limit: int = 5,
        module_id: Optional[str] = None,
        min_score: float = 0.7,
    ) -> List[Dict[str, Any]]:
        
        query_filter = None
        if module_id:
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="module_id",
                        match=MatchValue(value=module_id),
                    )
                ]
            )

        search_result = self.client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            query_filter=query_filter,
            limit=limit,
            score_threshold=min_score,
            with_payload=True # Retrieve payload (metadata) with results
        )
        
        results = []
        for hit in search_result:
            results.append({
                "score": hit.score,
                "text": hit.payload.get("text"),
                "module_id": hit.payload.get("module_id"),
                "chapter_id": hit.payload.get("chapter_id"),
                "section_id": hit.payload.get("section_id"),
                "page_number": hit.payload.get("page_number"),
                "file_path": hit.payload.get("file_path"),
            })
        return results

if __name__ == "__main__":
    # Example usage:
    # This block would typically be run from a main entry point that provides the keys
    print("QdrantRetriever example cannot run directly without API keys.")

