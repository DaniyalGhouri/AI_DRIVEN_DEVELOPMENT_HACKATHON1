from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
import json
from typing import List, Dict, Any, Optional
import uuid # Import uuid for user_id

from fastapi import FastAPI, HTTPException, BackgroundTasks, Depends, status, Request
from fastapi.responses import StreamingResponse
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel, Field

from backend.db.postgres import insert_content_chunk, get_db_connection, create_tables, create_user, get_user_by_username, create_user_profile, get_user_notes, get_chat_sessions_by_user # Import new DB functions
from backend.rag.embedding import generate_embeddings
from backend.rag.retriever import QdrantRetriever
from backend.agent.agent import RAGAgent # Import the RAGAgent
from backend.auth.utils import verify_password, get_password_hash, create_access_token
from backend.auth.dependencies import get_current_user # Import the dependency
from dotenv import load_dotenv

load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
SECRET_KEY = os.getenv("SECRET_KEY")
ALGORITHM = os.getenv("ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

app = FastAPI()
limiter = FastAPILimiter(identifier=lambda request: request.client.host, backend=MemoryBackend())
qdrant_retriever = QdrantRetriever(qdrant_host=QDRANT_HOST, qdrant_api_key=QDRANT_API_KEY)
rag_agent = RAGAgent(gemini_api_key=GEMINI_API_KEY) # Initialize RAGAgent

# Pydantic models for request bodies
class FileContent(BaseModel):
    file_path: str
    content: str
    module_id: str
    # Optional fields for more granular metadata
    chapter_id: Optional[str] = None
    section_id: Optional[str] = None
    page_number: Optional[int] = None

class IngestRequest(BaseModel):
    files: List[FileContent]

class QueryRequest(BaseModel):
    query: str
    user_id: Optional[str] = None # Changed to str for simplicity with UUID for now
    module_context: Optional[str] = None

class HighlightedQueryRequest(BaseModel):
    query: str
    highlighted_text: str
    user_id: Optional[str] = None
    module_context: Optional[str] = None

class TranslationRequest(BaseModel):
    text: str
    target_language: str

# JWT Authentication Models
class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None

class User(BaseModel):
    id: uuid.UUID # Add ID for the user
    username: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    disabled: Optional[bool] = None

class UserInDB(User):
    hashed_password: str

class UserCreate(BaseModel):
    username: str
    password: str
    email: Optional[str] = None
    full_name: Optional[str] = None

async def get_user(username: str):
    db_user = get_user_by_username(username)
    if db_user:
        return UserInDB(**db_user)
    return None

@app.on_event("startup")
async def startup_event():
    # Ensure database tables exist on startup
    try:
        create_tables()
        print("Database tables ensured.")
    except Exception as e:
        print(f"Failed to ensure database tables: {e}")
        # Depending on criticality, you might want to exit or log more severely

@app.post("/register", response_model=User)
@limiter.limit("5/minute")
async def register(user: UserCreate):
    db_user = await get_user(user.username)
    if db_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already registered"
        )
    user_id = create_user(user.username, user.email, get_password_hash(user.password))
    # Create a user profile upon registration
    create_user_profile(user_id) 
    return User(id=user_id, username=user.username, email=user.email, full_name=user.full_name)

@app.post("/token", response_model=Token)
@limiter.limit("10/minute")
async def login_for_access_token(form_data: Annotated[OAuth2PasswordRequestForm, Depends()]):
    user = await get_user(form_data.username)
    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@app.get("/users/me/", response_model=User)
@limiter.limit("60/minute")
async def read_users_me(current_user: Annotated[User, Depends(get_current_user)]):
    return current_user

@app.get("/users/me/notes")
@limiter.limit("30/minute")
async def read_my_notes(current_user: Annotated[User, Depends(get_current_user)]):
    # In a real app, user.id would be a UUID object from the DB.
    # Here, get_current_user returns { "username": username }, so we need to fetch the full user.
    db_user = get_user_by_username(current_user.username)
    if not db_user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    
    notes = get_user_notes(user_id=uuid.UUID(db_user["id"]))
    return notes

@app.get("/users/me/chat_sessions")
@limiter.limit("30/minute")
async def read_my_chat_sessions(current_user: Annotated[User, Depends(get_current_user)]):
    db_user = get_user_by_username(current_user.username)
    if not db_user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    
    chat_sessions = get_chat_sessions_by_user(user_id=uuid.UUID(db_user["id"]))
    return chat_sessions
        
@app.post("/ingest")
@limiter.limit("1/minute")
async def ingest_content(request: IngestRequest, background_tasks: BackgroundTasks):
    # This is a simplified ingestion. In a real scenario, chunking would happen here.
    # For now, we treat each 'file' as a single chunk for embedding.
    # A more advanced chunking strategy (e.g., hierarchical) would be implemented here.

    texts_to_embed = []
    metadatas_to_store = []
    
    for file_data in request.files:
        # Generate a unique ID for this content chunk
        chunk_id = uuid.uuid4()
        
        texts_to_embed.append(file_data.content)
        metadatas_to_store.append({
            "id": str(chunk_id),
            "text": file_data.content, # Store full text in metadata as well for retrieval
            "module_id": file_data.module_id,
            "chapter_id": file_data.chapter_id,
            "section_id": file_data.section_id,
            "page_number": file_data.page_number,
            "file_path": file_data.file_path,
            "version": "1.0", # Placeholder version
            "updated_at": datetime.now(timezone.utc).isoformat(),
        })

        # Store metadata in Neon Postgres
        background_tasks.add_task(
            insert_content_chunk,
            id=chunk_id,
            text=file_data.content,
            module_id=file_data.module_id,
            chapter_id=file_data.chapter_id,
            section_id=file_data.section_id,
            page_number=file_data.page_number,
            start_index=0, # Simplified
            end_index=len(file_data.content), # Simplified
            file_path=file_data.file_path,
            version="1.0",
        )
    
    # Generate embeddings and upsert to Qdrant in a background task
    # This assumes chunking has already happened or each file_data.content is a chunk
    async def _embed_and_upsert():
        try:
            embeddings = generate_embeddings(texts_to_embed, gemini_api_key=GEMINI_API_KEY)
            # Add embeddings to metadata for Qdrant payload if needed, or keep separate
            # QdrantRetriever expects metadatas as a list of dicts, and handles point IDs internally
            qdrant_retriever.upsert_vectors(embeddings, metadatas_to_store)
            print(f"Successfully embedded and upserted {len(embeddings)} chunks to Qdrant.")
        except Exception as e:
            print(f"Error during embedding or Qdrant upsert: {e}")

    background_tasks.add_task(_embed_and_upsert)

    return {"status": "success", "message": f"{len(request.files)} files submitted for ingestion."}


@app.post("/query")
@limiter.limit("20/minute")
async def query_rag(request: QueryRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        # Fetch the full user object to get the UUID id
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        request.user_id = str(db_user["id"]) # Assign the authenticated user's ID
        
        # 1. Generate embedding for the query
        query_embedding = generate_embeddings([request.query], gemini_api_key=GEMINI_API_KEY)[0]

        # 2. Retrieve relevant chunks from Qdrant
        # Use module_context for filtering if provided
        retrieved_chunks = qdrant_retriever.search_vectors(
            query_embedding=query_embedding,
            limit=7, # Top-k chunks (5-8 as per spec)
            module_id=request.module_context
        )
        
        # 3. Re-rank with LLM (simplified for now, actual re-ranking would be more complex)
        # For this implementation, we'll just use the retrieved order.
        # A proper re-ranking step would involve another LLM call or a re-ranking model.
        
        # 4. Generate response using RAGAgent
        response_data = await rag_agent.generate_response(request.query, retrieved_chunks)
        
        # 5. Handle streaming (simplified, as agent.generate_response currently returns full text)
        # For actual streaming, rag_agent.generate_response would yield chunks.
        return StreamingResponse(
            iter([json.dumps(response_data)]), # Yield a single JSON string for now
            media_type="application/json"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {e}")

@app.post("/highlighted-query")
@limiter.limit("20/minute")
async def highlighted_query_rag(request: HighlightedQueryRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        # Fetch the full user object to get the UUID id
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        request.user_id = str(db_user["id"]) # Assign the authenticated user's ID
        
        # Context from highlighted text is primary
        highlight_context = [{
            "text": request.highlighted_text,
            "file_path": "user_highlight", # Indicate source
            "module_id": request.module_context or "unknown",
        }]

        # Generate embedding for the query to retrieve additional context
        query_embedding = generate_embeddings([request.query], gemini_api_key=GEMINI_API_KEY)[0]

        # Retrieve relevant chunks from Qdrant, excluding anything already in highlight_context
        # For simplicity, we assume no overlap, or agent handles redundancy
        retrieved_chunks = qdrant_retriever.search_vectors(
            query_embedding=query_embedding,
            limit=5, # Fewer chunks as highlighted text is primary
            module_id=request.module_context
        )
        
        # Combine highlighted text and retrieved chunks
        # Prioritize highlighted text by placing it first
        combined_context = highlight_context + retrieved_chunks
        
        # Generate response using RAGAgent
        response_data = await rag_agent.generate_response(request.query, combined_context)
        
        return StreamingResponse(
            iter([json.dumps(response_data)]),
            media_type="application/json"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing highlighted query: {e}")

@app.post("/translate")
@limiter.limit("10/minute")
async def translate_text(request: TranslationRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        # Construct a prompt for the Gemini model to perform translation
        translation_prompt = (
            f"Translate the following text to {request.target_language}. "
            f"Only return the translated text and nothing else:\n\n"
            f"{request.text}"
        )
        
        # Use the RAGAgent to generate the translation.
        # Note: For pure translation, context might not be needed or would be empty.
        # Re-using RAGAgent for simplicity here.
        translation_response = await rag_agent.generate_response(
            query=translation_prompt,
            context=[] # No retrieval context needed for pure translation
        )
        
        # The generate_response method returns a dict with 'answer'.
        translated_text = translation_response.get("answer", "Translation failed.")
        
        return {"translated_text": translated_text}
    except Exception as e:
        print(f"Error during translation: {e}")
        raise HTTPException(status_code=500, detail=f"Error during translation: {e}")

@app.get("/users/me/recommendations")
@limiter.limit("5/minute")
async def get_my_recommendations(current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        
        user_id_uuid = uuid.UUID(db_user["id"])
        
        # Retrieve user's notes and chat sessions
        user_notes = get_user_notes(user_id=user_id_uuid)
        user_chat_sessions = get_chat_sessions_by_user(user_id=user_id_uuid)
        
        # Combine into a context for recommendations
        context_for_recommendations = []
        if user_notes:
            context_for_recommendations.append({"text": "User's notes: " + " ".join([note["note_content"] for note in user_notes]), "source": "notes"})
        
        if user_chat_sessions:
            chat_history_text = []
            for session in user_chat_sessions:
                for message in session.get("chat_history", []):
                    chat_history_text.append(message.get("text", ""))
            if chat_history_text:
                context_for_recommendations.append({"text": "User's chat history: " + " ".join(chat_history_text), "source": "chat_history"})

        if not context_for_recommendations:
            return {"recommendations": "No sufficient data to generate recommendations yet. Start interacting with the chatbot or add some notes!"}

        # Craft a prompt for the RAG agent to generate recommendations
        recommendation_query = (
            "Based on the following user interactions (notes and chat history), "
            "provide personalized learning recommendations related to Physical AI and Humanoid Robotics. "
            "Suggest relevant topics, modules, or external resources. Be concise and helpful."
        )

        response_data = await rag_agent.generate_response(
            query=recommendation_query,
            context=context_for_recommendations # Pass the combined context
        )
        
        return {"recommendations": response_data.get("answer", "Could not generate recommendations.")}

    except Exception as e:
        print(f"Error generating recommendations: {e}")
        raise HTTPException(status_code=500, detail=f"Error generating recommendations: {e}")

@app.get("/health")
async def health_check():
    status_qdrant = {"status": "ok", "message": ""}
    status_neon = {"status": "ok", "message": ""}

    # Check Qdrant status
    try:
        # Attempt to list collections to verify Qdrant connectivity
        qdrant_retriever.client.get_collections()
        status_qdrant["status"] = "ok"
    except Exception as e:
        status_qdrant["status"] = "error"
        status_qdrant["message"] = str(e)

    # Check Neon Postgres status
    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute("SELECT 1")
        cur.fetchone()
        status_neon["status"] = "ok"
    except Exception as e:
        status_neon["status"] = "error"
        status_neon["message"] = str(e)
    finally:
        if conn:
            cur.close()
            conn.close()

    overall_status = "ok" if status_qdrant["status"] == "ok" and status_neon["status"] == "ok" else "error"

    return {
        "overall_status": overall_status,
        "services": {
            "qdrant": status_qdrant,
            "neon_postgres": status_neon,
        }
    }

# Main application entry point for development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)