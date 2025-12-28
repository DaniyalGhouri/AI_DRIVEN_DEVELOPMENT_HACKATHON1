from datetime import datetime, timedelta, timezone
from fastapi import FastAPI, HTTPException, BackgroundTasks, Depends, status, Request
from fastapi.responses import StreamingResponse
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional, Annotated
import json
import uuid
import os

from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from backend.db.postgres import insert_content_chunk, get_db_connection, create_tables, create_user, get_user_by_username, create_user_profile, get_user_notes, get_chat_sessions_by_user, insert_user_note
from backend.rag.embedding import generate_embeddings
from backend.rag.retriever import QdrantRetriever
from backend.agent.agent import RAGAgent
from backend.auth.utils import verify_password, get_password_hash, create_access_token
from backend.auth.dependencies import get_current_user
from backend.schemas import User, UserInDB
from dotenv import load_dotenv

load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
SECRET_KEY = os.getenv("SECRET_KEY")
ALGORITHM = os.getenv("ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

app = FastAPI()

# Add CORS Middleware
from fastapi.middleware.cors import CORSMiddleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development/hackathon
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

@app.get("/")
async def root():
    return {"message": "Physical AI Textbook API is running", "docs_url": "/docs"}

try:
    if not QDRANT_HOST or not QDRANT_API_KEY:
        print("QDRANT_HOST or QDRANT_API_KEY not set in environment")
        raise ValueError("Missing QDRANT configuration")
    qdrant_retriever = QdrantRetriever(qdrant_host=QDRANT_HOST, qdrant_api_key=QDRANT_API_KEY)
    print("Qdrant connection successful.")
except Exception as e:
    print(f"Could not connect to Qdrant: {e}")
    print("Running in mock mode - RAG queries will return mock responses.")
    qdrant_retriever = None

rag_agent = RAGAgent(gemini_api_key=GEMINI_API_KEY)

# Pydantic models
class FileContent(BaseModel):
    file_path: str
    content: str
    module_id: str
    chapter_id: Optional[str] = None
    section_id: Optional[str] = None
    page_number: Optional[int] = None

class IngestRequest(BaseModel):
    files: List[FileContent]

class QueryRequest(BaseModel):
    query: str
    user_id: Optional[str] = None
    module_context: Optional[str] = None

class HighlightedQueryRequest(BaseModel):
    query: str
    highlighted_text: str
    user_id: Optional[str] = None
    module_context: Optional[str] = None

class TranslationRequest(BaseModel):
    text: str
    target_language: str

class PersonalizeRequest(BaseModel):
    text: str
    module_context: Optional[str] = None

class NoteCreate(BaseModel):
    note_content: str
    module_id: Optional[str] = None
    text_selection: Optional[str] = None

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None


class UserCreate(BaseModel):
    username: str
    password: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

async def get_user(username: str):
    db_user = get_user_by_username(username)
    if db_user:
        return UserInDB(**db_user)
    return None

@app.on_event("startup")
def startup_event():
    try:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            print("DATABASE_URL not set. Database functionality limited.")
        else:
            create_tables()
            print("Database tables ensured.")
    except Exception as e:
        print(f"Error creating tables: {e}")

@app.post("/register", response_model=User)
@limiter.limit("5/minute")
async def register(request: Request, user: UserCreate):
    db_user = await get_user(user.username)
    if db_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already registered"
        )
    user_id = create_user(
        user.username, 
        user.email, 
        get_password_hash(user.password),
        user.software_background,
        user.hardware_background
    )
    create_user_profile(user_id)
    return User(
        id=user_id, 
        username=user.username, 
        email=user.email, 
        full_name=user.full_name,
        software_background=user.software_background,
        hardware_background=user.hardware_background
    )

@app.post("/token", response_model=Token)
@limiter.limit("10/minute")
async def login_for_access_token(request: Request, form_data: Annotated[OAuth2PasswordRequestForm, Depends()]):
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
async def read_users_me(request: Request, current_user: Annotated[User, Depends(get_current_user)]):
    db_user = get_user_by_username(current_user.username)
    if not db_user:
         raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    # Return full DB user to include background info which might not be in token
    return User(**db_user)

@app.post("/users/me/notes")
@limiter.limit("10/minute")
async def create_my_note(request: Request, note: NoteCreate, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        
        note_id = insert_user_note(
            user_id=uuid.UUID(str(db_user["id"])),
            note_content=note.note_content,
            module_id=note.module_id,
            text_selection=note.text_selection
        )
        return {"note_id": str(note_id), "status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error saving note: {e}")

@app.get("/users/me/notes")
@limiter.limit("30/minute")
async def read_my_notes(request: Request, current_user: Annotated[User, Depends(get_current_user)]):
    db_user = get_user_by_username(current_user.username)
    if not db_user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    
    notes = get_user_notes(user_id=uuid.UUID(str(db_user["id"])))
    return notes

@app.get("/users/me/chat_sessions")
@limiter.limit("30/minute")
async def read_my_chat_sessions(request: Request, current_user: Annotated[User, Depends(get_current_user)]):
    db_user = get_user_by_username(current_user.username)
    if not db_user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    
    chat_sessions = get_chat_sessions_by_user(user_id=uuid.UUID(str(db_user["id"])))
    return chat_sessions

@app.post("/personalize")
@limiter.limit("10/minute")
async def personalize_content(request: Request, personalize_request: PersonalizeRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        
        sw_bg = db_user.get("software_background", "general")
        hw_bg = db_user.get("hardware_background", "general")

        prompt = (
            f"Rewrite the following educational content to be more suitable for a student with "
            f"a software background in '{sw_bg}' and a hardware background in '{hw_bg}'. "
            f"Use analogies and examples relevant to their background if possible. "
            f"Keep the core technical information accurate.\n\n"
            f"Content:\n{personalize_request.text}"
        )

        response = await rag_agent.generate_response(query=prompt, context=[])
        personalized_text = response.get("answer", "Could not personalize content.")
        
        return {"personalized_text": personalized_text}
    except Exception as e:
         raise HTTPException(status_code=500, detail=f"Error customizing content: {e}")

@app.post("/ingest")
@limiter.limit("1/minute")
async def ingest_content(request: Request, ingest_request: IngestRequest, background_tasks: BackgroundTasks):
    texts_to_embed = []
    metadatas_to_store = []
    
    for file_data in ingest_request.files:
        chunk_id = uuid.uuid4()
        texts_to_embed.append(file_data.content)
        metadatas_to_store.append({
            "id": str(chunk_id),
            "text": file_data.content,
            "module_id": file_data.module_id,
            "chapter_id": file_data.chapter_id,
            "section_id": file_data.section_id,
            "page_number": file_data.page_number,
            "file_path": file_data.file_path,
            "version": "1.0",
            "updated_at": datetime.now(timezone.utc).isoformat(),
        })

        background_tasks.add_task(
            insert_content_chunk,
            id=chunk_id,
            text=file_data.content,
            module_id=file_data.module_id,
            chapter_id=file_data.chapter_id,
            section_id=file_data.section_id,
            page_number=file_data.page_number,
            start_index=0,
            end_index=len(file_data.content),
            file_path=file_data.file_path,
            version="1.0",
        )
    
    async def _embed_and_upsert():
        try:
            embeddings = generate_embeddings(texts_to_embed, gemini_api_key=GEMINI_API_KEY, task_type="retrieval_document")
            qdrant_retriever.upsert_vectors(embeddings, metadatas_to_store)
            print(f"Successfully embedded and upserted {len(embeddings)} chunks to Qdrant.")
        except Exception as e:
            print(f"Error during embedding or Qdrant upsert: {e}")

    background_tasks.add_task(_embed_and_upsert)
    return {"status": "success", "message": f"{len(request.files)} files submitted for ingestion."}

@app.post("/query")
@limiter.limit("20/minute")
async def query_rag(request: Request, query_request: QueryRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        query_request.user_id = str(db_user["id"])

        if qdrant_retriever is None:
            response_data = {
                "answer": "Mock response: RAG system is not connected.",
                "citations": []
            }
        else:
            try:
                query_embedding = generate_embeddings([query_request.query], gemini_api_key=GEMINI_API_KEY, task_type="retrieval_query")[0]
                retrieved_chunks = qdrant_retriever.search_vectors(
                    query_embedding=query_embedding,
                    limit=7,
                    module_id=query_request.module_context
                )
                response_data = await rag_agent.generate_response(query_request.query, retrieved_chunks)
            except Exception as e:
                print(f"Embedding/Search failed (likely quota): {e}")
                # Fallback to direct model response without textbook context
                response_data = await rag_agent.generate_response(
                    query=f"Note: Search is temporarily disabled due to quota limits. Question: {query_request.query}", 
                    context=[]
                )

        return StreamingResponse(iter([json.dumps(response_data)]), media_type="application/json")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {e}")

@app.post("/highlighted-query")
@limiter.limit("20/minute")
async def highlighted_query_rag(request: Request, highlight_request: HighlightedQueryRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        highlight_request.user_id = str(db_user["id"])

        if qdrant_retriever is None:
            response_data = {
                "answer": "Mock response: RAG system is not connected.",
                "citations": []
            }
        else:
            highlight_context = [{
                "text": highlight_request.highlighted_text,
                "file_path": "user_highlight",
                "module_id": highlight_request.module_context or "unknown",
            }]
            try:
                query_embedding = generate_embeddings([highlight_request.query], gemini_api_key=GEMINI_API_KEY, task_type="retrieval_query")[0]
                retrieved_chunks = qdrant_retriever.search_vectors(
                    query_embedding=query_embedding,
                    limit=5,
                    module_id=highlight_request.module_context
                )
                combined_context = highlight_context + retrieved_chunks
                response_data = await rag_agent.generate_response(highlight_request.query, combined_context)
            except Exception as e:
                print(f"Embedding failed in highlight query: {e}")
                # Fallback: Just use the highlighted text as context
                response_data = await rag_agent.generate_response(highlight_request.query, highlight_context)

        return StreamingResponse(iter([json.dumps(response_data)]), media_type="application/json")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing highlighted query: {e}")

@app.post("/translate")
@limiter.limit("10/minute")
async def translate_text(request: Request, translate_request: TranslationRequest, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        translation_prompt = (
            f"Translate the following English text to Urdu. "
            f"Only return the translated Urdu text and nothing else:\n\n"
            f"{translate_request.text}"
        )
        translation_response = await rag_agent.generate_response(query=translation_prompt, context=[])
        translated_text = translation_response.get("answer", "Translation failed.")
        return {"translated_text": translated_text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error during translation: {e}")

@app.get("/users/me/recommendations")
@limiter.limit("5/minute")
async def get_my_recommendations(request: Request, current_user: Annotated[User, Depends(get_current_user)]):
    try:
        db_user = get_user_by_username(current_user.username)
        if not db_user:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
        
        user_id_uuid = uuid.UUID(str(db_user["id"]))
        user_notes = get_user_notes(user_id=user_id_uuid)
        user_chat_sessions = get_chat_sessions_by_user(user_id=user_id_uuid)
        
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
            return {"recommendations": "No sufficient data to generate recommendations yet."}

        recommendation_query = (
            "Based on the following user interactions, provide personalized learning recommendations. "
            "Suggest relevant topics or modules."
        )

        response_data = await rag_agent.generate_response(query=recommendation_query, context=context_for_recommendations)
        return {"recommendations": response_data.get("answer", "Could not generate recommendations.")}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating recommendations: {e}")

@app.get("/health")
async def health_check():
    status_qdrant = {"status": "ok", "message": ""}
    status_neon = {"status": "ok", "message": ""}
    try:
        qdrant_retriever.client.get_collections()
        status_qdrant["status"] = "ok"
    except Exception as e:
        status_qdrant["status"] = "error"
        status_qdrant["message"] = str(e)

    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute("SELECT 1")
        status_neon["status"] = "ok"
    except Exception as e:
        status_neon["status"] = "error"
        status_neon["message"] = str(e)
    finally:
        if conn:
            cur.close()
            conn.close()

    overall_status = "ok" if status_qdrant["status"] == "ok" and status_neon["status"] == "ok" else "error"
    return {"overall_status": overall_status, "services": {"qdrant": status_qdrant, "neon_postgres": status_neon}}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
