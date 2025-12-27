import os
import uuid
from datetime import datetime, timezone
from typing import List, Dict, Any, Optional

import psycopg2
from psycopg2 import sql
from psycopg2.extras import Json
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

DATABASE_URL = os.getenv("DATABASE_URL")

def get_db_connection():
    if not DATABASE_URL:
        raise ValueError("DATABASE_URL environment variable not set.")
    return psycopg2.connect(DATABASE_URL)

def create_tables():
    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        # The schema.sql now contains `gen_random_uuid()` for UUID defaults.
        # Make sure this is enabled in your PostgreSQL if you get errors regarding the function.
        cur.execute("CREATE EXTENSION IF NOT EXISTS \"uuid-ossp\";")
        with open(os.path.join(os.path.dirname(__file__), "schema.sql"), "r") as f:
            cur.execute(f.read())
        conn.commit()
        print("Tables created successfully.")
    except Exception as e:
        print(f"Error creating tables: {e}")
        if conn:
            conn.rollback()
    finally:
        if conn:
            cur.close()
            conn.close()

# --- CRUD for Users ---
def create_user(username: str, email: str, hashed_password: str, software_background: Optional[str] = None, hardware_background: Optional[str] = None) -> uuid.UUID:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            INSERT INTO users (username, email, hashed_password, software_background, hardware_background)
            VALUES (%s, %s, %s, %s, %s) RETURNING id
            """,
            (username, email, hashed_password, software_background, hardware_background),
        )
        user_id = cur.fetchone()[0]
        conn.commit()
        return user_id
    except Exception as e:
        print(f"Error creating user: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

def get_user_by_username(username: str) -> Optional[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT id, username, email, hashed_password, software_background, hardware_background FROM users WHERE username = %s",
            (username,),
        )
        row = cur.fetchone()
        if row:
            columns = [desc[0] for desc in cur.description]
            return dict(zip(columns, row))
        return None
    finally:
        cur.close()
        conn.close()

def get_user_by_id(user_id: uuid.UUID) -> Optional[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT id, username, email FROM users WHERE id = %s",
            (str(user_id),),
        )
        row = cur.fetchone()
        if row:
            columns = [desc[0] for desc in cur.description]
            return dict(zip(columns, row))
        return None
    finally:
        cur.close()
        conn.close()

# --- CRUD for ContentChunk Metadata ---
def insert_content_chunk(
    id: uuid.UUID,
    text: str,
    module_id: str,
    chapter_id: Optional[str],
    section_id: Optional[str],
    page_number: Optional[int],
    start_index: int,
    end_index: int,
    file_path: str,
    version: str,
):
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            INSERT INTO content_chunks (id, text, module_id, chapter_id, section_id, page_number, start_index, end_index, file_path, version)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            """,
            (
                str(id), text, module_id, chapter_id, section_id, page_number,
                start_index, end_index, file_path, version
            ),
        )
        conn.commit()
    except Exception as e:
        print(f"Error inserting content chunk: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

def get_content_chunk_metadata(id: uuid.UUID) -> Optional[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT id, text, module_id, chapter_id, section_id, page_number, start_index, end_index, file_path, version, updated_at FROM content_chunks WHERE id = %s",
            (str(id),),
        )
        row = cur.fetchone()
        if row:
            columns = [desc[0] for desc in cur.description]
            return dict(zip(columns, row))
        return None
    finally:
        cur.close()
        conn.close()

# --- CRUD for ChatSession ---
def create_chat_session(user_id: Optional[uuid.UUID] = None, module_context: Optional[str] = None) -> uuid.UUID:
    session_id = uuid.uuid4()
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            INSERT INTO chat_sessions (session_id, user_id, module_context, chat_history)
            VALUES (%s, %s, %s, %s)
            """,
            (str(session_id), str(user_id) if user_id else None, module_context, Json([]))
        )
        conn.commit()
        return session_id
    except Exception as e:
        print(f"Error creating chat session: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

def get_chat_session(session_id: uuid.UUID) -> Optional[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT session_id, user_id, module_context, chat_history, created_at, updated_at FROM chat_sessions WHERE session_id = %s",
            (str(session_id),),
        )
        row = cur.fetchone()
        if row:
            columns = [desc[0] for desc in cur.description]
            return dict(zip(columns, row))
        return None
    finally:
        cur.close()
        conn.close()

def get_chat_sessions_by_user(user_id: uuid.UUID) -> List[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT session_id, user_id, module_context, chat_history, created_at, updated_at FROM chat_sessions WHERE user_id = %s ORDER BY updated_at DESC",
            (str(user_id),),
        )
        rows = cur.fetchall()
        columns = [desc[0] for desc in cur.description]
        return [dict(zip(columns, row)) for row in rows]
    finally:
        cur.close()
        conn.close()

def update_chat_history(session_id: uuid.UUID, new_message: Dict[str, Any]):
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            UPDATE chat_sessions
            SET chat_history = chat_history || %s::jsonb, updated_at = %s
            WHERE session_id = %s
            """,
            (Json([new_message]), datetime.now(timezone.utc), str(session_id)),
        )
        conn.commit()
    except Exception as e:
        print(f"Error updating chat history: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

# --- CRUD for UserNote ---
def insert_user_note(
    user_id: uuid.UUID,
    note_content: str,
    module_id: Optional[str] = None,
    text_selection: Optional[str] = None,
) -> uuid.UUID:
    note_id = uuid.uuid4()
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            INSERT INTO user_notes (note_id, user_id, module_id, text_selection, note_content)
            VALUES (%s, %s, %s, %s, %s)
            """,
            (str(note_id), str(user_id), module_id, text_selection, note_content),
        )
        conn.commit()
        return note_id
    except Exception as e:
        print(f"Error inserting user note: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

def get_user_notes(user_id: uuid.UUID, module_id: Optional[str] = None) -> List[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        query = "SELECT note_id, user_id, module_id, text_selection, note_content, created_at, updated_at FROM user_notes WHERE user_id = %s"
        params = [str(user_id)]
        if module_id:
            query += " AND module_id = %s"
            params.append(module_id)
        cur.execute(query, tuple(params))
        rows = cur.fetchall()
        columns = [desc[0] for desc in cur.description]
        return [dict(zip(columns, row)) for row in rows]
    finally:
        cur.close()
        conn.close()

# --- CRUD for UserProfile ---
def create_user_profile(user_id: uuid.UUID) -> None:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            """
            INSERT INTO user_profiles (user_id)
            VALUES (%s)
            """,
            (str(user_id),),
        )
        conn.commit()
    except Exception as e:
        print(f"Error creating user profile: {e}")
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()

def get_user_profile(user_id: uuid.UUID) -> Optional[Dict[str, Any]]:
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT id, user_id, preferred_language, learning_style, background_info, created_at, updated_at FROM user_profiles WHERE user_id = %s",
            (str(user_id),),
        )
        row = cur.fetchone()
        if row:
            columns = [desc[0] for desc in cur.description]
            return dict(zip(columns, row))
        return None
    finally:
        cur.close()
        conn.close()

if __name__ == "__main__":
    # Example usage:
    # Set DATABASE_URL in your .env file
    # create_tables()
    pass
