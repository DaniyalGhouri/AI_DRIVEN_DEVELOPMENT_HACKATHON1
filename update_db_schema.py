import os
import psycopg2
from dotenv import load_dotenv

# Load environment variables
load_dotenv(os.path.join(os.path.dirname(__file__), 'backend', '.env'))

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    print("Error: DATABASE_URL not set.")
    exit(1)

def update_schema():
    conn = None
    try:
        conn = psycopg2.connect(DATABASE_URL)
        cur = conn.cursor()
        
        print("Adding missing columns to 'users' table...")
        
        # Add software_background column
        cur.execute("""
            ALTER TABLE users 
            ADD COLUMN IF NOT EXISTS software_background TEXT;
        """)
        
        # Add hardware_background column
        cur.execute("""
            ALTER TABLE users 
            ADD COLUMN IF NOT EXISTS hardware_background TEXT;
        """)
        
        conn.commit()
        print("Schema updated successfully.")
        
    except Exception as e:
        print(f"Error updating schema: {e}")
        if conn:
            conn.rollback()
    finally:
        if conn:
            cur.close()
            conn.close()

if __name__ == "__main__":
    update_schema()
