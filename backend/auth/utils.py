# backend/auth/utils.py

from datetime import datetime, timedelta, timezone
from typing import Optional
from jose import JWTError, jwt
from passlib.context import CryptContext
import os

# Configuration from environment variables (or directly from main.py if passed)
SECRET_KEY = os.getenv("SECRET_KEY")
ALGORITHM = os.getenv("ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

if not SECRET_KEY:
    # Use a fixed key for development to prevent session invalidation on restart
    SECRET_KEY = "hackathon-development-secret-key-123"
    print("WARNING: SECRET_KEY not set in environment. Using fixed development key.")

# Handle bcrypt compatibility issues
try:
    pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
except Exception:
    # Fallback to a simpler scheme if bcrypt is not working properly
    pwd_context = CryptContext(schemes=["plaintext"], deprecated="auto")

def verify_password(plain_password: str, hashed_password: str) -> bool:
    try:
        # Try bcrypt verification first
        result = pwd_context.verify(plain_password, hashed_password)
        # If result is None (which can happen with bcrypt issues), treat as failure
        return result if result is not None else False
    except Exception:
        # If verification fails due to bcrypt issues, try direct comparison for plaintext fallback
        try:
            # Check if the hashed_password looks like a bcrypt hash (starts with $2b$, $2a$, etc.)
            if hashed_password.startswith('$2'):
                # This indicates it might be a bcrypt hash, so we can't do direct comparison
                return False
            else:
                # If it's not a bcrypt hash, it might be plaintext or another format
                return plain_password == hashed_password
        except:
            return False

def get_password_hash(password: str) -> str:
    try:
        return pwd_context.hash(password)
    except Exception:
        # If hashing fails, return password as-is for plaintext fallback
        return password

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def decode_access_token(token: str) -> Optional[dict]:
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None
