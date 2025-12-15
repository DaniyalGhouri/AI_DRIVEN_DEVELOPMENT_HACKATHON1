# backend/auth/dependencies.py

from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from typing import Annotated

from backend.auth.utils import decode_access_token
# Assuming a User model will be defined elsewhere, e.g., in main.py or a dedicated models.py
# from .models import UserInDB # Placeholder for now

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

async def get_current_user(token: Annotated[str, Depends(oauth2_scheme)]):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    payload = decode_access_token(token)
    if payload is None:
        raise credentials_exception
    
    # In a real application, you would fetch the user from the database here
    # based on the payload (e.g., user ID). For now, we'll just return the username.
    username: str = payload.get("sub")
    if username is None:
        raise credentials_exception
    
    # Placeholder for a user object; replace with actual database lookup
    # user = get_user(db, username=username) 
    # if user is None:
    #     raise credentials_exception
    
    return {"username": username} # Return basic user info for now
