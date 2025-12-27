from pydantic import BaseModel
from typing import Optional
import uuid

class User(BaseModel):
    id: uuid.UUID
    username: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    disabled: Optional[bool] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

class UserInDB(User):
    hashed_password: str
