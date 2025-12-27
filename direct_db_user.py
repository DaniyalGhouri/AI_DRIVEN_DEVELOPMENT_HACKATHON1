# direct_db_user.py
import uuid
from datetime import datetime, timezone
from backend.db.postgres import create_user, get_db_connection
from backend.auth.utils import get_password_hash

def create_admin_user():
    """Directly create an admin user in the database"""
    try:
        # Create a user directly in the database
        username = "admin"
        email = "admin@example.com"
        password = "admin123"  # Shorter password to avoid bcrypt issues

        # Hash the password
        hashed_password = get_password_hash(password)

        # Create the user in the database
        user_id = create_user(username, email, hashed_password)
        print(f"Successfully created user {username} with ID: {user_id}")
        print(f"Username: {username}")
        print(f"Password: {password}")
        print("You can now use these credentials to log in!")

        return True
    except Exception as e:
        print(f"Error creating user directly in database: {e}")
        # If bcrypt issue, try to continue with login attempts anyway
        if "password cannot be longer than 72 bytes" in str(e):
            print("Password was too long - use a shorter password")
        return False

if __name__ == "__main__":
    print("Creating admin user directly in database...")
    success = create_admin_user()
    if success:
        print("\nYou can now log in with:")
        print("  Username: admin_user")
        print("  Password: secure_password123")
        print("And get your JWT token!")