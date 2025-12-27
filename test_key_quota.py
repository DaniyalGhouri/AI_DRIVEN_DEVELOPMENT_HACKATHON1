import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv("backend/.env")
api_key = os.getenv("GEMINI_API_KEY")

if not api_key:
    print("No API key found.")
    exit()

genai.configure(api_key=api_key)

models_to_try = ["gemini-2.0-flash", "gemini-flash-latest", "gemini-pro-latest"]

for model_name in models_to_try:
    print(f"\n--- Testing {model_name} ---")
    try:
        model = genai.GenerativeModel(model_name)
        response = model.generate_content("Say 'ready' if you can hear me.")
        print(f"Success: {response.text}")
        break 
    except Exception as e:
        print(f"Failed: {e}")
