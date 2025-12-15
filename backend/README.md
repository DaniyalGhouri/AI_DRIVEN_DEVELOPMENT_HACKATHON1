# FastAPI Backend for RAG Chatbot

This directory contains the FastAPI backend application for the RAG chatbot system.

## Setup

1.  **Environment Variables**: Create a `.env` file in the `backend/` directory based on `.env.example`.
2.  **Install Dependencies**:
    ```bash
    pipenv install
    ```
3.  **Database Migrations**:
    ```bash
    alembic upgrade head # (Requires Alembic to be set up)
    ```

## Running Locally

```bash
pipenv run uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

## Docker

To build the Docker image:

```bash
docker build -t rag-backend .
```

To run the Docker container:

```bash
docker run -p 8000:8000 rag-backend
```

## API Endpoints

-   `POST /ingest`: Ingest Markdown files
-   `POST /query`: Query the RAG system (full book)
-   `POST /highlighted-query`: Query with highlighted text
-   `GET /health`: Health check
