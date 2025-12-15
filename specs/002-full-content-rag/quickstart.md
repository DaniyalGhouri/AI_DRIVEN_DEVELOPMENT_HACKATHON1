# Quickstart: Full Content Regeneration and Advanced RAG Chatbot System

This guide outlines the steps to set up, regenerate textbook content, and deploy the RAG Chatbot system.

## 1. Environment Setup

-   **Clone the repository**: Ensure you have the latest code.
-   **Backend dependencies**:
    -   Install Python 3.10+.
    -   Install `pipenv` or `poetry`.
    -   Run `pipenv install` (or `poetry install`) in the `backend/` directory.
-   **Frontend dependencies**:
    -   Install Node.js (LTS recommended).
    -   Run `npm install` (or `yarn install`) in the `physical-ai-humanoid-robotics-textbook/` directory.
-   **API Keys**: Obtain API keys for OpenAI (for embeddings and ChatKit), Qdrant Cloud, and Neon Serverless Postgres. Configure them as environment variables (e.g., in a `.env` file).

## 2. Content Regeneration

1.  Ensure you are on the `002-full-content-rag` branch.
2.  Run the content generation script (to be developed) to populate the Docusaurus modules with detailed content. This script will overwrite existing content in `physical-ai-humanoid-robotics-textbook/docs/modules/*.md`.

## 3. RAG System Deployment

### a. Backend (FastAPI)

1.  Navigate to the `backend/` directory.
2.  Configure environment variables for Qdrant (URL, API Key), Neon (Postgres Connection String), and OpenAI (API Key).
3.  Run database migrations: `alembic upgrade head` (assuming Alembic is used for migrations).
4.  Start the FastAPI application: `uvicorn main:app --reload` (for development) or build/run Docker image for production.

### b. Content Ingestion

1.  Once the FastAPI backend is running, use the `/ingest` API endpoint to load the textbook content.
2.  Develop a separate script or use `curl`/Postman to send Markdown files to the `/ingest` endpoint.

### c. Frontend (Docusaurus) Integration

1.  Ensure the FastAPI backend is deployed and accessible.
2.  Update Docusaurus configuration (if necessary) to point to the RAG backend API endpoints.
3.  Run `npm run start` (or `npm run build` for production) in `physical-ai-humanoid-robotics-textbook/` to see the integrated chatbot widget.

## 4. Testing

-   Verify full-book retrieval by asking general questions.
-   Verify selected-text retrieval by highlighting text and asking contextual questions.
-   Check personalized features and Urdu translation if configured.
-   Monitor backend health via `GET /health` endpoint.
