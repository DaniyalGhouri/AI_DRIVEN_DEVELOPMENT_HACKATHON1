# Implementation Plan: Full Content Regeneration and Advanced RAG Chatbot System

**Branch**: `002-full-content-rag` | **Date**: 2025-12-09 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/002-full-content-rag/spec.md`

## Summary

This plan details the full content regeneration of all Docusaurus modules with academic depth and the development of an integrated RAG Chatbot system. The textbook content will be updated to include comprehensive theory, examples, diagrams, and exercises. The RAG chatbot will feature full-book and selected-text retrieval, a FastAPI backend, Qdrant for vector storage, Neon for metadata and user data, and a Docusaurus frontend with an embedded chat widget, personalization, and Urdu translation capabilities.

## Technical Context

**Language/Version**: Python 3.10+ (for FastAPI backend), JavaScript (ES2020+), TypeScript (for Docusaurus frontend)
**Primary Dependencies**:
    *   **Backend**: FastAPI, OpenAI Python client, Qdrant client, Psycopg2/asyncpg (for Neon), LangChain/LlamaIndex (for RAG orchestration), ChatKit SDK (for OpenAI Agents)
    *   **Frontend**: Docusaurus, React, Axios/Fetch (for API calls)
**Storage**: Qdrant Cloud Free Tier (vector embeddings), Neon Serverless Postgres (metadata, chat logs, user notes, user profiles)
**Testing**: Pytest (FastAPI), Jest (Docusaurus/Frontend)
**Target Platform**: Web (Docusaurus frontend), Serverless/Containerized (FastAPI backend)
**Project Type**: Monorepo (Docusaurus frontend, FastAPI backend)
**Performance Goals**: Chatbot response < 3s, streaming responses for perceived speed, content ingestion (indexing all modules) < 1 hour.
**Constraints**: Qdrant Cloud Free Tier limitations, OpenAI API rate limits and cost, local + cloud-friendly architecture, strict adherence to Docusaurus markdown rendering, open-source compatible licenses.
**Scale/Scope**: All current and future modules, support for multiple concurrent users for the RAG chatbot, personalized features for each user.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

-   **Principle 1: Purpose**: PASSED - Directly aligns with teaching Physical AI & Humanoid Robotics with interactive AI-enabled learning.
-   **Principle 2: Objectives**: PASSED - Directly addresses the creation of fully detailed modules and embedding an advanced RAG Chatbot with specific retrieval, personalization, and translation features.
-   **Principle 3: Core Components**: PASSED - Leverages Docusaurus, Advanced RAG Chatbot components (FastAPI, Qdrant, Neon, OpenAI Agents/ChatKit).
-   **Principle 4: Constraints**: PASSED - Adheres to the Docusaurus framework, open-source compatibility (implied by choices), and technical accuracy will be maintained through content generation.
-   **Principle 5: Workflows**: PASSED - Establishes Content Generation and Student Learning workflows, enhanced by RAG.
-   **Principle 6: Textbook Content Standards**: PASSED - The plan explicitly aims for full academic depth, detailed content types, and no placeholders as per the new constitution.
-   **Principle 7: RAG Chatbot System Standards**: PASSED - The plan aligns with all specified backend, frontend, and logic standards for the RAG chatbot.

## Project Structure

### Documentation (this feature)

```text
specs/002-full-content-rag/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── ingest.yaml
│   ├── query.yaml
│   ├── highlighted_query.yaml
│   └── health.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # FastAPI app entry point
├── rag/                 # RAG specific logic
│   ├── ingest.py        # Content ingestion and chunking
│   ├── retriever.py     # Qdrant querying and hybrid search
│   └── embedding.py     # OpenAI embedding generation
├── db/                  # Database related files
│   ├── postgres.py      # Neon Postgres connection and ORM
│   └── schema.sql       # SQL schema for Neon DB
├── agent/               # OpenAI Agents / ChatKit SDK integration
│   └── agent.py         # Agent orchestration
├── Dockerfile           # For FastAPI deployment
└── README.md            # Deployment instructions for backend

physical-ai-humanoid-robotics-textbook/  # Existing Docusaurus project
├── docs/modules/        # Fully regenerated Markdown files
│   ├── module-0-constraints.md
│   ├── module-1-introduction.md
│   ├── module-2-ros2.md
│   ├── module-3-simulation.md
│   └── module-4-perception.md
├── src/components/      # New React components for RAG frontend
│   ├── RagChatWidget/
│   │   └── index.tsx
│   ├── ChatPanel/
│   │   └── index.tsx
│   └── HighlightRagButton.tsx
├── src/lib/             # Frontend API service
│   └── ragClient.ts
├── src/theme/           # Docusaurus theme overrides
│   └── Root.js          # For injecting global widget
└── (existing Docusaurus files)

scripts/                 # (Existing) For sidebar generation and content generation orchestration
├── generate-sidebar.js  # (Existing)
└── generate-content.js  # New script for content generation
module-order.json        # (Existing) Defines module order
```

**Structure Decision**: A monorepo structure will be adopted, with the existing `physical-ai-humanoid-robotics-textbook/` directory serving as the frontend (Docusaurus project) and a new `backend/` directory for the FastAPI application. A new `scripts/generate-content.js` will be added to orchestrate content generation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
|           |            |                                     |