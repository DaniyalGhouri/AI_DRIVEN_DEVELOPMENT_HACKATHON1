# Tasks: Full Content Regeneration and Advanced RAG Chatbot System

**Input**: Design documents from `specs/002-full-content-rag/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initial project setup, environment configuration for both backend and frontend.

- [X] T001 Create `backend/` directory
- [X] T002 Create `backend/rag/` directory
- [X] T003 Create `backend/db/` directory
- [X] T004 Create `backend/agent/` directory
- [X] T005 [P] Create `scripts/generate-content.js` file (placeholder, will be implemented later)
- [X] T006 [P] Create `backend/Dockerfile`
- [X] T007 [P] Create `backend/README.md`
- [X] T008 [P] Initialize Python environment in `backend/` (e.g., `pipenv install fastapi openai qdrant-client psycopg2-binary`)
- [X] T009 [P] Update `physical-ai-humanoid-robotics-textbook/package.json` with necessary RAG frontend dependencies (e.g., `axios`)
- [X] T010 [P] Create `.env.example` in `backend/` for environment variables (OpenAI, Qdrant, Neon credentials)
- [X] T011 Update `physical-ai-humanoid-robotics-textbook/docusaurus.config.ts` to disable or warn for broken links instead of throwing (for development stability)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Implement core RAG components and database schema that are needed before content ingestion or chatbot interaction.

- [X] T012 Implement `backend/db/schema.sql` (Neon Serverless Postgres schema for ContentChunk metadata, ChatSession, UserNote, UserProfile)
- [X] T013 Implement `backend/db/postgres.py` for Neon database connection and basic CRUD operations
- [X] T014 Implement `backend/rag/embedding.py` for OpenAI `text-embedding-3-large` client initialization and embedding generation
- [X] T015 Implement `backend/rag/retriever.py` for Qdrant client initialization, vector storage, and hybrid search logic

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Regenerate Full Module Content (Priority: P1) 🎯 MVP

**Goal**: Comprehensive, academically deep textbook modules with rich content.

**Independent Test**: Review any single regenerated module Markdown file for presence and quality of all required content types and absence of placeholders.

### Implementation for User Story 1

- [X] T016 [US1] Implement `scripts/generate-content.js` to connect to OpenAI API, get module details, and generate full content for `physical-ai-humanoid-robotics-textbook/docs/modules/module-0-constraints.md` including all 13 content types.
- [X] T017 [P] [US1] Implement `scripts/generate-content.js` to connect to OpenAI API, get module details, and generate full content for `physical-ai-humanoid-robotics-textbook/docs/modules/module-1-introduction.md` including all 13 content types.
- [X] T018 [P] [US1] Implement `scripts/generate-content.js` to connect to OpenAI API, get module details, and generate full content for `physical-ai-humanoid-robotics-textbook/docs/modules/module-2-ros2.md` including all 13 content types.
- [X] T019 [P] [US1] Implement `scripts/generate-content.js` to connect to OpenAI API, get module details, and generate full content for `physical-ai-humanoid-robotics-textbook/docs/modules/module-3-simulation.md` including all 13 content types.
- [X] T020 [P] [US1] Implement `scripts/generate-content.js` to connect to OpenAI API, get module details, and generate full content for `physical-ai-humanoid-robotics-textbook/docs/modules/module-4-perception.md` including all 13 content types.
- [X] T021 [US1] Run `scripts/generate-content.js` to regenerate all module content.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interact with Full Book RAG Chatbot (Priority: P1)

**Goal**: Ask questions about the entire textbook content using an integrated chatbot and receive accurate, cited answers.

**Independent Test**: Ask a question that spans multiple modules and verify the chatbot's response includes relevant information and citations from multiple parts of the book.

### Implementation for User Story 2

- [X] T022 [US2] Implement `backend/main.py` (FastAPI) with `POST /ingest` endpoint logic (content chunking, embedding generation, storing in Qdrant and Neon).
- [X] T023 [US2] Implement `backend/agent/agent.py` for OpenAI Agent/ChatKit SDK integration, response generation, and citation mapping.
- [X] T024 [P] [US2] Implement `backend/main.py` (FastAPI) with `POST /query` endpoint logic (full book retrieval, LLM re-ranking, OpenAI Agent/ChatKit response, streaming, citations).
- [X] T025 [US2] Create and run a script to ingest all regenerated module content via `POST /ingest`.
- [X] T026 [P] [US2] Create `physical-ai-humanoid-robotics-textbook/src/components/RagChatWidget/index.tsx` (floating widget frontend component).
- [X] T027 [P] [US2] Create `physical-ai-humanoid-robotics-textbook/src/components/ChatPanel/index.tsx` (side-panel chatbot frontend component).
- [X] T028 [P] [US2] Create `physical-ai-humanoid-robotics-textbook/src/lib/ragClient.ts` for frontend API service to interact with backend RAG endpoints.
- [X] T029 [US2] Inject global `RagChatWidget` into Docusaurus using `physical-ai-humanoid-robotics-textbook/src/theme/Root.js`.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interact with Selected-Text RAG Chatbot (Priority: P1)

**Goal**: Highlight text and ask chatbot questions specifically about that text.

**Independent Test**: Highlight a specific paragraph and ask a question that *only* the highlighted text can answer, then verify response is limited to that context.

### Implementation for User Story 3

- [X] T030 [US3] Implement `backend/main.py` (FastAPI) with `POST /highlighted-query` endpoint logic (selected-text RAG query).
- [X] T031 [P] [US3] Create `physical-ai-humanoid-robotics-textbook/src/components/HighlightRagButton.tsx` (frontend button component).
- [X] T032 [US3] Integrate `HighlightRagButton.tsx` into Docusaurus module pages for selected text detection and API call.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Personalized & Translated Chatbot Interaction (Priority: P2)

**Goal**: Receive module-specific RAG responses, get custom learning recommendations, and view chat logs in Urdu.

**Independent Test**: Verify module-specific responses, check for basic personalized recommendations, and confirm Urdu translation.

### Implementation for User Story 4

- [X] T033 [US4] Implement `backend/main.py` (FastAPI) with JWT-based authentication for user management and session tracking.
- [X] T034 [US4] Update `backend/main.py` query endpoints (`/query`, `/highlighted-query`) to use `user_id` and `module_context` for personalized retrieval.
- [X] T035 [P] [US4] Implement "My Notes + Chat Logs" panel logic in `ChatPanel/index.tsx` to store/retrieve from Neon DB via backend.
- [X] T036 [P] [US4] Implement Urdu translation logic in `backend/main.py` (FastAPI) via OpenAI GPT model.
- [X] T037 [US4] Integrate Urdu translation feature into chatbot UI in `ChatPanel/index.tsx`.
- [X] T038 [P] [US4] Create a Personalization button component (within `ChatPanel/index.tsx` or separate) for basic recommendations.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final verification, optimization, and documentation.

- [X] T039 Implement rate limiting for all relevant endpoints in `backend/main.py`.
- [X] T040 Implement `backend/main.py` with `GET /health` endpoint (including checks for Qdrant and Neon status).
- [X] T041 Review and update `physical-ai-humanoid-robotics-textbook/docusaurus.config.ts` for any final RAG integration settings (e.g., custom themes, plugins).
- [ ] T042 Final code review and refactoring for both backend and frontend.
- [X] T043 Run `npm run build` in `physical-ai-humanoid-robotics-textbook/` to verify site builds without errors.
- [ ] T044 Run `npm start` in `physical-ai-humanoid-robotics-textbook/` for visual inspection and navigation testing.
- [ ] T045 Update overall project `README.md` with instructions for content generation, RAG setup, and deployment.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Story 1 (Phase 3)**: Depends on Foundational completion.
-   **User Story 2 (Phase 4)**: Depends on Foundational and User Story 1 (for content) completion.
-   **User Story 3 (Phase 5)**: Depends on Foundational and User Story 2 completion.
-   **User Story 4 (Phase 6)**: Depends on Foundational and User Story 2 completion.
-   **Polish (Phase 7)**: Depends on all user stories being complete.

### User Story Dependencies

-   **US1 (P1 - Content Regeneration)**: Depends on Foundational.
-   **US2 (P1 - Full Book RAG)**: Depends on Foundational and US1 (for indexed content).
-   **US3 (P1 - Selected-Text RAG)**: Depends on Foundational and US2.
-   **US4 (P2 - Personalized/Translated RAG)**: Depends on Foundational and US2.

### Within Each User Story

-   Backend components generally before frontend integration.
-   Core logic before UI components.
-   Tests (if generated) MUST be written and FAIL before implementation.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel.
-   Multiple content generation tasks (T017-T020) can run in parallel once T016 is complete.
-   Frontend RAG components (T026-T028) can be developed in parallel with backend endpoints (T022-T024), assuming API contracts are stable.
-   T025 and T026 can be developed in parallel.
-   T035, T036, T038 can be developed in parallel.
-   T040-T045 can be performed in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 & 2)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1 (Content Regeneration)
4.  Complete Phase 4: User Story 2 (Full Book RAG Chatbot)
5.  **STOP and VALIDATE**: Test content and full-book RAG independently.
6.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready.
2.  Add User Story 1 (Content) → Test independently.
3.  Add User Story 2 (Full RAG) → Test independently.
4.  Add User Story 3 (Selected Text RAG) → Test independently.
5.  Add User Story 4 (Personalized/Translated) → Test independently.
6.  Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 1 (Content Regeneration)
    -   Developer B: User Story 2 (Full Book RAG)
    -   Developer C: User Story 3 (Selected Text RAG)
    -   Developer D: User Story 4 (Personalized/Translated RAG)
3.  Stories complete and integrate independently.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
```