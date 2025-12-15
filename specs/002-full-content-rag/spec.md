# Feature Specification: Full Content Regeneration and Advanced RAG Chatbot System

**Feature Branch**: `002-full-content-rag`  
**Created**: 2025-12-09  
**Status**: Draft  
**Input**: User description: "Regenerate ALL modules of my “Physical AI & Humanoid Robotics” Docusaurus book, and also build a fully integrated RAG Chatbot system."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Regenerate Full Module Content (Priority: P1)

**Goal**: As a student, I want to access comprehensive, academically deep textbook modules with rich content including theory, examples, diagrams, formulas, lab exercises, and case studies, so I can thoroughly learn about Physical AI & Humanoid Robotics.

**Why this priority**: This is the foundational content for the entire textbook and directly addresses the core purpose of the project.

**Independent Test**: Can be fully tested by reviewing any single regenerated module Markdown file and verifying the presence and quality of all required content types (theory, examples, diagrams, etc.) and the absence of placeholders.

**Acceptance Scenarios**:
1.  **Given** a Docusaurus module page for "Module X", **When** I view the page, **Then** it displays in-depth theory and detailed subtopics.
2.  **Given** a Docusaurus module page, **When** I examine its content, **Then** it includes technical explanations and hands-on examples.
3.  **Given** a Docusaurus module page, **When** I read through the content, **Then** it contains visual diagrams (ASCII), system workflows, and mathematical formulas where relevant.
4.  **Given** a Docusaurus module page, **When** I review the entire module, **Then** it presents tables/charts, lab exercises, real-world industrial case studies, key takeaways, review questions, and a glossary.
5.  **Given** any generated module content, **When** I inspect the Markdown source, **Then** there are no placeholder lines (e.g., "[Generate detailed examples…]").

---

### User Story 2 - Interact with Full Book RAG Chatbot (Priority: P1)

**Goal**: As a student, I want to ask questions about the entire textbook content using an integrated chatbot, and receive accurate, cited answers, so I can quickly find information and deepen my understanding.

**Why this priority**: Provides core interactive learning functionality and leverages the full textbook content.

**Independent Test**: Can be tested by asking a question that spans multiple modules and verifying the chatbot's response includes relevant information and citations from multiple parts of the book.

**Acceptance Scenarios**:
1.  **Given** I am on any Docusaurus page, **When** I open the floating or side-panel RAG Chatbot, **Then** I can input a question about the textbook.
2.  **Given** I have asked a question, **When** the chatbot responds, **Then** the answer is based on the entire book content and includes citations linked back to the modules.
3.  **Given** the chatbot is integrated, **When** it provides a response, **Then** it is streamed, and relevant metadata is used for filtering during retrieval.

---

### User Story 3 - Interact with Selected-Text RAG Chatbot (Priority: P1)

**Goal**: As a student, I want to highlight text on a book page and ask the chatbot questions specifically about that selected text, so I can get highly contextualized answers without needing to retype the context.

**Why this priority**: Offers a critical, highly contextualized retrieval method for interactive learning.

**Independent Test**: Can be tested by highlighting a specific paragraph and asking a question that *only* the highlighted text can answer, then verifying the chatbot's response is limited to that context and falls back to full-book search if no text is selected.

**Acceptance Scenarios**:
1.  **Given** I am viewing a module page, **When** I highlight a section of text and click "Ask questions about my selected text", **Then** the chatbot's subsequent answer is based *only* on the selected text.
2.  **Given** I have not selected any text, **When** I ask a question via the chatbot, **Then** the chatbot automatically falls back to full-book RAG search.

---

### User Story 4 - Personalized & Translated Chatbot Interaction (Priority: P2)

**Goal**: As a student, I want to receive module-specific RAG responses, get custom learning recommendations, and view chat logs in Urdu, so my learning experience is tailored and accessible.

**Why this priority**: Enhances accessibility and personalization, supporting diverse user needs.

**Independent Test**: Can be tested by viewing module-specific responses, checking for personalized recommendations, and verifying Urdu translation capabilities.

**Acceptance Scenarios**:
1.  **Given** I am interacting with the chatbot on a specific module page, **When** I ask a question, **Then** the chatbot prioritizes retrieval and context relevant to that module.
2.  **Given** I click a "Personalization button", **When** the system provides recommendations, **Then** they are custom learning recommendations.
3.  **Given** I have a "My Notes + Chat Logs" panel, **When** I interact with the chatbot, **Then** my chat history is stored in Neon DB and accessible.
4.  **Given** I enable Urdu translation, **When** the chatbot responds, **Then** the response is translated into Urdu via OpenAI GPT.

### Edge Cases

-   What happens if a module Markdown file is empty during content generation? (System MUST generate full content as per FR-TEXT-001).
-   What happens if a user asks an out-of-scope question to the RAG chatbot? (Chatbot SHOULD respond gracefully, indicating limitations or inability to answer from the given context).
-   How does the RAG system handle very long documents during chunking? (System MUST chunk hierarchically and effectively to maintain context).
-   What happens if Qdrant or Neon DB is unavailable? (Backend MUST handle gracefully, return appropriate error messages, and attempt re-connection).
-   What happens if the OpenAI API is unavailable or rate-limited? (Backend MUST handle gracefully, implement retry mechanisms with exponential backoff, and inform the user if prolonged unavailability).
-   How is authentication handled if a user is not logged in for personalized features? (System SHOULD fallback to guest access for general RAG queries and prompt for login for personalized features).

## Requirements *(mandatory)*

### Functional Requirements - Textbook Content Regeneration

-   **FR-TEXT-001**: System MUST regenerate ALL existing Docusaurus module Markdown files (`/docs/modules/*`) with fully detailed and academically deep content.
-   **FR-TEXT-002**: Each regenerated module MUST include in-depth theory.
-   **FR-TEXT-003**: Each regenerated module MUST include detailed subtopics.
-   **FR-TEXT-004**: Each regenerated module MUST include technical explanations.
-   **FR-TEXT-005**: Each regenerated module MUST include hands-on examples.
-   **FR-TEXT-006**: Each regenerated module MUST include visual diagrams (ASCII diagrams).
-   **FR-TEXT-007**: Each regenerated module MUST include system workflows.
-   **FR-TEXT-008**: Each regenerated module MUST include mathematical formulas where appropriate.
-   **FR-TEXT-009**: Each regenerated module MUST include tables and charts where relevant.
-   **FR-TEXT-010**: Each regenerated module MUST include lab exercises.
-   **FR-TEXT-011**: Each regenerated module MUST include real-world industrial case studies.
-   **FR-TEXT-012**: Each regenerated module MUST include key takeaways.
-   **FR-TEXT-013**: Each regenerated module MUST include review questions.
-   **FR-TEXT-014**: Each regenerated module MUST include a glossary.
-   **FR-TEXT-015**: No placeholder lines are permitted in the final regenerated content.

### Functional Requirements - RAG Chatbot System (Full Book Retrieval)

-   **FR-RAG-FB-001**: The RAG system MUST load every file from `/docs/modules/**/*.md` for indexing.
-   **FR-RAG-FB-002**: The RAG system MUST chunk loaded content into hierarchical sections.
-   **FR-RAG-FB-003**: The RAG system MUST store content embeddings in Qdrant Cloud.
-   **FR-RAG-FB-004**: The RAG system MUST store content metadata in Neon Serverless Postgres.
-   **FR-RAG-FB-005**: The RAG system MUST enable hybrid search (Vector + Metadata filtering) from Qdrant.
-   **FR-RAG-FB-006**: The RAG system MUST use OpenAI `text-embedding-3-large` for embeddings.
-   **FR-RAG-FB-007**: The chatbot MUST return citations linked back to the Docusaurus modules.

### Functional Requirements - RAG Chatbot System (Selected-Text Retrieval)

-   **FR-RAG-ST-001**: The chatbot system MUST detect if a user has selected text within a book page.
-   **FR-RAG-ST-002**: If text is selected, the chatbot MUST answer questions *only* based on the selected text.
-   **FR-RAG-ST-003**: If no text is selected, the chatbot MUST fall back to full-book RAG search.

### Functional Requirements - Backend Stack

-   **FR-BACK-001**: The backend MUST be implemented using FastAPI.
-   **FR-BACK-002**: The backend MUST integrate OpenAI Agents / ChatKit SDK for chatbot responses.
-   **FR-BACK-003**: The backend MUST use Qdrant Cloud Free Tier for vector storage.
-   **FR-BACK-004**: The backend MUST use Neon Serverless Postgres for metadata and user notes/chat logs.
-   **FR-BACK-005**: The backend MUST support streaming responses for chatbot interactions.
-   **FR-BACK-006**: The backend MUST implement citation mapping.
-   **FR-BACK-007**: The backend MUST implement rate limiting for API access.
-   **FR-BACK-008**: The backend MUST implement authentication using JWT or Clerk/Auth.js.
-   **FR-BACK-009**: The backend MUST expose a `POST /ingest` endpoint for content ingestion.
-   **FR-BACK-010**: The backend MUST expose a `POST /query` endpoint for full-book RAG queries.
-   **FR-BACK-011**: The backend MUST expose a `POST /highlighted-query` endpoint for selected-text RAG queries.
-   **FR-BACK-012**: The backend MUST expose a `GET /health` endpoint for health checks.
-   **FR-BACK-013**: A Dockerfile for the FastAPI application MUST be provided.
-   **FR-BACK-014**: A README for deployment instructions MUST be provided.

### Functional Requirements - Frontend Integration (Docusaurus)

-   **FR-FRONT-001**: Every module page MUST embed a floating RAG Chatbot widget.
-   **FR-FRONT-002**: Every module page MUST embed a side-panel chatbot.
-   **FR-FRONT-003**: Every module page MUST include an "Ask questions about this section" button.
-   **FR-FRONT-004**: Every module page MUST include an "Ask questions about my selected text" button.
-   **FR-FRONT-005**: The frontend MUST display a contextual breadcrumb showing which module the answer comes from.
-   **FR-FRONT-006**: A personalized "My Notes + Chat Logs" panel MUST be integrated, storing data in Neon DB.
-   **FR-FRONT-007**: The chatbot frontend MUST support Urdu translation via OpenAI GPT model.
-   **FR-FRONT-008**: A Personalization button for custom learning recommendations MUST be present.

### Functional Requirements - Chatbot System Logic

-   **FR-LOGIC-001**: The chatbot logic MUST detect if the user has selected text; if so, it MUST use the selected-text pipeline.
-   **FR-LOGIC-002**: If no text is selected, the chatbot logic MUST run a normal RAG query (full-book search).
-   **FR-LOGIC-003**: The RAG system MUST retrieve top-k (5–8) chunks from Qdrant.
-   **FR-LOGIC-004**: Retrieved chunks MUST be re-ranked with an LLM.
-   **FR-LOGIC-005**: The system MUST construct a final context package for the LLM.
-   **FR-LOGIC-006**: OpenAI Agent with ChatKit MUST be used for generating chatbot responses.
-   **FR-LOGIC-007**: Citations linked back to modules MUST be returned with the response.
-   **FR-LOGIC-008**: Citations MUST be shown in the UI on the right side.

### Key Entities

-   **Module**: A Docusaurus Markdown file representing a textbook module. Attributes: `id`, `title`, `sidebar_label`, `full_content` (including theory, examples, diagrams, etc.).
-   **Text Chunk**: A segment of content from a module, used for RAG retrieval. Attributes: `text`, `embedding`, `module_id`, `chapter_id`, `section_id`, `page_number` (for citation mapping).
-   **Chat Session**: A record of a user's interaction with the chatbot. Attributes: `session_id`, `user_id`, `module_context` (if any), `chat_history` (array of messages), `timestamp`.
-   **User Notes**: User-specific notes associated with module content or chat interactions. Attributes: `note_id`, `user_id`, `module_id`, `text_selection` (if any), `note_content`, `timestamp`.
-   **User Profile**: User preferences for personalization and language. Attributes: `user_id`, `preferred_language` (e.g., 'en', 'ur'), `learning_style`, `background_info`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 100% of the Docusaurus module Markdown files are regenerated with full academic depth, containing all 13 specified content types, and no placeholder lines.
-   **SC-002**: The RAG chatbot successfully answers 90% of full-book queries with relevant, cited information.
-   **SC-003**: The RAG chatbot successfully answers 95% of selected-text queries with relevant, cited information *only* from the selected text.
-   **SC-004**: Average response time for chatbot queries is under 3 seconds.
-   **SC-005**: 100% of backend endpoints (`/ingest`, `/query`, `/highlighted-query`, `/health`) are implemented and functional.
-   **SC-006**: The Docusaurus frontend successfully embeds the floating widget, side-panel, and all specified buttons on every module page.
-   **SC-007**: User chat logs and personalized notes are successfully stored and retrieved from Neon Serverless Postgres.
-   **SC-008**: Urdu translation of chatbot responses functions correctly for 100% of test cases.
-   **SC-009**: The Docusaurus project builds successfully after all content regeneration and RAG integration.