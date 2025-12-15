# Data Model: Full Content Regeneration and Advanced RAG Chatbot System

## 1. Textbook Module Content (for Regeneration)

-   **Module**: A Docusaurus Markdown file representing a textbook module.
    -   `id`: (string) Unique identifier (e.g., `module-1-introduction`).
    -   `title`: (string) Module title.
    -   `sidebar_label`: (string) Label for Docusaurus sidebar.
    -   `full_content`: (string, Markdown) Comprehensive content including:
        -   In-depth theory
        -   Detailed subtopics
        -   Technical explanations
        -   Hands-on examples
        -   Visual diagrams (ASCII representation)
        -   System workflows
        -   Mathematical formulas
        -   Tables and charts
        -   Lab exercises
        -   Real-world industrial case studies
        -   Key takeaways
        -   Review questions
        -   Glossary terms and definitions

## 2. RAG System Data Models (Neon Serverless Postgres & Qdrant Cloud)

### a. `ContentChunk` (Qdrant & Neon Metadata)

Represents a semantically coherent chunk of text from a module.

-   `id`: (UUID) Unique ID for the chunk. (Qdrant payload field)
-   `text`: (string) The actual text content of the chunk. (Qdrant payload field)
-   `embedding`: (vector) OpenAI `text-embedding-3-large` vector representation of the `text`. (Qdrant vector)
-   `module_id`: (string) ID of the module the chunk belongs to (e.g., `module-1-introduction`). (Qdrant payload field, Neon field)
-   `chapter_id`: (string, optional) ID of the chapter within the module. (Qdrant payload field, Neon field)
-   `section_id`: (string, optional) ID of the section within the chapter/module. (Qdrant payload field, Neon field)
-   `page_number`: (integer, optional) Page number or approximate location for citation. (Neon field)
-   `start_index`: (integer) Character start index of the chunk within the original file. (Neon field)
-   `end_index`: (integer) Character end index of the chunk within the original file. (Neon field)
-   `file_path`: (string) Relative path to the original Markdown file (e.g., `docs/modules/module-1-introduction.md`). (Neon field)
-   `version`: (string) Version of the textbook content. (Neon field)
-   `updated_at`: (timestamp) Last update time. (Neon field)

### b. `ChatSession` (Neon Serverless Postgres)

Represents a single conversation session with a user.

-   `session_id`: (UUID) Unique ID for the chat session.
-   `user_id`: (UUID, optional) ID of the authenticated user.
-   `module_context`: (string, optional) `module_id` if the chat is contextual to a specific module page.
-   `chat_history`: (JSONB/array of objects) Array of `{ role: 'user' | 'assistant', content: string, timestamp: datetime }`.
-   `created_at`: (timestamp) Session creation time.
-   `updated_at`: (timestamp) Last update time.

### c. `UserNote` (Neon Serverless Postgres)

Represents a user's personal notes.

-   `note_id`: (UUID) Unique ID for the note.
-   `user_id`: (UUID) ID of the authenticated user.
-   `module_id`: (string, optional) ID of the module the note is associated with.
-   `text_selection`: (string, optional) The text highlighted by the user to create the note.
-   `note_content`: (string) The actual note content.
-   `created_at`: (timestamp) Note creation time.
-   `updated_at`: (timestamp) Last update time.

### d. `UserProfile` (Neon Serverless Postgres)

Stores user preferences and personalization data.

-   `user_id`: (UUID) Unique ID for the user.
-   `username`: (string) User's chosen username.
-   `email`: (string) User's email address.
-   `preferred_language`: (string, e.g., 'en', 'ur') User's preferred language for chatbot responses.
-   `learning_style`: (string, optional) e.g., 'visual', 'auditory', 'kinesthetic'.
-   `background_info`: (JSONB, optional) Additional data for personalization.
-   `created_at`: (timestamp) Profile creation time.
-   `updated_at`: (timestamp) Last update time.

## 3. `module-order.json` (Existing, but relevant for content generation and sidebar)

-   A JSON file that defines the order of the modules in the sidebar.
-   The content generation process will ensure modules are created/updated based on this order.

## 4. `docusaurus.config.ts`

-   Configuration file for Docusaurus, defining `navbar` structure, `sidebarPath`, and other site settings.
