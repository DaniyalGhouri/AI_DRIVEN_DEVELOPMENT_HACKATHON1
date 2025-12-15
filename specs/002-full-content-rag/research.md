# Research: Advanced RAG Chatbot Integration

## 1. OpenAI Agents/ChatKit SDK Integration with Custom RAG

**Decision**: Integrate OpenAI Agents/ChatKit SDK as the final response generation layer, feeding it the context retrieved and re-ranked from Qdrant.

**Rationale**: This leverages the advanced conversational capabilities of OpenAI Agents while maintaining control over the retrieval process for domain-specific knowledge from the textbook. ChatKit SDK will simplify frontend integration.

**Implementation Details**:
- The FastAPI backend will orchestrate:
    1. Receiving user query (and selected text, if any).
    2. Performing Qdrant hybrid search.
    3. Re-ranking top-k chunks using a smaller LLM or a heuristic.
    4. Constructing a prompt with the user query and the retrieved context.
    5. Sending this contextualized prompt to the OpenAI Agent via ChatKit SDK.
    6. Streaming the response back to the frontend.
- Citations: Metadata (module ID, section) will be attached to chunks. The agent's response will need to be parsed to identify which parts of the response are based on which retrieved chunks, to map back to original module paths for citation display. This might require fine-tuning the agent's prompt to encourage citation-like output or a post-processing step on the agent's response. This will be a key area for iterative development.

## 2. Authentication Mechanism

**Decision**: Implement JWT-based authentication for the FastAPI backend.

**Rationale**: JWTs are stateless, scalable, and a common pattern for API authentication. This provides flexibility for future frontend integration (e.g., with a Docusaurus plugin for user accounts or integration with Clerk/Auth.js if needed later, as JWTs are compatible). Clerk/Auth.js are frontend-focused solutions, and implementing JWT at the backend level provides a solid API foundation.

**Implementation Details**:
- FastAPI will use standard OAuth2/JWT Bearer token scheme.
- User registration/login endpoints will be exposed (if not using an external IdP like Clerk). Assuming for now, local user management with Neon.

## 3. Urdu Translation Mechanism

**Decision**: Proxy Urdu translation requests through the FastAPI backend.

**Rationale**: Centralizing API calls (especially to OpenAI) through the backend allows for better control over API keys, rate limiting, logging, and potential caching. It also simplifies frontend logic.

**Implementation Details**:
- Frontend will send text to a backend endpoint (e.g., `POST /translate`).
- Backend will call OpenAI GPT model (e.g., `gpt-3.5-turbo` or `gpt-4` for better quality) with a prompt like "Translate the following English text to Urdu: [text]".
- The translated text will be returned to the frontend.

## 4. Personalization Recommendations Logic

**Decision**: Initially, provide basic module-based recommendations; further personalization criteria will be a future enhancement.

**Rationale**: Building a robust personalization engine is a complex task involving user behavior tracking, content tagging, and sophisticated algorithms. For the MVP, simpler recommendations (e.g., "users who read Module 1 also found Module 3 useful") or explicit recommendations from the content generation agent based on current module context can be a starting point. Detailed personalization based on learning style or background is a larger feature.

**Implementation Details**:
- The content generation agent will embed "custom learning recommendations" at strategic points within modules.
- The chatbot, when on a specific module page, can suggest related modules or resources based on pre-defined links/metadata.
- No complex user behavior tracking will be implemented in the initial phase.
