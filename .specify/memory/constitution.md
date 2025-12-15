<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Modified principles:
  - Principle 2: Objectives (Expanded scope of content and RAG)
  - Principle 3: Core Components (Updated to include Advanced RAG details)
- Added sections:
  - Principle 6: Textbook Content Standards
  - Principle 7: RAG Chatbot System Standards
- Removed sections: none
- Templates requiring updates: ⚠ pending
- Follow-up TODOs: none
-->
# Physical AI & Humanoid Robotics Project Constitution

**Version**: 1.1.0
**Ratification Date**: TODO(RATIFICATION_DATE): Determine the original adoption date of this constitution.
**Last Amended**: 2025-12-09

## 1. Governance

This constitution is the supreme governing document for the "Physical AI & Humanoid Robotics" project. All development, content, and contributions MUST align with its principles.

### 1.1. Versioning

This document follows semantic versioning (Major.Minor.Patch):
- **MAJOR**: Reserved for backward-incompatible changes, such as removing a core principle.
- **MINOR**: For adding new principles or significant, non-breaking changes.
- **PATCH**: For minor clarifications, typo fixes, or stylistic improvements.

### 1.2. Amendments

Amendments require a formal review process. Changes MUST be documented in the Sync Impact Report at the top of this file.

## 2. Principles

### 2.1. Principle 1: Purpose
Teach Physical AI & Humanoid Robotics course with interactive AI-enabled learning.

### 2.2. Principle 2: Objectives
1. Create Docusaurus-based textbook with fully detailed and academically deep modules.
2. Embed advanced RAG Chatbot for interactive Q&A and selected-text retrieval.
3. Enable personalization based on user background.
4. Enable Urdu translation of content.
5. Use Claude Code Sub-Agents and Skills for Reusable Intelligence.

### 2.3. Principle 3: Core Components
- Docusaurus Book (with fully detailed modules)
- Advanced RAG Chatbot (FastAPI, Qdrant, Neon, OpenAI Agents/ChatKit)
- Better-Auth for signup/signin
- Claude Code Sub-Agents + Skills
- Backend APIs (FastAPI)
- Databases (Neon + Qdrant)

### 2.4. Principle 4: Constraints
1. Must follow hackathon course content exactly.
2. Use only open-source or compatible licenses.
3. Content must be technically accurate.
4. Docusaurus framework must be used.

### 2.5. Principle 5: Workflows
- Content Generation Workflow (Claude Sub-Agents)
- Student Learning Workflow (RAG + personalization)
- Authentication & Personalization Workflow (Better-Auth)

### 2.6. Principle 6: Textbook Content Standards
- All module content MUST be generated with full academic depth and detail.
- Content MUST include in-depth theory, detailed subtopics, technical explanations, hands-on examples, visual diagrams (ASCII), system workflows, mathematical formulas, tables/charts, lab exercises, real-world industrial case studies, key takeaways, review questions, and glossary.
- NO placeholder lines (e.g., "[Generate detailed examples…]") are permitted in the final content.

### 2.7. Principle 7: RAG Chatbot System Standards
- The RAG chatbot MUST be fully integrated into each Docusaurus page.
- It MUST support full book retrieval and selected-text retrieval.
- The backend MUST use FastAPI, OpenAI Agents/ChatKit SDK, Qdrant Cloud, and Neon Serverless Postgres.
- The frontend MUST include a floating widget, side-panel chatbot, "Ask questions" buttons, contextual breadcrumb, personalized "My Notes + Chat Logs" panel, Urdu translation support, and a personalization button.
- The chatbot logic MUST detect selected text for retrieval, use top-k chunk retrieval with LLM re-ranking, and return citations.