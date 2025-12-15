# Tasks: Textbook Structure Generation

**Input**: Design documents from `specs/001-textbook-structure/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `scripts/` directory
- [X] T002 [P] Create `module-order.json` file
- [X] T003 [P] Create `scripts/generate-sidebar.js` file

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

_(No foundational tasks are required for this feature)_

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Textbook Module Navigation (Priority: P1) 🎯 MVP

**Goal**: A student can browse the structured textbook, navigate between the 4 defined modules, and view their introductory content.

**Independent Test**: Can be fully tested by navigating to each module's main page and verifying its presence and basic structure.

### Implementation for User Story 1

- [X] T004 [US1] Create `docs/module-1-introduction.md` with front-matter
- [X] T005 [P] [US1] Create `docs/module-2-ros2.md` with front-matter
- [X] T006 [P] [US1] Create `docs/module-3-simulation.md` with front-matter
- [X] T007 [P] [US1] Create `docs/module-4-perception.md` with front-matter
- [X] T008 [P] [US1] Create `docs/module-0-constraints.md`
- [X] T009 [US1] Implement `scripts/generate-sidebar.js` to read `module-order.json` and generate `sidebars.js`
- [X] T010 [US1] Run `node scripts/generate-sidebar.js` to generate `physical-ai-humanoid-robotics-textbook/sidebars.js`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interactive Learning Placeholders (Priority: P2)

**Goal**: A student can see placeholders for interactive features like the RAG Chatbot, Personalization Button, Urdu Translation Button, and Claude Sub-Agent hooks within each module.

**Independent Test**: Can be tested by visually inspecting each module page to ensure the presence of the specified placeholders/hooks.

### Implementation for User Story 2

- [X] T011 [US2] Add placeholders and hooks to `docs/module-1-introduction.md`
- [X] T012 [P] [US2] Add placeholders and hooks to `docs/module-2-ros2.md`
- [X] T013 [P] [US2] Add placeholders and hooks to `docs/module-3-simulation.md`
- [X] T014 [P] [US2] Add placeholders and hooks to `docs/module-4-perception.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T015 Run `npm run build` in `physical-ai-humanoid-robotics-textbook/` to verify site builds without errors
- [X] T016 Run `npm start` in `physical-ai-humanoid-robotics-textbook/` to visually inspect the site and navigation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Stories (Phase 3-4)**: All depend on Setup phase completion
- **Polish (Final Phase)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup (Phase 1) - No dependencies on other stories
- **User Story 2 (P2)**: Depends on User Story 1 completion.

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 3: User Story 1
3. **STOP and VALIDATE**: Test User Story 1 independently
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
