# Data Model: Textbook Structure

## Module Markdown File

- **`id`**: (string) Unique identifier for the module (e.g., `module-1-introduction`).
- **`title`**: (string) The main title of the module displayed on the page (e.g., "Module 1: Introduction").
- **`sidebar_label`**: (string) The short label for the module displayed in the sidebar (e.g., "Introduction").
- **`content`**: (string, Markdown) The main content of the module, including placeholders.
- **`placeholders`**: (array of strings) A list of placeholders to be included in the content, such as:
  - `RAG Chatbot`
  - `Personalization Button`
  - `Urdu Translation Button`
- **`hooks`**: (array of strings) A list of hooks for Claude Sub-Agents/Skills, such as:
  - `[Generate examples, diagrams, exercises for <Module Name>]`

## `sidebars.js`

- A JavaScript object that defines the structure of the sidebar.
- The top-level key is the sidebar ID (e.g., `tutorialSidebar`).
- The value is an array of items, which can be strings (for individual docs) or objects (for categories).

**Example `sidebars.js` structure**:

```javascript
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Modules',
      items: [
        'modules/module-1-introduction',
        'modules/module-2-ros2',
        'modules/module-3-simulation',
        'modules/module-4-perception',
      ],
    },
  ],
};
```

## `module-order.json`

- A JSON file that defines the order of the modules in the sidebar.
- The script that generates `sidebars.js` will use this file to ensure the correct order.

**Example `module-order.json`**:

```json
{
  "modules": [
    "module-1-introduction",
    "module-2-ros2",
    "module-3-simulation",
    "module-4-perception"
  ]
}
```
