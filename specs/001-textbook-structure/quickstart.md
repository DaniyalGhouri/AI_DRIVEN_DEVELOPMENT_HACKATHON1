# Quickstart: Managing Textbook Modules

This guide explains how to add, remove, or reorder modules in the textbook.

## Adding a New Module

1.  Create a new Markdown file in the `docs/` directory (e.g., `docs/new-module.md`).
2.  Add the necessary front-matter to the file: `id`, `title`, and `sidebar_label`.
3.  Add the new module's `id` to the `module-order.json` file in the desired position.
4.  Run the sidebar generation script to update `sidebars.js`:
    ```bash
    node scripts/generate-sidebar.js
    ```

## Removing a Module

1.  Delete the module's Markdown file from the `docs/` directory.
2.  Remove the module's `id` from the `module-order.json` file.
3.  Run the sidebar generation script:
    ```bash
    node scripts/generate-sidebar.js
    ```

## Reordering Modules

1.  Edit the `module-order.json` file and arrange the module `id`s in the desired order.
2.  Run the sidebar generation script:
    ```bash
    node scripts/generate-sidebar.js
    ```
