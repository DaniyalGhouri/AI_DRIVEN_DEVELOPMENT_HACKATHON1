# Navigation Guide

## Structure:
- Home: src/pages/index.tsx (main homepage)
- Modules: docs/modules/*.md
- Chapters: docs/part-*/module-*/chapter-*.md
- Docs index: docs/index.md (newly created)

## Linking Rules:
1. Use relative paths: `./chapter-1-1` for same directory
2. Use `../` to go up one level
3. Use `../../` to go up two levels
4. For Docusaurus links, use absolute paths starting with `/` for internal docs

## Navigation Components:
1. **Sidebar Navigation**: Defined in sidebars.ts with structured categories
2. **Module Pages**: Each module page links to its detailed chapters
3. **Chapter Pages**: Within parts, chapters are organized under modules
4. **Navigation Component**: Use NavigationButtons.js for prev/next navigation

## Quick Reference:
- Module 1: `/modules/module-1-introduction`
- Module 2: `/modules/module-2-ros2`
- Module 3: `/modules/module-3-simulation`
- And so on for all modules

## How to Add Navigation:
To add navigation to any existing document, include at the bottom:

```markdown
## Navigation
- [← Previous](./link-to-previous)
- [Module Overview](../modules/module-1-introduction) 
- [Next →](./link-to-next)
- [Back to Table of Contents](../../index)
```

## Best Practices:
1. Always test navigation links after making changes
2. Use consistent naming for modules and chapters
3. Keep sidebar structure consistent with document structure
4. Use the same terminology throughout the navigation