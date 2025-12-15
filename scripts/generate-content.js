// scripts/generate-content.js
const fs = require('fs');
const path = require('path');
const { GoogleGenerativeAI } = require('@google/generative-ai'); // Assuming this is installed via npm

const GEMINI_API_KEY = process.env.GEMINI_API_KEY; // Ensure API key is set in environment

if (!GEMINI_API_KEY) {
  console.error("Error: GEMINI_API_KEY environment variable is not set.");
  process.exit(1);
}

const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);
const model = genAI.getGenerativeModel({ model: "gemini-pro"});

const MODULE_PATH = path.join(__dirname, '..', 'physical-ai-humanoid-robotics-textbook', 'docs', 'modules');

async function generateModuleContent(moduleId, moduleTitle, sidebarLabel) {
  const filePath = path.join(MODULE_PATH, `${moduleId}.md`);

  const prompt = `You are an expert in Physical AI and Humanoid Robotics. Your task is to generate academically deep and comprehensive content for a Docusaurus textbook module. The module is titled "${moduleTitle}".

The content MUST include the following 13 sections. Do NOT omit any section. Provide actual content for each, not placeholders. Use Markdown format.

1.  **In-depth Theory**: Provide a comprehensive theoretical foundation.
2.  **Detailed Subtopics**: Break down the theory into detailed sub-sections.
3.  **Technical Explanations**: Explain complex technical concepts clearly.
4.  **Hands-on Examples**: Provide practical, code-oriented, or conceptual examples.
5.  **Visual Diagrams (ASCII diagrams)**: Use ASCII art to illustrate concepts or architectures.
6.  **System Workflows**: Describe processes or interactions using flowcharts or step-by-step descriptions.
7.  **Mathematical Formulas**: Include relevant mathematical formulas with explanations.
8.  **Tables and Charts**: Present data or comparisons in tables or simple charts.
9.  **Lab Exercises**: Design practical exercises or mini-projects for students.
10. **Real-world Industrial Case Studies**: Describe how these concepts apply in real industry scenarios.
11. **Key Takeaways**: Summarize the most important points.
12. **Review Questions**: Provide questions to test understanding.
13. **Glossary**: Define key terms used in the module.

---
Module ID: ${moduleId}
Module Title: ${moduleTitle}
Sidebar Label: ${sidebarLabel}
---

Generate the full Markdown content for this module, adhering strictly to the above requirements.`;

  console.log(`Generating content for ${moduleTitle}...`);
  try {
    const result = await model.generateContent(prompt);
    const generatedContent = result.response.text();

    // Add Docusaurus front matter
    const frontMatter = `---
id: ${moduleId}
title: "${moduleTitle}"
sidebar_label: "${sidebarLabel}"
---
`;

    fs.writeFileSync(filePath, frontMatter + generatedContent, 'utf8');
    console.log(`Successfully generated content for ${moduleTitle} at ${filePath}`);
    return true;
  } catch (error) {
    console.error(`Error generating content for ${moduleTitle}:`, error);
    return false;
  }
}

// Export the function to be called programmatically
module.exports = generateModuleContent;
