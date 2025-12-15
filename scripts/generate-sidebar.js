// scripts/generate-sidebar.js
// This script will read module-order.json and generate sidebars.js

const fs = require('fs');
const path = require('path');

const MODULE_ORDER_PATH = path.join(__dirname, '..', 'module-order.json');
const SIDEBARS_PATH = path.join(__dirname, '..', 'physical-ai-humanoid-robotics-textbook', 'sidebars.js');

function generateSidebar() {
  try {
    const moduleOrder = JSON.parse(fs.readFileSync(MODULE_ORDER_PATH, 'utf8'));

    const moduleItems = moduleOrder.modules.map(moduleId => `modules/${moduleId}`);

    const sidebars = {
      tutorialSidebar: [
        {
          type: 'category',
          label: 'Modules',
          items: moduleItems,
        },
      ],
    };

    const content = `module.exports = ${JSON.stringify(sidebars, null, 2)};`;
    fs.writeFileSync(SIDEBARS_PATH, content, 'utf8');
    console.log('Successfully generated sidebars.js');
  } catch (error) {
    console.error('Error generating sidebars.js:', error);
  }
}

generateSidebar();
