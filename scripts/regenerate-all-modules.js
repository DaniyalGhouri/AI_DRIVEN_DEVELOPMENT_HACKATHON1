// scripts/regenerate-all-modules.js
const generateModuleContent = require('./generate-content');
const fs = require('fs');
const path = require('path');

const MODULE_ORDER_PATH = path.join(__dirname, '..', 'module-order.json');

async function regenerateAllModules() {
  try {
    const moduleOrder = JSON.parse(fs.readFileSync(MODULE_ORDER_PATH, 'utf8'));
    const modules = moduleOrder.modules;

    for (const moduleId of modules) {
      // Assuming a simple mapping for title and sidebar label for now
      // In a more complex scenario, these might come from a config or another prompt
      let moduleTitle = moduleId.replace(/-/g, ' ').split(' ').map(word => word.charAt(0).toUpperCase() + word.slice(1)).join(' ');
      let sidebarLabel = moduleTitle;

      if (moduleId === "module-1-introduction") {
        moduleTitle = "Module 1: Introduction to Physical AI and Humanoid Robotics";
        sidebarLabel = "Module 1: Introduction";
      } else if (moduleId === "module-2-ros2") {
        moduleTitle = "Module 2: Robotic Operating System (ROS 2)";
        sidebarLabel = "Module 2: ROS 2";
      } else if (moduleId === "module-3-simulation") {
        moduleTitle = "Module 3: Simulation and Digital Twins";
        sidebarLabel = "Module 3: Simulation";
      } else if (moduleId === "module-4-perception") {
        moduleTitle = "Module 4: Perception and Sensor Fusion";
        sidebarLabel = "Module 4: Perception";
      }

      await generateModuleContent(moduleId, moduleTitle, sidebarLabel);
    }
    console.log("All modules regenerated successfully.");
  } catch (error) {
    console.error("Error regenerating all modules:", error);
    process.exit(1);
  }
}

regenerateAllModules();
