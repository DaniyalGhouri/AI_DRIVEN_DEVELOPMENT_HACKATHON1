// scripts/ingest-content.js
const fs = require('fs');
const path = require('path');
const axios = require('axios'); // Assuming this is installed via npm

const MODULE_PATH = path.join(__dirname, '..', 'physical-ai-humanoid-robotics-textbook', 'docs', 'modules');
const MODULE_ORDER_PATH = path.join(__dirname, '..', 'module-order.json');
const BACKEND_INGEST_URL = "http://localhost:8000/ingest"; // Assuming backend runs locally

async function ingestModuleContent() {
    try {
        const moduleOrder = JSON.parse(fs.readFileSync(MODULE_ORDER_PATH, 'utf8'));
        const filesToIngest = [];

        for (const moduleId of moduleOrder.modules) {
            const filePath = path.join(MODULE_PATH, `${moduleId}.md`);
            if (fs.existsSync(filePath)) {
                const content = fs.readFileSync(filePath, 'utf8');
                filesToIngest.push({
                    file_path: `docs/modules/${moduleId}.md`, // Relative path for metadata
                    content: content,
                    module_id: moduleId,
                    // Add more metadata if available, e.g., chapter_id, section_id
                });
            } else {
                console.warn(`Warning: Module file not found: ${filePath}`);
            }
        }

        if (filesToIngest.length === 0) {
            console.log("No module files found to ingest.");
            return;
        }

        console.log(`Ingesting ${filesToIngest.length} module files...`);
        const response = await axios.post(BACKEND_INGEST_URL, { files: filesToIngest });

        console.log("Ingestion successful:", response.data);
    } catch (error) {
        console.error("Error during content ingestion:", error.response ? error.response.data : error.message);
        process.exit(1);
    }
}

ingestModuleContent();
