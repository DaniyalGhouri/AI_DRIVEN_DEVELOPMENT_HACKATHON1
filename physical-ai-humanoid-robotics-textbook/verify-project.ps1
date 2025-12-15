# PHASE 7: Verification Script

# This script checks the integrity and completeness of the Docusaurus project.
# It verifies the file structure, sidebar configuration, and key homepage links.

$ErrorActionPreference = "SilentlyContinue"
$hasErrors = $false

# Define base path
$basePath = "./physical-ai-humanoid-robotics-textbook"
$docsPath = "$basePath/docs"

Write-Host "---"
Write-Host "PHASE 7: RUNNING PROJECT VERIFICATION SCRIPT"
Write-Host "---"

# 1. Textbook Structure Verification
Write-Host "1. Verifying textbook file structure..."
$expectedStructure = @(
    "$docsPath/modules/_category_.json",
    "$docsPath/modules/module-1-introduction.md",
    "$docsPath/modules/module-2-ros2.md",
    "$docsPath/modules/module-3-simulation.md",
    "$docsPath/modules/module-4-perception.md",
    "$docsPath/modules/module-5-control.md",
    "$docsPath/modules/module-6-ai.md",
    "$docsPath/modules/module-7-hardware.md",
    "$docsPath/modules/module-8-integration.md",
    "$docsPath/modules/module-9-capstone.md",
    "$docsPath/modules/module-10-appendix.md"
)

foreach ($file in $expectedStructure) {
    if (-not (Test-Path $file)) {
        Write-Host "  [FAIL] Missing file: $file" -ForegroundColor Red
        $hasErrors = $true
    }
}

if (-not $hasErrors) {
    Write-Host "  [PASS] All expected files and directories are present." -ForegroundColor Green
}
Write-Host ""


# 2. Sidebar Hierarchy Validation
Write-Host "2. Validating sidebar hierarchy..."
$sidebarPath = "$basePath/sidebars.ts"
if (-not (Test-Path $sidebarPath)) {
    Write-Host "  [FAIL] sidebars.ts file not found!" -ForegroundColor Red
    $hasErrors = $true
} else {
    $sidebarContent = Get-Content $sidebarPath -Raw
    # Extract all 'id' fields from the sidebar file
    $sidebarIds = $sidebarContent | Select-String -Pattern "id: '([^']*)'" -AllMatches | ForEach-Object { $_.Matches.Groups[1].Value }

    $sidebarOk = $true
    foreach ($id in $sidebarIds) {
        $expectedFilePath = "$docsPath/$id.md"
        if (-not (Test-Path $expectedFilePath)) {
            Write-Host "  [FAIL] Sidebar ID '$id' points to a non-existent file: $expectedFilePath" -ForegroundColor Red
            $hasErrors = $true
            $sidebarOk = $false
        }
    }
    if ($sidebarOk) {
        Write-Host "  [PASS] All sidebar entries correspond to existing files." -ForegroundColor Green
    }
}
Write-Host ""


# 3. Homepage Load Configuration
Write-Host "3. Verifying homepage configuration..."
$homepagePath = "$basePath/src/pages/index.tsx"
$homepageContent = Get-Content $homepagePath -Raw

# Check for dynamic title
if ($homepageContent -notmatch '<h1[^>]*>{siteConfig.title}</h1>') {
    Write-Host "  [FAIL] Homepage title does not use {siteConfig.title}." -ForegroundColor Red
    $hasErrors = $true
}

# Check for "Start Reading" button link
if ($homepageContent -notmatch 'to="/modules/module-1-introduction"') {
    Write-Host "  [FAIL] Homepage 'Start Reading' button has an incorrect link. Expected 'to=\"/modules/module-1-introduction\"'." -ForegroundColor Red
    $hasErrors = $true
} 

if (-not $hasErrors) {
    Write-Host "  [PASS] Homepage is configured correctly." -ForegroundColor Green
}
Write-Host ""


# 4. Final Verification Summary
Write-Host "---"
Write-Host "VERIFICATION SUMMARY"
Write-Host "---"
if ($hasErrors) {
    Write-Host "Project verification FAILED. Please review the errors above." -ForegroundColor Red
} else {
    Write-Host "All verification checks passed successfully!" -ForegroundColor Green
    Write-Host "The project structure and configuration appear to be correct."
}
Write-Host ""
Write-Host "To start the development server and verify the site manually, run the following command:"
Write-Host "cd ./physical-ai-humanoid-robotics-textbook && npm start"
