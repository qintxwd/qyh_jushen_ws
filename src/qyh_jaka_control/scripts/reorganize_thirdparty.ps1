# Reorganize thirdparty directory structure
# Move common include and lib to arm64 subdirectory

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ThirdpartyDir = Join-Path (Split-Path -Parent $ScriptDir) "thirdparty"

Set-Location $ThirdpartyDir

Write-Host "Current directory: $(Get-Location)" -ForegroundColor Cyan
Write-Host ""
Write-Host "Reorganizing thirdparty directory structure..." -ForegroundColor Yellow
Write-Host ""

# 1. Check if already reorganized
if (Test-Path "arm64\lib\libjakaAPI_2_3_0_13.so") {
    Write-Host "arm64 directory already contains library files, skipping reorganization" -ForegroundColor Green
    exit 0
}

# 2. Move include to arm64
if ((Test-Path "include") -and !(Test-Path "arm64\include")) {
    Write-Host "Moving include\ to arm64\include\" -ForegroundColor Cyan
    Move-Item -Path "include" -Destination "arm64\include"
} else {
    Write-Host "Skipping include move (already exists or source not found)" -ForegroundColor Gray
}

# 3. Move lib to arm64
if ((Test-Path "lib") -and !(Test-Path "arm64\lib")) {
    Write-Host "Moving lib\ to arm64\lib\" -ForegroundColor Cyan
    Move-Item -Path "lib" -Destination "arm64\lib"
} else {
    Write-Host "Skipping lib move (already exists or source not found)" -ForegroundColor Gray
}

Write-Host ""
Write-Host "Reorganization complete!" -ForegroundColor Green
Write-Host ""
Write-Host "Directory structure:" -ForegroundColor Cyan
Write-Host "thirdparty/"
Write-Host "├── arm64/"
Write-Host "│   ├── include/       (ARM64 headers)"
Write-Host "│   └── lib/           (ARM64 library: libjakaAPI_2_3_0_13.so)"
Write-Host "└── x64/"
Write-Host "    ├── include/       (x64 headers)"
Write-Host "    └── lib/           (x64 library: libjakaAPI_2_3_3.so)"
Write-Host ""
Write-Host "Please ensure architecture-specific dependencies are installed before compiling" -ForegroundColor Yellow
