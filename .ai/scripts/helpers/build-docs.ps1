# Build Documentation Script
# Usage: .\.ai\scripts\helpers\build-docs.ps1 [-Clean]

param (
    [switch]$Clean
)

$rootDir = git rev-parse --show-toplevel 2>$null
if ($LASTEXITCODE -ne 0) { $rootDir = Get-Location }
Set-Location $rootDir

Write-Host "--- Building Documentation ---" -ForegroundColor Cyan

$distDir = Join-Path $rootDir "dist"
if (-not (Test-Path $distDir)) { New-Item -ItemType Directory -Path $distDir | Out-Null }

# Build LaTeX documents
$texFiles = Get-ChildItem -Path "docs" -Filter "*.tex" -Recurse -ErrorAction SilentlyContinue

if (-not $texFiles) {
    Write-Host "No .tex files found in docs/." -ForegroundColor Yellow
    exit 0
}

foreach ($file in $texFiles) {
    Write-Host "Building $($file.Name)..." -ForegroundColor Yellow
    $outDir = Join-Path $file.DirectoryName "build"

    latexmk -pdf -interaction=nonstopmode -shell-escape "-outdir=$outDir" $file.FullName

    if ($LASTEXITCODE -eq 0) {
        $pdfName = $file.BaseName + ".pdf"
        $generatedPdf = Join-Path $outDir $pdfName
        if (Test-Path $generatedPdf) {
            Copy-Item $generatedPdf -Destination (Join-Path $distDir $pdfName) -Force
            Write-Host "Done: $pdfName -> dist/" -ForegroundColor Green
        }
    } else {
        Write-Host "FAILED: $($file.Name)" -ForegroundColor Red
    }
}

if ($Clean) {
    Write-Host "Cleaning build directories..." -ForegroundColor Gray
    Get-ChildItem -Path "docs" -Directory -Filter "build" -Recurse | Remove-Item -Recurse -Force
}

Write-Host "--- Build Finished ---" -ForegroundColor Cyan
