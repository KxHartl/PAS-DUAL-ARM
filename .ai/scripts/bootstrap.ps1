<#
.SYNOPSIS
    Bootstrap a new LiteRealm project.
.EXAMPLE
    .\.ai\scripts\bootstrap.ps1 -Name "Autonomni_Vilicari"
    .\.ai\scripts\bootstrap.ps1 -Name "Seminar_MEV" -Rag cloud
#>

param (
    [Parameter(Mandatory = $true)]
    [string]$Name,

    [ValidateSet("none", "cloud", "local")]
    [string]$Rag = "none",

    [ValidateSet("none", "global")]
    [string]$Brain = "global"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
if (-not $root) { $root = Get-Location }

Write-Host "`n=== LiteRealm Bootstrap ===" -ForegroundColor Cyan
Write-Host "Projekt: $Name"
Write-Host "RAG:     $Rag"
Write-Host "Brain:   $Brain"
Write-Host ""

# --- 1. Set project name in config files ---
Write-Host "[1/5] Postavljam naziv projekta..." -ForegroundColor Yellow

$stateFile = Join-Path $root "STATE.md"
$yamlFile = Join-Path $root ".ai\config\project.yaml"

if (Test-Path $stateFile) {
    (Get-Content $stateFile -Raw) -replace '_TBD_', $Name | Set-Content $stateFile -NoNewline
}
if (Test-Path $yamlFile) {
    (Get-Content $yamlFile -Raw) -replace '"TBD"', "`"$Name`"" | Set-Content $yamlFile -NoNewline
}

Write-Host "  Naziv '$Name' upisan u STATE.md i project.yaml." -ForegroundColor Green

# --- 2. Create directory structure ---
Write-Host "[2/5] Kreiram direktorije..." -ForegroundColor Yellow

$dirs = @("docs", "src", "dist", "data\raw", "data\processed")
if ($Rag -ne "none") {
    $dirs += "data\rag\sources"
}

foreach ($d in $dirs) {
    $path = Join-Path $root $d
    if (-not (Test-Path $path)) {
        New-Item -ItemType Directory -Path $path -Force | Out-Null
    }
}
Write-Host "  Direktoriji kreirani." -ForegroundColor Green

# --- 3. Setup .env ---
Write-Host "[3/5] Konfiguriram .env..." -ForegroundColor Yellow

$envFile = Join-Path $root ".env"
$envExample = Join-Path $root ".env.example"

if (-not (Test-Path $envFile) -and (Test-Path $envExample)) {
    Copy-Item $envExample $envFile
    Write-Host "  .env kreiran iz .env.example." -ForegroundColor Green

    if ($Brain -eq "global") {
        $brainPath = Join-Path $env:USERPROFILE ".agentbrain"
        if (-not (Test-Path $brainPath)) {
            New-Item -ItemType Directory -Path $brainPath -Force | Out-Null
            Write-Host "  Global Brain direktorij kreiran: $brainPath" -ForegroundColor Green
        } else {
            Write-Host "  Global Brain postoji: $brainPath" -ForegroundColor Green
        }
    }
} else {
    Write-Host "  .env vec postoji, preskačem." -ForegroundColor Gray
}

# --- 4. Python venv + dependencies ---
Write-Host "[4/5] Python okruzenje..." -ForegroundColor Yellow

$pythonCmd = $null
foreach ($cmd in @("python3", "python")) {
    try {
        $ver = & $cmd --version 2>&1
        if ($ver -match "Python 3\.") {
            $pythonCmd = $cmd
            break
        }
    } catch {}
}

if ($pythonCmd) {
    $venvPath = Join-Path $root ".venv"
    if (-not (Test-Path $venvPath)) {
        Write-Host "  Kreiram virtualnu okolinu..." -ForegroundColor Gray
        & $pythonCmd -m venv $venvPath
    }

    $pipCmd = Join-Path $venvPath "Scripts\pip.exe"
    if (Test-Path $pipCmd) {
        $oldPreference = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        try {
            & $pipCmd install -q python-dotenv

            if ($Rag -eq "cloud") {
                Write-Host "  Instaliram RAG Cloud pakete..." -ForegroundColor Gray
                & $pipCmd install -q langchain langchain-community chromadb pypdf google-generativeai
            } elseif ($Rag -eq "local") {
                Write-Host "  Instaliram RAG Local pakete..." -ForegroundColor Gray
                & $pipCmd install -q langchain langchain-community chromadb pypdf sentence-transformers
            }
        }
        finally {
            $ErrorActionPreference = $oldPreference
        }
    }
    Write-Host "  Python OK ($($pythonCmd))." -ForegroundColor Green
} else {
    Write-Host "  Python nije pronadjen — preskačem venv setup." -ForegroundColor Yellow
}

# --- 5. Check LaTeX ---
Write-Host "[5/5] Provjeravam LaTeX..." -ForegroundColor Yellow

$latexmk = Get-Command latexmk -ErrorAction SilentlyContinue
if ($latexmk) {
    Write-Host "  latexmk pronadjen." -ForegroundColor Green
} else {
    Write-Host "  latexmk NIJE pronadjen — instaliraj MiKTeX ili TeX Live za kompilaciju PDF-a." -ForegroundColor Red
}

# --- Done ---
Write-Host "`n=== Bootstrap zavrsen ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "Sljedeci koraci:" -ForegroundColor White
Write-Host "  1. Popuni STATE.md s detaljima o radu"
Write-Host "  2. Kopiraj LaTeX predlozak iz .ai/templates/ u docs/"
Write-Host "  3. Pokreni AI agenta i reci mu sto treba napisati"
if ($Rag -ne "none") {
    Write-Host "  4. Stavi PDF izvore u data/rag/sources/ za RAG citiranje"
}
Write-Host ""
