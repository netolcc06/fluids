<#
.SYNOPSIS
    Builds the fluids_wasm target (core + Embind bindings) with Emscripten
    and drops the generated glue module into web/public/wasm/.

.DESCRIPTION
    Activates the local emsdk environment (expects it at <repo>/.emsdk, per
    the project setup instructions — never installed system-wide), then runs
    `emcmake cmake` to configure wasm/ and `emmake cmake --build` to build
    it. Safe to re-run; it reuses the existing build directory. Output goes
    to web/src/wasm-gen (NOT web/public — Vite's dev server can't serve
    public/ files through its ES module import pipeline).

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\build_wasm.ps1
#>

[CmdletBinding()]
param(
    [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$EmsdkDir = Join-Path $RepoRoot ".emsdk"
$WasmSrcDir = Join-Path $RepoRoot "wasm"
$BuildDir = Join-Path $RepoRoot "build\wasm"

if (-not (Test-Path $EmsdkDir)) {
    throw "emsdk not found at $EmsdkDir. Clone it there first:`n  git clone https://github.com/emscripten-core/emsdk.git `"$EmsdkDir`"`n  & `"$EmsdkDir\emsdk.ps1`" install latest`n  & `"$EmsdkDir\emsdk.ps1`" activate latest"
}

Write-Host "Activating emsdk environment from $EmsdkDir ..." -ForegroundColor Cyan
$emsdkEnvScript = Join-Path $EmsdkDir "emsdk_env.ps1"
. $emsdkEnvScript

# emcmake needs a Makefile/Ninja-style generator; this machine has neither
# Visual Studio nor MinGW make, so Ninja is vendored locally (via `pip
# install --target` into .emsdk/ninja-pkg, done once during setup) rather
# than installed system-wide.
$NinjaDir = Join-Path $EmsdkDir "ninja-pkg\bin"
if (Test-Path $NinjaDir) {
    $env:Path = "$NinjaDir;$env:Path"
}

Write-Host "Configuring wasm/ with emcmake ..." -ForegroundColor Cyan
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
emcmake cmake -S $WasmSrcDir -B $BuildDir "-DCMAKE_BUILD_TYPE=$Config"
if ($LASTEXITCODE -ne 0) { throw "emcmake cmake configure failed (exit $LASTEXITCODE)." }

Write-Host "Building fluids_wasm with emmake ..." -ForegroundColor Cyan
emmake cmake --build $BuildDir --config $Config
if ($LASTEXITCODE -ne 0) { throw "emmake cmake --build failed (exit $LASTEXITCODE)." }

$OutDir = Join-Path $RepoRoot "web\src\wasm-gen"
Write-Host "Build complete. Output in $OutDir" -ForegroundColor Green
Get-ChildItem $OutDir -ErrorAction SilentlyContinue | Format-Table Name, Length
