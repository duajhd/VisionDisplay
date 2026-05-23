$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$QtRoot = "D:\Qt\6.8.3\msvc2022_64"
$QtBin = Join-Path $QtRoot "bin"
$QtPlugins = Join-Path $QtRoot "plugins"
$QtQml = Join-Path $QtRoot "qml"
$BuildQml = Join-Path $RepoRoot "build_asan\qml"
$Exe = Join-Path $RepoRoot "build_asan\appVisionDisplay.exe"
$WorkDir = Split-Path -Parent $Exe
$LogDir = Join-Path $RepoRoot "asan_logs"
$StdoutLog = Join-Path $LogDir "asan_stdout.log"
$StderrLog = Join-Path $LogDir "asan_stderr.log"

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
Remove-Item -LiteralPath $StdoutLog, $StderrLog -ErrorAction SilentlyContinue

Write-Host "===================================="
Write-Host "run_asan_app.ps1"
Write-Host "RepoRoot: $RepoRoot"
Write-Host "QtRoot:   $QtRoot"
Write-Host "QtBin:    $QtBin"
Write-Host "QtPlugin: $QtPlugins"
Write-Host "QtQml:    $QtQml"
Write-Host "BuildQml: $BuildQml"
Write-Host "Exe:      $Exe"
Write-Host "WorkDir:  $WorkDir"
Write-Host "Stdout:   $StdoutLog"
Write-Host "Stderr:   $StderrLog"
Write-Host "===================================="

if (-not (Test-Path -LiteralPath $QtBin)) {
    throw "Qt bin path not found: $QtBin"
}
if (-not (Test-Path -LiteralPath (Join-Path $QtBin "Qt6Qmld.dll"))) {
    throw "Qt6Qmld.dll not found in: $QtBin"
}
if (-not (Test-Path -LiteralPath $QtPlugins)) {
    throw "Qt plugins path not found: $QtPlugins"
}
if (-not (Test-Path -LiteralPath $QtQml)) {
    throw "Qt qml path not found: $QtQml"
}
if (-not (Test-Path -LiteralPath $Exe)) {
    throw "ASan app not found: $Exe. Build it first with: cmake --build build_asan"
}

$PathEntries = @($QtBin)
$VcpkgDebugBin = Join-Path $RepoRoot "vcpkg_installed\x64-windows\debug\bin"
if (Test-Path -LiteralPath $VcpkgDebugBin) {
    $PathEntries += $VcpkgDebugBin
}
$env:PATH = ($PathEntries -join ";") + ";" + $env:PATH

$env:QT_PLUGIN_PATH = $QtPlugins + ";" + $env:QT_PLUGIN_PATH

$QmlImportEntries = @($QtQml)
if (Test-Path -LiteralPath $BuildQml) {
    $QmlImportEntries += $BuildQml
}
$env:QML2_IMPORT_PATH = ($QmlImportEntries -join ";") + ";" + $env:QML2_IMPORT_PATH

$env:QT_DEBUG_PLUGINS = "1"
$env:QML_IMPORT_TRACE = "1"
$env:QT_LOGGING_TO_CONSOLE = "1"

Write-Host ""
Write-Host "Starting app with Start-Process -PassThru -Wait..."
Write-Host "Command: $Exe"
Write-Host ""

$Process = Start-Process `
    -FilePath $Exe `
    -WorkingDirectory $WorkDir `
    -PassThru `
    -Wait `
    -RedirectStandardOutput $StdoutLog `
    -RedirectStandardError $StderrLog

Write-Host ""
Write-Host "App exited."
Write-Host "ExitCode: $($Process.ExitCode)"

Write-Host ""
Write-Host "===== STDERR last 120 lines ====="
if (Test-Path -LiteralPath $StderrLog) {
    Get-Content -LiteralPath $StderrLog -Tail 120
} else {
    Write-Host "(stderr log not created)"
}

Write-Host ""
Write-Host "===== STDOUT last 120 lines ====="
if (Test-Path -LiteralPath $StdoutLog) {
    Get-Content -LiteralPath $StdoutLog -Tail 120
} else {
    Write-Host "(stdout log not created)"
}
