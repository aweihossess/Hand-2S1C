param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "force_sensor_monitor.py"
$pythonCandidates = @()

$pythonCommand = Get-Command python -ErrorAction SilentlyContinue
if ($pythonCommand) {
    $pythonCandidates += $pythonCommand.Source
}

$pyCommand = Get-Command py -ErrorAction SilentlyContinue
if ($pyCommand) {
    $pythonCandidates += $pyCommand.Source
}

$bundledPython = "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$pythonCandidates += $bundledPython

foreach ($python in $pythonCandidates) {
    if ($python -and (Test-Path -LiteralPath $python)) {
        & $python $scriptPath --port $Port
        exit $LASTEXITCODE
    }
}

Write-Error "No Python executable was found. Install Python or edit this script to point at python.exe."
