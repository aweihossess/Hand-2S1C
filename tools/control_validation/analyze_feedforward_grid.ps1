param(
    [Parameter(Mandatory=$true)][string]$CsvPath,
    [string]$Python = "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe",
    [string]$FitTarget = "now",
    [double]$MaxLoadAbs = -1,
    [double]$MaxBiasAbs = -1
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $CsvPath)) {
    throw "CSV not found: $CsvPath"
}
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python not found: $Python"
}

$base = [System.IO.Path]::Combine(
    [System.IO.Path]::GetDirectoryName((Resolve-Path -LiteralPath $CsvPath)),
    [System.IO.Path]::GetFileNameWithoutExtension($CsvPath))
$timeseries = "${base}_ff_timeseries.csv"
$fitPrefix = "${base}_rawfit_${FitTarget}"

& $Python ".\tools\control_validation\export_feedforward_timeseries.py" $CsvPath --out $timeseries

$fitArgs = @(
    ".\tools\control_validation\fit_feedforward_model.py",
    $CsvPath,
    "--fit-target", $FitTarget,
    "--out-prefix", $fitPrefix
)
if ($MaxLoadAbs -ge 0) {
    $fitArgs += @("--max-load-abs", "$MaxLoadAbs")
}
if ($MaxBiasAbs -ge 0) {
    $fitArgs += @("--max-bias-abs", "$MaxBiasAbs")
}
& $Python @fitArgs

Write-Host "Timeseries CSV: $timeseries"
Write-Host "Fit report: ${fitPrefix}_report.txt"
Write-Host "Fit coefficients: ${fitPrefix}_coeff.csv"
