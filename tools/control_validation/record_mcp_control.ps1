param(
    [Parameter(Mandatory=$true)][string]$Port,
    [int]$Baud = 921600,
    [double]$OpenDelay = 3.0,
    [string]$Output = "",
    [double]$Duration = 0,
    [string]$Init = "text;zero;start;degree;j0 0;j1 0;j2 0;j3 0",
    [int]$StepJoint = -1,
    [double]$StepAmp = 5.0,
    [string]$StepTargets = "",
    [double]$StepHold = 90.0,
    [double]$StepSettleTolerance = 2.0,
    [double]$StepSettleTimeout = 30.0,
    [double]$StepBaselineHold = 1.0,
    [double]$StepAfterReachedHold = 5.0,
    [int]$SineJoint = -1,
    [double]$SineOffset = 0.0,
    [double]$SineAmp = 5.0,
    [double]$SineFreq = 0.05,
    [double]$SineDuration = 60.0,
    [double]$SineInterval = 0.1,
    [switch]$PidZeroThenFeedforwardStep,
    [switch]$PidZeroThenFeedforwardSine,
    [switch]$EmpiricalCollect,
    [switch]$FeedforwardCollect,
    [switch]$FeedforwardGrid,
    [switch]$CollectServoZeroFirst,
    [int]$CollectJoint = 3,
    [double]$CollectStart = 0.0,
    [double]$CollectEnd = 12.0,
    [double]$CollectStep = 1.0,
    [double]$CollectHold = 3.0,
    [double]$CollectAbortAbsActual = 0.0,
    [switch]$CollectReverse,
    [switch]$CollectAllJoints,
    [string]$CollectRanges = "0:-20:30:1;1:-20:80:1;2:0:88:1;3:0:93:1",
    [string]$GridJ0 = "-10,0,10",
    [string]$GridJ1 = "0,5,10",
    [string]$GridJ2 = "0,5,10",
    [string]$GridJ3 = "0,5,10",
    [double]$GridHold = 2.0,
    [int]$GridRepeat = 1,
    [switch]$GridSerpentine,
    [switch]$StopOnExit,
    [switch]$EchoRaw
)

if ($PidZeroThenFeedforwardStep) {
    $Init = "text;ff off;tension on;start;degree;j0 0;j1 0;j2 0;j3 0"
    if ($StepJoint -lt 0 -and [string]::IsNullOrWhiteSpace($StepTargets)) { $StepJoint = 3 }
}
if ($PidZeroThenFeedforwardSine) {
    $Init = "text;ff off;tension on;start;degree;j0 0;j1 0;j2 0;j3 0"
}
if ($EmpiricalCollect) {
    $FeedforwardCollect = $true
    if (-not $PSBoundParameters.ContainsKey("CollectAllJoints") -and
        -not $PSBoundParameters.ContainsKey("CollectJoint") -and
        -not $PSBoundParameters.ContainsKey("CollectRanges")) {
        $CollectAllJoints = $true
    }
    if (-not $PSBoundParameters.ContainsKey("CollectRanges")) {
        $CollectRanges = "0:-30:30:1;1:-30:30:1;2:-30:30:1;3:-30:30:1"
    }
    if (-not $PSBoundParameters.ContainsKey("CollectHold")) {
        $CollectHold = 2.0
    }
    $Init = "text;ff off;tension off;start;degree;j0 0;j1 0;j2 0;j3 0"
}
if ($FeedforwardCollect -or $FeedforwardGrid) {
    if (-not $EmpiricalCollect) {
        $Init = "text;ff off;tension on;start;degree;j0 0;j1 0;j2 0;j3 0"
    }
}
if ($FeedforwardCollect -and $CollectServoZeroFirst) {
    $collectTensionCommand = if ($EmpiricalCollect) { "tension off" } else { "tension on" }
    $Init = "text;ff off;stop;zero;$collectTensionCommand;start;degree;j0 0;j1 0;j2 0;j3 0"
}

if ($Output -eq "") {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Output = Join-Path "run_data" "mcp_control_$stamp.csv"
}
$outDir = Split-Path -Parent $Output
if ($outDir -ne "") {
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
}

$fieldNames = @("t_wall","t_rel","collect_joint","collect_request_deg","collect_point","collect_point_count","line")
foreach ($j in 0..3) {
    $fieldNames += @("j${j}_target","j${j}_actual","j${j}_error")
}
foreach ($m in 0..4) {
    $fieldNames += @("m${m}_len0","m${m}_len1","m${m}_len2","m${m}_map","m${m}_solver","m${m}_cmd","m${m}_now","m${m}_load","m${m}_cur","m${m}_bias")
}

function Write-SerialCommand($Serial, [string]$Command) {
    if ([string]::IsNullOrWhiteSpace($Command)) { return }
    $Serial.WriteLine($Command.Trim())
    Write-Host ">>> $($Command.Trim())"
}

function Write-SerialCommandList($Serial, [string]$Commands) {
    foreach ($part in $Commands -split ";") {
        $cmd = $part.Trim()
        if ($cmd -ne "") {
            Write-SerialCommand $Serial $cmd
            $lowerCmd = $cmd.ToLowerInvariant()
            if ($lowerCmd -eq "zero") {
                Start-Sleep -Milliseconds 2000
            } elseif ($lowerCmd -eq "start" -or $lowerCmd -eq "enable" -or $lowerCmd -eq "run") {
                Start-Sleep -Milliseconds 300
            } elseif ($lowerCmd -eq "text" -or $lowerCmd -eq "monitor" -or $lowerCmd -eq "quiet") {
                Start-Sleep -Milliseconds 200
            } else {
                Start-Sleep -Milliseconds 30
            }
        }
    }
}

function Parse-DoubleList([string]$Text, [string]$Name) {
    $values = New-Object System.Collections.Generic.List[double]
    foreach ($part in $Text -split ",") {
        $trimmed = $part.Trim()
        if ($trimmed -eq "") { continue }
        $parsed = 0.0
        if (-not [double]::TryParse($trimmed, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$parsed)) {
            throw "Bad $Name value: '$trimmed'"
        }
        $values.Add([double]$parsed)
    }
    if ($values.Count -eq 0) { throw "$Name must contain at least one numeric value." }
    return @($values)
}

function New-GridTargets([double[]]$J0, [double[]]$J1, [double[]]$J2, [double[]]$J3, [int]$Repeat, [bool]$Serpentine) {
    if ($Repeat -lt 1) { throw "GridRepeat must be >= 1." }
    $targets = New-Object System.Collections.Generic.List[object]
    for ($rep = 0; $rep -lt $Repeat; $rep++) {
        for ($i0 = 0; $i0 -lt $J0.Count; $i0++) {
            $j1Vals = $J1
            if ($Serpentine -and (($i0 % 2) -eq 1)) {
                $j1Vals = @($J1 | Sort-Object -Descending)
            }
            foreach ($v1 in $j1Vals) {
                for ($i2 = 0; $i2 -lt $J2.Count; $i2++) {
                    $j3Vals = $J3
                    if ($Serpentine -and (($i2 % 2) -eq 1)) {
                        $j3Vals = @($J3 | Sort-Object -Descending)
                    }
                    foreach ($v3 in $j3Vals) {
                        $targets.Add(@([double]$J0[$i0], [double]$v1, [double]$J2[$i2], [double]$v3))
                    }
                }
            }
        }
    }
    return @($targets)
}

function New-RangeTargets([double]$Start, [double]$End, [double]$Step, [bool]$Reverse) {
    if ($Step -le 0) { throw "Range step must be positive." }
    $targets = New-Object System.Collections.Generic.List[double]
    if ($Start -le $End) {
        for ($v = $Start; $v -le $End + 1.0e-9; $v += $Step) {
            $targets.Add([double]$v)
        }
    } else {
        for ($v = $Start; $v -ge $End - 1.0e-9; $v -= $Step) {
            $targets.Add([double]$v)
        }
    }
    if ($Reverse) {
        for ($i = $targets.Count - 2; $i -ge 0; $i--) {
            $targets.Add($targets[$i])
        }
    }
    return @($targets)
}

function New-CollectRangeMap([string]$RangesText, [double]$FallbackStart, [double]$FallbackEnd, [double]$FallbackStep, [bool]$Reverse) {
    $map = @{}
    if ([string]::IsNullOrWhiteSpace($RangesText)) {
        foreach ($j in 0..3) {
            $map[$j] = New-RangeTargets $FallbackStart $FallbackEnd $FallbackStep $Reverse
        }
        return $map
    }

    foreach ($segment in $RangesText -split ";") {
        $part = $segment.Trim()
        if ($part -eq "") { continue }
        $fields = @($part -split ":")
        if ($fields.Count -ne 4) {
            throw "Bad CollectRanges segment '$part'. Expected joint:min:max:step."
        }
        $joint = 0
        $min = 0.0
        $max = 0.0
        $step = 0.0
        if (-not [int]::TryParse($fields[0], [ref]$joint) -or $joint -lt 0 -or $joint -gt 3) {
            throw "Bad CollectRanges joint in '$part'."
        }
        if (-not [double]::TryParse($fields[1], [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$min) -or
            -not [double]::TryParse($fields[2], [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$max) -or
            -not [double]::TryParse($fields[3], [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$step)) {
            throw "Bad CollectRanges numeric value in '$part'."
        }
        $map[$joint] = New-RangeTargets $min $max $step $Reverse
    }
    foreach ($j in 0..3) {
        if (-not $map.ContainsKey($j)) {
            $map[$j] = New-RangeTargets $FallbackStart $FallbackEnd $FallbackStep $Reverse
        }
    }
    return $map
}

function Get-CollectRangeJoints([string]$RangesText) {
    $joints = New-Object System.Collections.Generic.List[int]
    if ([string]::IsNullOrWhiteSpace($RangesText)) { return @() }
    foreach ($segment in $RangesText -split ";") {
        $part = $segment.Trim()
        if ($part -eq "") { continue }
        $fields = @($part -split ":")
        if ($fields.Count -ne 4) { continue }
        $joint = 0
        if ([int]::TryParse($fields[0], [ref]$joint) -and $joint -ge 0 -and $joint -le 3 -and -not $joints.Contains($joint)) {
            $joints.Add($joint)
        }
    }
    return @($joints)
}

function Parse-ControlLine([string]$Line, [datetime]$StartTime) {
    if ($Line -notmatch "\[MCP5 CTRL\]") { return $null }
    $now = Get-Date
    $row = [ordered]@{
        t_wall = [Math]::Round((New-TimeSpan -Start ([datetime]"1970-01-01Z") -End $now.ToUniversalTime()).TotalSeconds, 6)
        t_rel = [Math]::Round((New-TimeSpan -Start $StartTime -End $now).TotalSeconds, 6)
        collect_joint = ""
        collect_request_deg = ""
        collect_point = ""
        collect_point_count = ""
        line = $Line.Trim()
    }
    foreach ($j in 0..3) {
        $row["j${j}_target"] = ""
        $row["j${j}_actual"] = ""
        $row["j${j}_error"] = ""
    }
    foreach ($m in 0..4) {
        foreach ($name in @("len0","len1","len2","map","solver","cmd","now","load","cur","bias")) {
            $row["m${m}_$name"] = ""
        }
    }

    $jointMatches = [regex]::Matches($Line, "J(\d\d) target=([-0-9.]+) actual=([-0-9.]+)")
    foreach ($jm in $jointMatches) {
        $idx = [int]$jm.Groups[1].Value
        if ($idx -lt 4) {
            $target = [double]$jm.Groups[2].Value
            $actual = [double]$jm.Groups[3].Value
            $row["j${idx}_target"] = $target
            $row["j${idx}_actual"] = $actual
            $row["j${idx}_error"] = $target - $actual
        }
    }

    $motorPattern = "M(\d\d) len=([-0-9.]+)/([-0-9.]+)/([-0-9.]+) map=([-0-9.]+) solver=([-0-9]+) cmd=([-0-9]+) now=([-0-9]+) load=([-0-9]+) cur=([-0-9]+) bias=([-0-9]+)"
    $motorMatches = [regex]::Matches($Line, $motorPattern)
    foreach ($mm in $motorMatches) {
        $idx = [int]$mm.Groups[1].Value
        if ($idx -lt 5) {
            $row["m${idx}_len0"] = [double]$mm.Groups[2].Value
            $row["m${idx}_len1"] = [double]$mm.Groups[3].Value
            $row["m${idx}_len2"] = [double]$mm.Groups[4].Value
            $row["m${idx}_map"] = [double]$mm.Groups[5].Value
            $row["m${idx}_solver"] = [int]$mm.Groups[6].Value
            $row["m${idx}_cmd"] = [int]$mm.Groups[7].Value
            $row["m${idx}_now"] = [int]$mm.Groups[8].Value
            $row["m${idx}_load"] = [int]$mm.Groups[9].Value
            $row["m${idx}_cur"] = [int]$mm.Groups[10].Value
            $row["m${idx}_bias"] = [int]$mm.Groups[11].Value
        }
    }
    return [pscustomobject]$row
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud)
$serial.NewLine = "`n"
$serial.ReadTimeout = 50
$serial.WriteTimeout = 500
$rows = 0
$rawOutput = [System.IO.Path]::ChangeExtension($Output, ".raw.txt")
$start = Get-Date
$scheduledStart = $start.AddSeconds(1)
$scheduledDone = $false
$stepState = "idle"
$stepSettleStart = $null
$stepBaselineUntil = $null
$stepReachedUntil = $null
$ffZeroUntil = $null
$ffResumeUntil = $null
$latestJointActual = @([double]::NaN, [double]::NaN, [double]::NaN, [double]::NaN)
$nextStepTime = $scheduledStart
$sineStart = $null
$nextSineTime = $scheduledStart
$collectTargetMap = @{}
$collectJoints = @()
$collectIndex = 0
$collectJointIndex = 0
$collectNextTime = $scheduledStart
$collectStarted = $false
$collectState = "idle"
$collectSettleStart = $null
$collectResumeUntil = $null
$currentCollectJoint = ""
$currentCollectRequestDeg = ""
$currentCollectPoint = ""
$currentCollectPointCount = ""
$gridTargets = @()
$gridIndex = 0
$gridState = "idle"
$gridSettleStart = $null
$gridResumeUntil = $null
$gridNextTime = $scheduledStart
$deadline = $null
$collectAbortTriggered = $false
if ($Duration -gt 0) { $deadline = $start.AddSeconds($Duration) }

if ($FeedforwardCollect) {
    if ($CollectStep -le 0) { throw "CollectStep must be positive." }
    if ($CollectJoint -lt 0 -or $CollectJoint -gt 3) { throw "CollectJoint must be 0..3." }
    if ($CollectAllJoints) {
        $collectJoints = @(0, 1, 2, 3)
    } elseif ($EmpiricalCollect -and $PSBoundParameters.ContainsKey("CollectRanges") -and -not $PSBoundParameters.ContainsKey("CollectJoint")) {
        $rangeJoints = @(Get-CollectRangeJoints $CollectRanges)
        $collectJoints = if ($rangeJoints.Count -gt 0) { $rangeJoints } else { @($CollectJoint) }
    } else {
        $collectJoints = @($CollectJoint)
    }
    $collectTargetMap = New-CollectRangeMap $CollectRanges $CollectStart $CollectEnd $CollectStep ([bool]$CollectReverse)
    foreach ($joint in $collectJoints) {
        $jointTargets = @($collectTargetMap[[int]$joint])
        if ($jointTargets.Count -eq 0) {
            throw "Collect range for J$joint has no targets."
        }
    }
}

if ($FeedforwardGrid) {
    $gridTargets = New-GridTargets `
        (Parse-DoubleList $GridJ0 "GridJ0") `
        (Parse-DoubleList $GridJ1 "GridJ1") `
        (Parse-DoubleList $GridJ2 "GridJ2") `
        (Parse-DoubleList $GridJ3 "GridJ3") `
        $GridRepeat `
        ([bool]$GridSerpentine)
    if ($GridHold -le 0) { throw "GridHold must be positive." }
}

function Test-JointsNearTargets([double[]]$Actual, [double[]]$Targets, [double]$Tolerance) {
    for ($i = 0; $i -lt 4; $i++) {
        if ([double]::IsNaN($Actual[$i])) { return $false }
        if ([Math]::Abs($Actual[$i] - $Targets[$i]) -gt $Tolerance) { return $false }
    }
    return $true
}

function Format-JointActuals([double[]]$Actual) {
    return ("J0={0:F2} J1={1:F2} J2={2:F2} J3={3:F2}" -f $Actual[0], $Actual[1], $Actual[2], $Actual[3])
}

function Get-StepTargetArray() {
    if (-not [string]::IsNullOrWhiteSpace($StepTargets)) {
        $values = @(Parse-DoubleList $StepTargets "StepTargets")
        if ($values.Count -ne 4) {
            throw "StepTargets must contain exactly four comma-separated values: j0,j1,j2,j3."
        }
        return @([double]$values[0], [double]$values[1], [double]$values[2], [double]$values[3])
    }
    if ($StepJoint -lt 0 -or $StepJoint -gt 3) {
        throw "StepJoint must be 0..3 when StepTargets is not provided."
    }
    $targets = @(0.0, 0.0, 0.0, 0.0)
    $targets[$StepJoint] = $StepAmp
    return $targets
}

function Format-StepTargets([double[]]$Targets) {
    return ("J0={0:F3} J1={1:F3} J2={2:F3} J3={3:F3}" -f $Targets[0], $Targets[1], $Targets[2], $Targets[3])
}

try {
    $serial.Open()
    Write-Host "Recording $Port at $Baud -> $Output"
    Write-Host "Raw text -> $rawOutput"
    if ($OpenDelay -gt 0) {
        Write-Host ("Waiting {0:F1}s after opening serial so the board can finish reset/startup" -f $OpenDelay)
        Start-Sleep -Milliseconds ([int]($OpenDelay * 1000))
        try { $serial.DiscardInBuffer() } catch {}
    }
    Set-Content -LiteralPath $Output -Value (($fieldNames -join ","))
    Set-Content -LiteralPath $rawOutput -Value ""
    if ($Init -ne "") { Write-SerialCommandList $serial $Init }

    while ($true) {
        $now = Get-Date
        if ($deadline -ne $null -and $now -ge $deadline) { break }

        if (-not $scheduledDone -and $now -ge $scheduledStart) {
            if ($StepJoint -ge 0 -or -not [string]::IsNullOrWhiteSpace($StepTargets)) {
                if ($stepState -eq "idle") {
                    Write-Host ("Settling J0/J1/J2/J3 to 0 deg; waiting for all abs(actual) <= {0:F2} deg" -f $StepSettleTolerance)
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $stepSettleStart = $now
                    $nextStepTime = $now.AddSeconds(1)
                    $stepState = "settle"
                } elseif ($stepState -eq "settle") {
                    $zeroTargets = @(0.0, 0.0, 0.0, 0.0)
                    if (Test-JointsNearTargets $latestJointActual $zeroTargets $StepSettleTolerance) {
                        if ($PidZeroThenFeedforwardStep) {
                            Write-Host ("PID zero settled: {0}; stopping control and setting servo software zero" -f (Format-JointActuals $latestJointActual))
                            Write-SerialCommand $serial "stop"
                            Start-Sleep -Milliseconds 300
                            Write-SerialCommand $serial "zero"
                            $ffZeroUntil = (Get-Date).AddMilliseconds(2000)
                            $stepState = "servo_zero"
                        } else {
                            Write-Host ("Settled all joints: {0}; holding baseline {1:F1}s" -f (Format-JointActuals $latestJointActual), $StepBaselineHold)
                            $stepBaselineUntil = $now.AddMilliseconds([int]($StepBaselineHold * 1000))
                            $stepState = "baseline"
                        }
                    } elseif ($stepSettleStart -ne $null -and (New-TimeSpan -Start $stepSettleStart -End $now).TotalSeconds -ge $StepSettleTimeout) {
                        Write-Warning ("Step test aborted: not all joints settled within +/-{0:F2} deg in {1:F1}s. Last actual: {2}" -f $StepSettleTolerance, $StepSettleTimeout, (Format-JointActuals $latestJointActual))
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    } elseif ($now -ge $nextStepTime) {
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $nextStepTime = $now.AddSeconds(1)
                    }
                } elseif ($stepState -eq "servo_zero" -and $now -ge $ffZeroUntil) {
                    Write-Host "Switching to feedforward+PID after servo zero"
                    Write-SerialCommand $serial "ff on"
                    Start-Sleep -Milliseconds 30
                    Write-SerialCommand $serial "tension on"
                    Start-Sleep -Milliseconds 30
                    Write-SerialCommand $serial "start"
                    Start-Sleep -Milliseconds 300
                    Write-SerialCommand $serial "degree"
                    Start-Sleep -Milliseconds 30
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $ffResumeUntil = (Get-Date).AddMilliseconds(1000)
                    $stepState = "ff_resume"
                } elseif ($stepState -eq "ff_resume" -and $now -ge $ffResumeUntil) {
                    Write-Host ("Feedforward baseline active; holding baseline {0:F1}s" -f $StepBaselineHold)
                    $stepBaselineUntil = $now.AddMilliseconds([int]($StepBaselineHold * 1000))
                    $stepState = "baseline"
                } elseif ($stepState -eq "baseline" -and $now -ge $stepBaselineUntil) {
                    $targetAfterStep = Get-StepTargetArray
                    Write-Host ("Step targets: {0}" -f (Format-StepTargets $targetAfterStep))
                    Write-SerialCommand $serial "degree"
                    Start-Sleep -Milliseconds 50
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} {1:F3}" -f $j, $targetAfterStep[$j])
                        Start-Sleep -Milliseconds 30
                    }
                    $nextStepTime = $now.AddMilliseconds([int]($StepHold * 1000))
                    $stepState = "hold"
                }
                if ($stepState -eq "hold") {
                    $targetAfterStep = Get-StepTargetArray
                    if (Test-JointsNearTargets $latestJointActual $targetAfterStep $StepSettleTolerance) {
                        if ($stepReachedUntil -eq $null) {
                            $stepReachedUntil = $now.AddMilliseconds([int]($StepAfterReachedHold * 1000))
                            Write-Host ("Reached step targets: {0}; recording {1:F1}s more" -f (Format-JointActuals $latestJointActual), $StepAfterReachedHold)
                        } elseif ($now -ge $stepReachedUntil) {
                            $scheduledDone = $true
                            if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                        }
                    } else {
                        $stepReachedUntil = $null
                    }
                    if (-not $scheduledDone -and $now -ge $nextStepTime) {
                        Write-Warning ("Step hold timeout reached before all targets settled. Last actual: {0}" -f (Format-JointActuals $latestJointActual))
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    }
                }
            } elseif ($FeedforwardGrid) {
                if ($gridState -eq "idle") {
                    Write-Host ("Feedforward grid settling to zero; waiting for all abs(actual) <= {0:F2} deg" -f $StepSettleTolerance)
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $gridSettleStart = $now
                    $gridNextTime = $now.AddSeconds(1)
                    $gridState = "settle"
                } elseif ($gridState -eq "settle") {
                    $zeroTargets = @(0.0, 0.0, 0.0, 0.0)
                    if (Test-JointsNearTargets $latestJointActual $zeroTargets $StepSettleTolerance) {
                        Write-Host ("Feedforward grid zero settled: {0}; setting servo software zero" -f (Format-JointActuals $latestJointActual))
                        Write-SerialCommand $serial "stop"
                        Start-Sleep -Milliseconds 300
                        Write-SerialCommand $serial "zero"
                        $gridResumeUntil = (Get-Date).AddMilliseconds(2000)
                        $gridState = "servo_zero"
                    } elseif ($gridSettleStart -ne $null -and (New-TimeSpan -Start $gridSettleStart -End $now).TotalSeconds -ge $StepSettleTimeout) {
                        Write-Warning ("Feedforward grid aborted: not all joints settled within +/-{0:F2} deg in {1:F1}s. Last actual: {2}" -f $StepSettleTolerance, $StepSettleTimeout, (Format-JointActuals $latestJointActual))
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    } elseif ($now -ge $gridNextTime) {
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $gridNextTime = $now.AddSeconds(1)
                    }
                } elseif ($gridState -eq "servo_zero" -and $now -ge $gridResumeUntil) {
                    Write-Host "Starting feedforward grid after servo zero"
                    Write-SerialCommand $serial "ff off"
                    Start-Sleep -Milliseconds 30
                    Write-SerialCommand $serial "tension off"
                    Start-Sleep -Milliseconds 30
                    Write-SerialCommand $serial "start"
                    Start-Sleep -Milliseconds 300
                    Write-SerialCommand $serial "degree"
                    Start-Sleep -Milliseconds 30
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $gridNextTime = (Get-Date).AddSeconds(1)
                    $gridState = "grid"
                    Write-Host ("Feedforward grid: points={0}; hold={1:F1}s; recording actual angles, motor now, solver, cmd, load, bias" -f $gridTargets.Count, $GridHold)
                } elseif ($gridState -eq "grid" -and $gridIndex -lt $gridTargets.Count -and $now -ge $gridNextTime) {
                    $target = $gridTargets[$gridIndex]
                    for ($j = 0; $j -lt 4; $j++) {
                        Write-SerialCommand $serial ("j{0} {1:F3}" -f $j, [double]$target[$j])
                        Start-Sleep -Milliseconds 30
                    }
                    Write-Host ("Grid point {0}/{1}: J0={2:F2} J1={3:F2} J2={4:F2} J3={5:F2}" -f ($gridIndex + 1), $gridTargets.Count, [double]$target[0], [double]$target[1], [double]$target[2], [double]$target[3])
                    $gridIndex++
                    $gridNextTime = $now.AddMilliseconds([int]($GridHold * 1000))
                } elseif ($gridState -eq "grid" -and $gridIndex -ge $gridTargets.Count -and $now -ge $gridNextTime) {
                    $scheduledDone = $true
                    if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                }
            } elseif ($FeedforwardCollect) {
                if ($collectState -eq "idle") {
                    if ($CollectServoZeroFirst) {
                        Write-Host "Starting feedforward collection after servo zero"
                        $collectNextTime = (Get-Date).AddSeconds(1)
                        $collectState = "collect"
                        $collectStarted = $true
                        $rangeSummary = New-Object System.Collections.Generic.List[string]
                        foreach ($joint in $collectJoints) {
                            $jointTargets = @($collectTargetMap[[int]$joint])
                            $rangeSummary.Add(("J{0}:{1:F1}->{2:F1} step points={3}" -f [int]$joint, [double]$jointTargets[0], [double]$jointTargets[$jointTargets.Count - 1], $jointTargets.Count))
                        }
                        Write-Host ("Feedforward collection: {0}; hold={1:F1}s; recording actual angles, motor now, solver, cmd, load, bias" -f ($rangeSummary -join "; "), $CollectHold)
                    } else {
                        Write-Host ("Feedforward collection settling to zero; waiting for all abs(actual) <= {0:F2} deg" -f $StepSettleTolerance)
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $collectSettleStart = $now
                        $collectNextTime = $now.AddSeconds(1)
                        $collectState = "settle"
                    }
                } elseif ($collectState -eq "settle") {
                    $zeroTargets = @(0.0, 0.0, 0.0, 0.0)
                    if (Test-JointsNearTargets $latestJointActual $zeroTargets $StepSettleTolerance) {
                        Write-Host ("Feedforward collection zero settled: {0}; setting servo software zero" -f (Format-JointActuals $latestJointActual))
                        Write-SerialCommand $serial "stop"
                        Start-Sleep -Milliseconds 300
                        Write-SerialCommand $serial "zero"
                        $collectResumeUntil = (Get-Date).AddMilliseconds(2000)
                        $collectState = "servo_zero"
                    } elseif ($collectSettleStart -ne $null -and (New-TimeSpan -Start $collectSettleStart -End $now).TotalSeconds -ge $StepSettleTimeout) {
                        Write-Warning ("Feedforward collection aborted: not all joints settled within +/-{0:F2} deg in {1:F1}s. Last actual: {2}" -f $StepSettleTolerance, $StepSettleTimeout, (Format-JointActuals $latestJointActual))
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    } elseif ($now -ge $collectNextTime) {
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $collectNextTime = $now.AddSeconds(1)
                    }
                } elseif ($collectState -eq "servo_zero" -and $now -ge $collectResumeUntil) {
                    Write-Host "Starting feedforward collection after servo zero"
                    Write-SerialCommand $serial "ff off"
                    Start-Sleep -Milliseconds 30
                    if ($EmpiricalCollect) {
                        Write-SerialCommand $serial "tension off"
                    } else {
                        Write-SerialCommand $serial "tension on"
                    }
                    Start-Sleep -Milliseconds 30
                    Write-SerialCommand $serial "start"
                    Start-Sleep -Milliseconds 300
                    Write-SerialCommand $serial "degree"
                    Start-Sleep -Milliseconds 30
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $collectNextTime = (Get-Date).AddSeconds(1)
                    $collectState = "collect"
                    $collectStarted = $true
                    $rangeSummary = New-Object System.Collections.Generic.List[string]
                    foreach ($joint in $collectJoints) {
                        $jointTargets = @($collectTargetMap[[int]$joint])
                        $rangeSummary.Add(("J{0}:{1:F1}->{2:F1} step points={3}" -f [int]$joint, [double]$jointTargets[0], [double]$jointTargets[$jointTargets.Count - 1], $jointTargets.Count))
                    }
                    Write-Host ("Feedforward collection: {0}; hold={1:F1}s; recording actual angles, motor now, solver, cmd, load, bias" -f ($rangeSummary -join "; "), $CollectHold)
                } elseif ($collectState -eq "collect" -and $collectJointIndex -lt $collectJoints.Count -and $now -ge $collectNextTime) {
                    $activeJoint = [int]$collectJoints[$collectJointIndex]
                    $activeTargets = @($collectTargetMap[$activeJoint])
                    if ($collectIndex -lt $activeTargets.Count) {
                        $target = [double]$activeTargets[$collectIndex]
                        $currentCollectJoint = $activeJoint
                        $currentCollectRequestDeg = $target
                        $currentCollectPoint = $collectIndex + 1
                        $currentCollectPointCount = $activeTargets.Count
                        foreach ($j in 0..3) {
                            $cmdTarget = if ($j -eq $activeJoint) { $target } else { 0.0 }
                            Write-SerialCommand $serial ("j{0} {1:F3}" -f $j, $cmdTarget)
                            Start-Sleep -Milliseconds 30
                        }
                        Write-Host ("Collect joint {0}/{1} J{2}, point {3}/{4}: target {5:F3}" -f ($collectJointIndex + 1), $collectJoints.Count, $activeJoint, ($collectIndex + 1), $activeTargets.Count, $target)
                        $collectIndex++
                        $collectNextTime = $now.AddMilliseconds([int]($CollectHold * 1000))
                    } else {
                        $collectJointIndex++
                        $collectIndex = 0
                        if ($collectJointIndex -lt $collectJoints.Count) {
                            $currentCollectJoint = ""
                            $currentCollectRequestDeg = ""
                            $currentCollectPoint = ""
                            $currentCollectPointCount = ""
                            foreach ($j in 0..3) {
                                Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                                Start-Sleep -Milliseconds 30
                            }
                            $collectNextTime = $now.AddMilliseconds([int]($CollectHold * 1000))
                        } else {
                            $scheduledDone = $true
                            if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                        }
                    }
                }
            } elseif ($SineJoint -ge 0) {
                if ($PidZeroThenFeedforwardSine -and $stepState -eq "idle") {
                    Write-Host ("Settling J0/J1/J2/J3 to 0 deg before sine; waiting for all abs(actual) <= {0:F2} deg" -f $StepSettleTolerance)
                    foreach ($j in 0..3) {
                        Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                        Start-Sleep -Milliseconds 30
                    }
                    $stepSettleStart = $now
                    $nextStepTime = $now.AddSeconds(1)
                    $stepState = "settle"
                } elseif ($PidZeroThenFeedforwardSine -and $stepState -eq "settle") {
                    $zeroTargets = @(0.0, 0.0, 0.0, 0.0)
                    if (Test-JointsNearTargets $latestJointActual $zeroTargets $StepSettleTolerance) {
                        Write-Host ("PID zero settled before sine: {0}; stopping control and setting servo software zero" -f (Format-JointActuals $latestJointActual))
                        Write-SerialCommand $serial "stop"
                        Start-Sleep -Milliseconds 300
                        Write-SerialCommand $serial "zero"
                        $ffZeroUntil = (Get-Date).AddMilliseconds(2000)
                        $stepState = "servo_zero"
                    } elseif ($stepSettleStart -ne $null -and (New-TimeSpan -Start $stepSettleStart -End $now).TotalSeconds -ge $StepSettleTimeout) {
                        Write-Warning ("Sine test aborted: not all joints settled within +/-{0:F2} deg in {1:F1}s. Last actual: {2}" -f $StepSettleTolerance, $StepSettleTimeout, (Format-JointActuals $latestJointActual))
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    } elseif ($now -ge $nextStepTime) {
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $nextStepTime = $now.AddSeconds(1)
                    }
                } elseif ($PidZeroThenFeedforwardSine -and $stepState -eq "servo_zero") {
                    if ($now -ge $ffZeroUntil) {
                        Write-Host "Switching to feedforward+PID before sine"
                        Write-SerialCommand $serial "ff on"
                        Start-Sleep -Milliseconds 30
                        Write-SerialCommand $serial "tension on"
                        Start-Sleep -Milliseconds 30
                        Write-SerialCommand $serial "start"
                        Start-Sleep -Milliseconds 300
                        Write-SerialCommand $serial "degree"
                        Start-Sleep -Milliseconds 30
                        foreach ($j in 0..3) {
                            Write-SerialCommand $serial ("j{0} 0.000" -f $j)
                            Start-Sleep -Milliseconds 30
                        }
                        $ffResumeUntil = (Get-Date).AddMilliseconds(1000)
                        $stepState = "ff_resume"
                    }
                } elseif ($PidZeroThenFeedforwardSine -and $stepState -eq "ff_resume" -and $now -lt $ffResumeUntil) {
                    # Wait for feedforward baseline to become active.
                } else {
                    if ($PidZeroThenFeedforwardSine -and $stepState -eq "ff_resume") {
                        $stepState = "sine"
                    }
                    if ($sineStart -eq $null) {
                        $sineStart = $now
                        $nextSineTime = $now
                        Write-Host ("Sine j{0}: target = {1:F3} + {2:F3}*sin(2*pi*{3:F3}*t), duration={4:F1}s" -f $SineJoint, $SineOffset, $SineAmp, $SineFreq, $SineDuration)
                    }
                    $elapsed = (New-TimeSpan -Start $sineStart -End $now).TotalSeconds
                    if ($elapsed -gt $SineDuration) {
                        Write-SerialCommand $serial ("j{0} 0" -f $SineJoint)
                        $scheduledDone = $true
                        if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                    } elseif ($now -ge $nextSineTime) {
                        $target = $SineOffset + $SineAmp * [Math]::Sin(2.0 * [Math]::PI * $SineFreq * $elapsed)
                        Write-SerialCommand $serial ("j{0} {1:F3}" -f $SineJoint, $target)
                        $nextSineTime = $nextSineTime.AddMilliseconds([int]($SineInterval * 1000))
                    }
                }
            }
        }

        try {
            $line = $serial.ReadLine()
            Add-Content -LiteralPath $rawOutput -Value $line
            if ($EchoRaw) { Write-Host $line }
            $row = Parse-ControlLine $line $start
            if ($row -ne $null) {
                $row.collect_joint = $currentCollectJoint
                $row.collect_request_deg = $currentCollectRequestDeg
                $row.collect_point = $currentCollectPoint
                $row.collect_point_count = $currentCollectPointCount
                for ($j = 0; $j -lt 4; $j++) {
                    $actualValue = $row.("j${j}_actual")
                    if ($actualValue -ne "") {
                        $latestJointActual[$j] = [double]$actualValue
                    }
                }
                if ($FeedforwardCollect -and $CollectAbortAbsActual -gt 0 -and -not $collectAbortTriggered) {
                    for ($j = 0; $j -lt 4; $j++) {
                        if (-not [double]::IsNaN($latestJointActual[$j]) -and [Math]::Abs($latestJointActual[$j]) -gt $CollectAbortAbsActual) {
                            $collectAbortTriggered = $true
                            Write-Warning ("Collection abort: J{0} actual {1:F2} deg exceeded abs safety limit {2:F2} deg. Last actual: {3}" -f $j, $latestJointActual[$j], $CollectAbortAbsActual, (Format-JointActuals $latestJointActual))
                            try { Write-SerialCommand $serial "stop" } catch {}
                            $scheduledDone = $true
                            if ($deadline -eq $null) { $deadline = (Get-Date).AddSeconds(2) }
                            break
                        }
                    }
                }
                $row | Export-Csv -LiteralPath $Output -Append -NoTypeInformation
                $rows++
                if (($rows % 20) -eq 0) {
                    Write-Host "rows=$rows"
                }
            }
        } catch [System.TimeoutException] {}
    }
} finally {
    if ($StopOnExit -and $serial.IsOpen) {
        try {
            if ($FeedforwardCollect -or $FeedforwardGrid) {
                Write-SerialCommand $serial "tension on"
                Start-Sleep -Milliseconds 30
            }
            Write-SerialCommand $serial "stop"
        } catch {
            Write-Warning "Failed to send stop on exit: $_"
        }
    }
    if ($serial.IsOpen) { $serial.Close() }
}
Write-Host "Saved $rows rows to $Output"
