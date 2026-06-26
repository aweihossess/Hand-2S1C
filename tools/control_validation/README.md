# MCP Control Validation

These tools record `[MCP5 CTRL]` text logs into CSV and generate simple SVG plots.

## Record With PowerShell

Use this on Windows without extra Python packages:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -Duration 30
```

Step test. The script first commands J0/J1/J2/J3 back to 0 deg, waits until
all four measured angles are within the settle tolerance, records a short
baseline, then sends one step from 0 deg to `StepAmp` on the selected joint.
It stops after all four joints reach their requested targets and stay there
for `StepAfterReachedHold` seconds, or when `StepHold` expires.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -StepJoint 3 -StepAmp 10 -StepHold 60 -StepSettleTolerance 1 -StepSettleTimeout 60 -StepBaselineHold 2 -StepAfterReachedHold 5
```

Add `-StopOnExit` to send `stop` automatically when the script exits:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -StepJoint 3 -StepAmp 10 -StepHold 60 -StepSettleTolerance 1 -StepSettleTimeout 60 -StepBaselineHold 2 -StepAfterReachedHold 5 -StopOnExit
```

Sine test:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -SineJoint 3 -SineAmp 5 -SineFreq 0.05 -SineDuration 60 -SineInterval 0.1
```

Feedforward collection. This first uses PID to settle all four MCP joints near
0 deg, sets servo software zero, disables firmware tendon LUT feedforward, keeps
MCP tension bias enabled, then sweeps one joint at a time while the other three
targets stay at 0 deg. The CSV still records all measured joint angles,
including passive coupling, plus all five motor `now/load/bias/solver/cmd`
values.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -FeedforwardCollect -CollectAllJoints -CollectHold 2 -StepSettleTolerance 3 -StepSettleTimeout 60 -StopOnExit -EchoRaw
```

For realistic tight-tendon feedforward data, use `-FeedforwardCollect`, not
`-EmpiricalCollect`. This keeps `tension on` after servo software zero while
still keeping firmware tendon LUT feedforward off:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM6 -FeedforwardCollect -CollectAllJoints -CollectRanges "0:-30:30:1;1:-30:30:1;2:-30:30:1;3:-30:30:1" -CollectHold 2 -StepSettleTolerance 3 -StepSettleTimeout 60 -CollectAbortAbsActual 100 -StopOnExit
```

Empirical feedforward collection for a changed mechanism. This does not use the
firmware LUT or tension bias during collection. It sweeps one joint at a time
from -30 deg to +30 deg in 1 deg steps, while the other three commanded joints
stay at 0 deg. The important recorded fields are measured joint angles
`j*_actual` and motor positions relative to servo software zero `m*_now`.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM4 -EmpiricalCollect -CollectHold 2 -StepSettleTolerance 3 -StepSettleTimeout 60 -StopOnExit -EchoRaw
```

For P4 on `COM6`, start with a smaller single-joint safety run before sweeping
the full range:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\control_validation\record_mcp_control.ps1 -Port COM6 -EmpiricalCollect -CollectRanges "0:-10:10:1" -CollectHold 3 -StepSettleTolerance 3 -StepSettleTimeout 60 -StopOnExit -EchoRaw
```

Use `-CollectAbortAbsActual` when you want an extra host-side safety stop. For
example, `-CollectAbortAbsActual 100` sends `stop` and exits if any measured
joint angle exceeds `+/-100 deg`. If the option is omitted, the collection keeps
recording overshoot and coupling beyond the commanded sweep range.

Summarize the raw control CSV into one averaged sample per commanded point, then
export measured feedforward points:

```powershell
& "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\control_validation\summarize_feedforward_collect.py run_data\mcp_control_YYYYMMDD_HHMMSS.csv --all-joints --avg-tail-sec 1 --out run_data\mcp_control_YYYYMMDD_HHMMSS_empirical_samples.csv
& "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\control_validation\build_empirical_feedforward_table.py run_data\mcp_control_YYYYMMDD_HHMMSS_empirical_samples.csv --max-bias-abs 1 --out-prefix run_data\mcp_control_YYYYMMDD_HHMMSS_empirical
```

To preserve every control row, including transient overshoot and passive
coupling, export row-level empirical points:

```powershell
& "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\control_validation\export_empirical_row_points.py run_data\mcp_control_YYYYMMDD_HHMMSS.csv --out-prefix run_data\mcp_control_YYYYMMDD_HHMMSS_empirical_rows
```

Default collection ranges are `J0 -20..30`, `J1 -20..80`, `J2 0..88`, and
`J3 0..93`, all at 1 deg steps. Override them with `-CollectRanges`, for
example `-CollectRanges "0:-10:20:1;1:0:60:1;2:0:60:1;3:0:60:1"`.

The default init command is:

```text
text; zero; start; degree; j0 0; j1 0; j2 0; j3 0
```

## Plot CSV

Use the Codex bundled Python if normal `python` is not on PATH:

```powershell
& "C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\control_validation\plot_mcp_control.py run_data\mcp_control_YYYYMMDD_HHMMSS.csv
```

By default the plotter crops away the pre-step settling period. It detects the
first `J3` target change to `10 deg`, sets that row to `t=0`, and plots only
the step response. Add `--no-step-crop` to plot the full record.

Outputs:

```text
joints.svg
motors.svg
loads.svg
summary.txt
```
