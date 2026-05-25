# Low Tension Take-Up Collector

This tool is intentionally separate from the main hand controller firmware and
desktop GUI. It uses the firmware text monitor commands to run a low-tension
slack take-up loop for MCP tendon motors and records data for offline tendon
mapping.

## What It Does

The loop keeps each selected tendon motor lightly tensioned:

```text
read motor_abs and current
estimate current above idle
increase bias slowly if current is too low
release bias quickly if current is too high
command target = current motor_abs + tighten_direction * bias
record joint angles, motor_abs, current, and bias to CSV
```

The target follows the current motor position, so the servo does not hold a
fixed old position while you manually move the finger. The small bias only takes
up slack.

## Safety Assumptions

- Start with the tendons detached or very loose for the first dry run.
- Confirm `tighten_dir` for each motor before using the loop.
- Keep your hand near power or emergency stop.
- Use small thresholds first. This is not precision tension control.

## Files

- `low_tension_takeup.py`: main collector.
- `config_mcp.json`: default MCP 3-tendon configuration.
- `requirements.txt`: Python dependency.

## Firmware Mode

The script talks to the existing firmware text monitor for snapshots. It sends:

```text
text
start
direct
```

Then it periodically sends commands such as:

```text
CMD_MOTOR_POS_ABS binary frames with all 22 motor targets
```

Sending the full 22-channel absolute target frame matters because single text
commands like `m0 123` rebuild the remaining motor targets from the latest
feedback and can overwrite another tendon's pending tiny step.

It reads snapshots using:

```text
servo
load
encoder
```

## Install

```powershell
python -m pip install -r .\tools\low_tension_takeup\requirements.txt
```

## Dry Run

Dry run prints targets and records CSV, but does not command motors:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --dry-run --duration 20 --out .\run_data\low_tension_dry.csv
```

## Run

Use conservative settings first:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --duration 60 --out .\run_data\mcp_low_tension_collect.csv
```

While it runs, slowly move the MCP joint by hand or with a fixture. The CSV will
contain:

```text
time_s
j00_deg, j01_deg
m00_abs, m01_abs, m02_abs
m00_current, m01_current, m02_current
m00_bias, m01_bias, m02_bias
m00_target, m01_target, m02_target
```

## Tune The Config

Edit `config_mcp.json`:

```json
{
  "motors": [
    {"channel": 0, "name": "M00/R", "tighten_dir": 1},
    {"channel": 1, "name": "M01/L", "tighten_dir": 1},
    {"channel": 2, "name": "M02/C", "tighten_dir": 1}
  ]
}
```

`tighten_dir` must be `+1` if increasing motor_abs tightens that tendon, or `-1`
if decreasing motor_abs tightens it.

Important thresholds:

```text
idle_samples      number of samples used to estimate idle current
i_low_counts      below this effective current, add bias slowly
i_high_counts     above this effective current, release bias quickly
tighten_step      counts added to bias per control cycle
release_step      counts removed from bias per control cycle
bias_max_each     max bias per motor
bias_total_max    total bias budget across selected motors
max_target_step   max command step from current motor_abs
```

## Use The Data

After collection, fit one map per motor:

```text
[J00_deg, J01_deg] -> M00_abs
[J00_deg, J01_deg] -> M01_abs
[J00_deg, J01_deg] -> M02_abs
```

Start with a regular grid plus bilinear interpolation, then consider a small MLP
or GPR later if the mapping is strongly nonlinear.

## Hold A Motor And Pull The Tendon

Use `hold-probe` to test whether servo telemetry reacts to external tendon load.
This mode holds the selected motor's current absolute position and records
`current`, `load`, `effective_current`, and `effective_load`. It does not add
take-up bias.

For M02 only:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --config .\tools\low_tension_takeup\config_m02_probe.json --mode hold-probe --duration 30 --confirm-start --out .\run_data\m02_hold_probe.csv
```

After the `YES` confirmation, pull and release the M02 tendon by hand. Press
`Ctrl+C` to stop; the tool sends `stop` on exit.

## Two-Stage Pulse Probe

Use `pulse-probe` with `--pulse-stage-prompt` to collect a loose phase and a
loaded phase in one CSV. The tool moves the selected motor in small steps,
pauses after the first stage, holds position, and waits for you to type `2`.

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --config .\tools\low_tension_takeup\config_motor_only_probe.json --mode pulse-probe --duration 120 --pulse-step 80 --pulse-max-travel 16384 --pulse-stage-prompt --pulse-stage-travel 4096 --pulse-load-stop 0 --pulse-current-stop 0 --pulse-angle-stop 0 --confirm-start --out .\run_data\m02_two_stage_pulse.csv
```

Phase labels are written to the CSV as `loose` and `loaded`.

## Tie Slack At A Target Angle

If all tendons are loose and you want to tighten them at a target posture, use
`tie-angle` mode. This mode tightens one tendon at a time and watches J00/J01.
If the joint drifts more than the tolerance, it releases instead of continuing
to pull.

Capture the current J00/J01 as the target posture:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --mode tie-angle --duration 30 --out .\run_data\mcp_tie_angle.csv
```

Or specify the target:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --mode tie-angle --target-j00 0 --target-j01 20 --duration 30 --out .\run_data\mcp_tie_angle.csv
```

Recommended first run:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --mode tie-angle --dry-run --duration 15
```

Useful tie-angle options:

```text
--angle-tolerance     max allowed J00/J01 drift before release, default 1.5 deg
--ignore-target-drift disable relative target drift release while keeping absolute safety bounds
--safe-window         absolute safety window around target for all joints
--j00-safe-min/max    absolute safety bounds for J00
--j01-safe-min/max    absolute safety bounds for J01
--tie-current         effective current target, default 25 counts
--tie-step            tightening step in motor counts, default 2
--angle-release-step  release step if angle drifts, default 8
```

The default config also contains absolute safety limits:

```text
J00: -20..30 deg
J01: -20..80 deg
```

These are independent of `--angle-tolerance`. `--angle-tolerance` protects the
target posture from being pulled away; the absolute safety bounds protect the
joint from entering an unsafe region at all. If either protection trips, the
tool releases instead of tightening.

When all tendons are very loose and you are holding the finger by hand, the
joint angle can shake enough to trigger `target_drift_release`. For this initial
tie-up stage, keep the absolute joint bounds conservative and disable only the
relative target drift release:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --mode tie-angle --duration 2 --tie-step 1 --tie-current 1 --ignore-target-drift --j00-safe-min -20 --j00-safe-max 45 --j01-safe-min -20 --j01-safe-max 95 --out .\run_data\mcp_initial_tie.csv
```

This is not posture control. It only takes up slack slowly. Stop immediately if
any tendon tightens in the wrong direction, then flip that motor's `tighten_dir`
in `config_mcp.json`.

If your encoder mapping currently reports a valid resting posture outside these
defaults, for example `J00=50`, the tool will refuse to enable output. In that
case, confirm whether the encoder zero/range is correct. Only after confirming
the real safe range should you change `config_mcp.json` or pass explicit bounds:

```powershell
python .\tools\low_tension_takeup\low_tension_takeup.py --port COM10 --mode tie-angle --j00-safe-min -20 --j00-safe-max 60 --j01-safe-min -20 --j01-safe-max 80 --duration 30
```

Start small. If the tendons remain loose, raise `--tie-current` slightly or
increase `--duration`; if the finger is pulled away from target, lower
`--tie-current`, lower `bias_max_each` in the config, or reduce `--tie-step`.
