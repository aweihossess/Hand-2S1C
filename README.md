# Hand-2S1C Servo Control Quick Start

This repository contains firmware and desktop tools for the Hand-2S1C servo hand.
The current recommended debug path is to use the ESP32-P4 ServoBoard firmware through
the Arduino Serial Monitor with plain text commands.

## Current Firmware Focus

- Direct motor position control.
- Degree/joint control for the MCP two-motor tendon mechanism.
- Text-mode serial commands for clean Arduino Serial Monitor operation.
- Servo software zeroing with `motor_abs = hardware_abs - sw_zero_ofs`.

## Build And Upload

Open:

```text
ESP32-P4/ServoBoardMain/ServoBoardMain.ino
```

Board:

```text
ESP32P4 Dev Module
```

Useful CLI compile command on the current Windows setup:

```powershell
& 'C:\Users\hand\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32:esp32:esp32p4 --libraries 'D:\mmhand-main\ESP32-P4\libraries' 'D:\mmhand-main\ESP32-P4\ServoBoardMain'
```

Upload, after closing Arduino Serial Monitor or any app using the port:

```powershell
& 'C:\Users\hand\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --upload -p COM10 --fqbn esp32:esp32:esp32p4 --libraries 'D:\mmhand-main\ESP32-P4\libraries' 'D:\mmhand-main\ESP32-P4\ServoBoardMain'
```

## Serial Monitor Setup

After connecting, send:

```text
text
```

This disables binary telemetry so the Arduino Serial Monitor stays readable. To re-enable
binary telemetry for the desktop UI, send:

```text
binary
```

## Core Commands

```text
text        clean text monitor mode
binary      binary telemetry / desktop UI mode
start       enable control output and enter RUNNING state
stop        stop control output
reset       clear command state and return to IDLE
zero        set current servo hardware positions as software zero
servo       print M00/M01/... motor_abs, hardware_abs, sw_zero_ofs
encoder     print magnetic encoder raw/mapped/degree values
status      print mode/enabled/owner/state/fault summary
direct      enter direct motor mode
degree      enter joint angle mode
idle        leave active control mode
```

Common aliases are accepted, for example `motor`, `enc`, `deg`, `joint`, and `motor_mode`.

## Position Meaning

Servo feedback uses:

```text
motor_abs = hardware_abs - sw_zero_ofs
hardware_target = motor_target + sw_zero_ofs
```

After `zero`, the current hardware position becomes the servo software zero:

```text
hardware_abs = sw_zero_ofs
motor_abs = 0
```

This is motor zero only. It does not automatically make magnetic encoder joint angles
become `0 deg`.

## Safe Direct Motor Test

Use this when checking whether a motor can receive commands and move accurately.

```text
text
stop
zero
servo
start
direct
m0 100
servo
m0 0
m1 100
servo
m1 0
```

Direct motor targets are motor-relative positions in counts:

```text
m0 1000
m0=-1000
motor 0 500
m1 200
```

Direct motor commands are accepted only after `direct`.

## Degree / Joint Control Test

Use small targets first:

```text
text
stop
zero
servo
encoder
start
degree
j0 5
j0 10
j1 10
j1 20
```

Joint commands are accepted only after `degree`:

```text
j0 10
j1 20
joint0 10
degree 1 20
```

For the current MCP setup:

```text
J00 = MCP-AA
J01 = MCP-FE
M00 = model R tendon
M01 = model L tendon
```

Current limits:

```text
J00 / MCP-AA: -20 deg to +30 deg
J01 / MCP-FE: 0 deg to 90 deg
tracking error protection: 30 deg
MCP motor abs guard: +/-6400 counts
```

## MCP Control Log

A typical line:

```text
[MCP CTRL] J00 target=10.00 actual=7.10 J01 target=0.00 actual=9.76 M00/R targetLen=27.331 actualLen=25.882 mappedMotor=-529.0 solver=-528 cmd=-528 M01/L targetLen=24.599 actualLen=24.190 mappedMotor=-91.8 solver=-91 cmd=-91 [SERVO TARGET] ...
```

Meaning:

- `J00/J01 target`: requested joint angle in degrees.
- `J00/J01 actual`: magnetic encoder feedback in degrees.
- `M00/R targetLen`: model tendon length for M00's assigned R tendon target.
- `M00/R actualLen`: model tendon length estimated from encoder feedback.
- `mappedMotor`: model feedforward plus tendon length feedback, in motor counts.
- `solver`: rounded solver output.
- `cmd`: actual command after per-cycle step limiting and motor guard limiting.
- `motorTarget`: command sent to the servo layer, in `motor_abs` counts.
- `hardwareTarget`: `motorTarget + swZero`.

If `mappedMotor` jumps between two large values, the tendon length feedback gain or
derivative term is too aggressive. The current conservative tuning is:

```text
lengthToPulse: -160 counts/mm for M00 and M01
tendon length Kp: 10
tendon length Kd: 0.05
feedback correction clamp: +/-2 mm
```

## Safety Messages

Examples:

```text
[JOINT SAFETY] MCP encoder out of range ...
[JOINT SAFETY] tracking error out of range ...
[JOINT SAFETY] MCP motor abs out of range ...
```

These mean the firmware is holding or blocking motion because a joint angle, tracking
error, or motor position exceeded the configured safety range.

## Recommended Debug Order

1. Send `text`.
2. Send `zero`, then confirm `servo` shows `motor_abs=0`.
3. Test direct mode with small `m0/m1` targets.
4. Send `encoder` and confirm J00/J01 feedback is connected and plausible.
5. Test degree mode with small `j0` targets.
6. Test `j1` targets only after J00 direction looks correct.
7. If a motor moves but the joint angle goes the wrong way, check motor placement,
   tendon routing, and encoder direction before increasing gains.

