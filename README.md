# Hand-2S1C Tendon Feedforward + Angle PID Firmware

This branch is the next control experiment after the tendon-length PD version.
The core idea is:

```text
tendon model = feedforward
joint angle error = feedback PID
motorTarget = tendon feedforward + 2x2 angle PID feedback
```

This is intended to avoid the previous behavior where the motor reached a model
tendon-length target while the joint angle still had steady-state error.

## Current MCP Mapping

```text
J00 = MCP-AA
J01 = MCP-FE
M00 = model R tendon
M01 = model L tendon
```

Current base parameters:

```text
lengthToPulse M00/M01 = -160 counts/mm
MCP command max step  = 320 counts per control update
MCP motor abs guard   = +/-6400 counts
tracking error guard  = 30 deg
```

Current 2x2 angle feedback matrix:

```text
          J00 error   J01 error
M00/R P   -20          +10
M01/L P   +20          +10

M00/R I   -1           +0.5
M01/L I   +1           +0.5

M00/R D    0            0
M01/L D    0            0
```

Integrator limit:

```text
J00 integral = +/-30 deg*s
J01 integral = +/-30 deg*s
```

Angle feedback output limit:

```text
M00 feedback = +/-1200 counts
M01 feedback = +/-1200 counts
```

## Build And Upload

Open this sketch in Arduino IDE:

```text
ESP32-P4/ServoBoardMain/ServoBoardMain.ino
```

Board:

```text
ESP32P4 Dev Module
```

Compile from PowerShell:

```powershell
& 'C:\Users\hand\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32:esp32:esp32p4 --libraries 'D:\mmhand-main\ESP32-P4\libraries' 'D:\mmhand-main\ESP32-P4\ServoBoardMain'
```

Upload after closing Arduino Serial Monitor or any desktop app using the port:

```powershell
& 'C:\Users\hand\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --upload -p COM10 --fqbn esp32:esp32:esp32p4 --libraries 'D:\mmhand-main\ESP32-P4\libraries' 'D:\mmhand-main\ESP32-P4\ServoBoardMain'
```

## Serial Monitor Mode

Send this first:

```text
text
```

This disables binary telemetry so Arduino Serial Monitor stays readable.

To restore binary telemetry for the desktop UI:

```text
binary
```

## Command Reference

```text
text        clean text monitor mode
binary      binary telemetry / desktop UI mode
start       enable control output and enter RUNNING state
stop        stop control output
reset       clear command state and return to IDLE
zero        use current servo hardware positions as software zero
servo       print motor_abs, hardware_abs, sw_zero_ofs, online
encoder     print magnetic encoder raw/mapped/degree values
status      print mode/enabled/owner/state/fault summary
direct      enter direct motor mode
degree      enter joint angle mode
idle        leave active control mode
```

Accepted aliases include:

```text
motor, enc, deg, joint, motor_mode
```

## Motor Position Meaning

Servo feedback uses:

```text
motor_abs = hardware_abs - sw_zero_ofs
hardware_target = motor_target + sw_zero_ofs
```

After `zero`, current servo positions become software zero:

```text
hardware_abs = sw_zero_ofs
motor_abs = 0
```

Important: `zero` only defines motor software zero. It does not redefine magnetic
encoder joint angles.

## Direct Motor Mode

Use direct mode first to verify each servo command path.

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

Direct targets are `motor_abs` counts:

```text
m0 1000
m0=-1000
m1 200
motor 0 500
```

Direct motor targets are accepted only after `direct`.

## Degree Mode Usage

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

Joint targets are accepted only after `degree`:

```text
j0 10
j1 20
joint0 10
degree 1 20
```

Current software limits:

```text
J00 / MCP-AA: -20 deg to +30 deg
J01 / MCP-FE: 0 deg to 90 deg
```

## Degree Control Principle

The degree-mode control path for J00/J01 is now:

```text
1. Receive J00/J01 target angles.
2. Filter target and magnetic encoder feedback.
3. Convert target angles to model target tendon lengths.
4. Use the tendon model only as feedforward:
      feedforward = (targetLen - zeroLen) * lengthToPulse
5. Compute joint angle errors directly:
      e00 = J00_target - J00_actual
      e01 = J01_target - J01_actual
6. Apply the 2x2 angle PID matrix to produce motor feedback counts.
7. Add feedforward and angle feedback:
      motor_abs_target = feedforward + angle_feedback
8. Limit per-cycle command step and motor_abs range.
9. Send M00/M01 motor targets to the servo layer.
```

This means the angle feedback can keep pushing while the joint angle has not reached
the target, even if the model feedforward target was already reached.

## Reading The MCP Log

Example:

```text
[MCP CTRL] J00 target=10.00 actual=7.10 J01 target=0.00 actual=9.76 M00/R targetLen=27.331 actualLen=25.882 mappedMotor=-529.0 solver=-528 cmd=-528 M01/L targetLen=24.599 actualLen=24.190 mappedMotor=-91.8 solver=-91 cmd=-91 [SERVO TARGET] ...
```

Fields:

```text
J00/J01 target     requested joint angle, deg
J00/J01 actual     magnetic encoder feedback, deg
M00/R targetLen    model R tendon length for M00 target, mm
M00/R actualLen    model R tendon length from encoder feedback, shown for diagnosis
M01/L targetLen    model L tendon length for M01 target, mm
M01/L actualLen    model L tendon length from encoder feedback, shown for diagnosis
mappedMotor        tendon feedforward + angle PID feedback, motor_abs counts
solver             rounded solver output
cmd                final command after step limiting and motor guard
motorTarget        command sent to servo layer, motor_abs counts
swZero             servo software zero offset
hardwareTarget     motorTarget + swZero
motorAbsNow        current hardware_abs - swZero
hardwareAbsNow     current hardware absolute multi-turn position
```

In this branch, `actualLen` is diagnostic only. It no longer directly drives the PD
feedback term.

## Safety Messages

Examples:

```text
[JOINT SAFETY] MCP encoder out of range ...
[JOINT SAFETY] tracking error out of range ...
[JOINT SAFETY] MCP motor abs out of range ...
```

Meanings:

```text
MCP encoder out of range     J00/J01 encoder feedback exceeded the configured angle range.
tracking error out of range  target angle - actual angle exceeded 30 deg.
MCP motor abs out of range   M00/M01 motor_abs exceeded +/-6400 counts.
```

## Recommended Test Order

1. Send `text`.
2. Send `zero`.
3. Send `servo` and confirm M00/M01 show `motor_abs=0`.
4. Enter `direct` mode and test small `m0/m1` moves.
5. Send `encoder` and confirm J00/J01 are valid and plausible.
6. Enter `degree` mode.
7. Test `j0 5`, then `j0 10`.
8. Test `j1 10`, then `j1 20`.
9. If the joint overshoots or oscillates, reduce the 2x2 angle gains.
10. If the joint stalls before reaching target, increase the relevant angle P or I gain carefully.

If direction is wrong, check motor placement, tendon routing, and encoder direction before
increasing gains.

