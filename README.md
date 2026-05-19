# Hand-2S1C Tendon-Length PD Firmware

This branch records the current ESP32-P4 ServoBoard debug firmware for the
two-servo MCP tendon mechanism. The key idea of this version is:

```text
joint angle command -> MCP tendon length model -> tendon-length PD -> motor_abs target
```

In other words, degree mode is not a pure joint-angle PID yet. The geometric tendon
model is part of both feedforward and feedback.

## What This Version Contains

- Clean text commands for Arduino Serial Monitor.
- Direct motor mode for checking whether each servo can execute `motor_abs` targets.
- Degree mode for J00/J01 using the current MCP tendon length model.
- Servo software zeroing using `motor_abs = hardware_abs - sw_zero_ofs`.
- MCP debug logs that show joint target/feedback, tendon length target/feedback, solver output, and final servo target.

Current MCP mapping:

```text
J00 = MCP-AA
J01 = MCP-FE
M00 = model R tendon
M01 = model L tendon
```

Current tuning:

```text
lengthToPulse M00/M01 = -160 counts/mm
tendon length Kp      = 10
tendon length Kd      = 0.05
feedback clamp        = +/-2 mm
MCP command max step  = 320 counts per control update
MCP motor abs guard   = +/-6400 counts
tracking error guard  = 30 deg
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

Recommended test:

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

If the target is outside the range, the firmware clamps it and reports the applied value.

## Degree Control Principle

The degree-mode control path for J00/J01 is:

```text
1. Receive J00/J01 target angles.
2. Filter target and magnetic encoder feedback.
3. Convert target angles to model target tendon lengths.
4. Convert feedback angles to model actual tendon lengths.
5. Compute tendon length error.
6. Apply tendon-length PD correction.
7. Add model feedforward and convert mm to motor_abs counts.
8. Limit per-cycle command step and motor_abs range.
9. Send M00/M01 motor targets to the servo layer.
```

The core formula is:

```text
length_error = target_tendon_length - actual_tendon_length
feedback_mm = Kp * length_error + Kd * d(length_error)/dt
feedback_mm = clamp(feedback_mm, -2 mm, +2 mm)

feedforward_mm = target_tendon_length - zero_tendon_length
motor_abs_target = (feedforward_mm + feedback_mm) * lengthToPulse
```

For this branch:

```text
lengthToPulse = -160 counts/mm
```

Because `lengthToPulse` is negative, a smaller model tendon length can produce a positive
motor target, depending on the assigned tendon and motor placement.

## Important Limitation Of This Version

This branch uses tendon-length PD, not joint-angle PID. That means the firmware tries to
make the model tendon length match the model target tendon length. If the real mechanism
does not match the model perfectly, the motor may reach its computed `motor_abs` target
while the joint angle still has steady-state error.

Example symptom:

```text
J00 target=10 deg
J00 actual settles near 5 deg
M00/M01 motorTarget stops changing
```

That means the tendon-length solver reached its current motor target. It does not mean a
joint-angle integrator is still pushing toward the angle target. A future branch should
test:

```text
motorTarget = tendon_model_feedforward + 2x2 joint-angle PID feedback
```

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
M00/R actualLen    model R tendon length from encoder feedback, mm
M01/L targetLen    model L tendon length for M01 target, mm
M01/L actualLen    model L tendon length from encoder feedback, mm
mappedMotor        feedforward + tendon-length PD output, motor_abs counts
solver             rounded solver output
cmd                final command after step limiting and motor guard
motorTarget        command sent to servo layer, motor_abs counts
swZero             servo software zero offset
hardwareTarget     motorTarget + swZero
motorAbsNow        current hardware_abs - swZero
hardwareAbsNow     current hardware absolute multi-turn position
```

If you see:

```text
mappedMotor=873.8 solver=873 cmd=873 motorAbsNow=873
```

then the motor has reached the target computed by the tendon-length solver. It is not
being blocked by the motor guard unless a `[JOINT SAFETY]` message is printed.

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
9. Watch whether `actual` angles move toward targets and whether `mappedMotor` stops at a fixed value.

If direction is wrong, check motor placement, tendon routing, and encoder direction before
increasing gains.

