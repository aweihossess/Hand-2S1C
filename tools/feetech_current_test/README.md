# Feetech ST3215 Current Test

This folder is a standalone PC-side test for a Feetech ST3215-C018 servo and a
Feetech TTL servo controller/USB adapter. It commands one servo in wheel mode
or position mode and prints current feedback from registers 69/70.

## Install

```powershell
python -m pip install pyserial
```

Or:

```powershell
python -m pip install -r .\tools\feetech_current_test\requirements.txt
```

## Find The Serial Port

Open Windows Device Manager and check the COM port created by the Feetech USB
servo controller. Replace `COM6` below with your actual port.

The default baud rate is `1000000`, which is the usual STS/SMS Feetech baud.

## Read Only

Use this first to confirm communication:

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --mode read
```

The old alias also works:

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --read-only
```

## Wheel Mode

This makes the servo spin back and forth and prints current while it is moving:

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --mode wheel --speed 120 --duration 20
```

## Position Mode

This alternates between two position targets around the current position:

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --mode position --amplitude 200 --speed 800 --duration 20
```

## Hold Mode

This holds the current position. Use this to test whether external torque
appears in `current` while the servo is actively holding:

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --mode hold --duration 0
```

The output contains:

```text
current      signed raw current register value
current_mA   current * 6.5 mA, approximate
load         PRESENT_LOAD register, often less reliable for tendon tension
pos          signed present position count
speed        signed present speed count
```

For ST3215-C018:

```text
1 current raw count ~= 6.5 mA
```

## Suggested Current Test

1. Run `--mode read` with the servo idle.
2. Run `--mode wheel` with no external load.
3. While moving, gently resist the output shaft and observe `current`.
4. Run `--mode hold`, then gently twist the horn while it holds position.
5. Compare loose, lightly tensioned, and strongly resisted states.

If `current` rises only while the servo actively drives against resistance, use
current as a tension proxy only when the controller is actively tensioning.

## CSV Logging

```powershell
python .\tools\feetech_current_test\feetech_current_test.py --port COM6 --id 1 --mode hold --duration 20 --csv .\tools\feetech_current_test\hold_test.csv
```

## Notes

- `duration 0` means run until `Ctrl+C`.
- Torque is disabled when the script exits. Add `--keep-torque-on-exit` if you
  want the servo to keep holding after the script stops.
- Use a low speed first if the servo is connected to a mechanism.
- If there is no response, check ID, baud rate, power, GND, and TX/RX direction.
