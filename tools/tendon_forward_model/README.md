# Tendon Forward Model Tools

This folder is the offline source of the tendon-length feedforward model. The
Python model is used to generate lookup tables that are exported into the
ESP32-P4 firmware.

The intended integration candidate is:

```text
tendon_length_feedforward_model.py
```

It is the unified five-tendon feedforward model. The per-motor scripts are kept
as debug tools and geometry references.

## Current Model

Implemented:

```text
M00 / ID1 / MCP right swing motor
M01 / ID2 / MCP left swing motor
M02 / ID3 / PIP flexion motor
M03 / ID4 / distal flexion motor
M04 / ID5 / common return motor
```

Current meaning from the latest hardware mapping:

```text
M00 / ID1: MCP-AA right swing + MCP-FE flexion
M01 / ID2: MCP-AA left swing  + MCP-FE flexion
M02 / ID3: MCP routing correction + PIP-FE flexion
M03 / ID4: MCP routing correction + PIP-FE + DIP-FE flexion
M04 / ID5: MCP-AA + MCP-FE + PIP-FE + DIP-FE common tendon
```

## Inputs

```text
theta1 = J00 / MCP-AA, deg
theta2 = J01 / MCP-FE, deg
theta3 = J02 / PIP-FE, deg
theta4 = J03 / DIP/distal-FE, deg
```

## M00 / ID1 Right-Swing Model

The moving point `D1` is computed from:

```text
A1D1 = [
  cos(theta1) * (l1 + l2 * cos(theta2 + theta_o)) - l3 * sin(theta1) - x1,
  -l2 * sin(theta2 + theta_o) - y1,
  sin(theta1) * (l1 + l2 * cos(theta2 + theta_o)) + l3 * cos(theta1) - z1
]
```

In the code, `D1` is reconstructed as the absolute moving point:

```text
D1 = A1 + A1D1
```

With the current provided `A1`, this simplifies to:

```text
D1 = [
  cos(theta1) * (l1 + l2 * cos(theta2 + theta_o)) - l3 * sin(theta1),
  -l2 * sin(theta2 + theta_o),
  sin(theta1) * (l1 + l2 * cos(theta2 + theta_o)) + l3 * cos(theta1)
]
```

## M01 / ID2 Left-Swing Model

The moving point `D2` is computed from:

```text
A2D2 = [
  cos(theta1) * (l1 + l2 * cos(theta2 + theta_o)) + l3 * sin(theta1) - x1,
  -l2 * sin(theta2 + theta_o) - y1,
  sin(theta1) * (l1 + l2 * cos(theta2 + theta_o)) - l3 * cos(theta1) + z1
]
```

The code interprets this as the mirror of the ID1 model:

```text
A2 = (-3.66, -2.25, -8.58) mm
```

Then `D2 = A2 + A2D2`, which simplifies to:

```text
D2 = [
  cos(theta1) * (l1 + l2 * cos(theta2 + theta_o)) + l3 * sin(theta1),
  -l2 * sin(theta2 + theta_o),
  sin(theta1) * (l1 + l2 * cos(theta2 + theta_o)) - l3 * cos(theta1)
]
```

## Obstacle Model

The obstacle is modeled as an infinite cylinder:

```text
axis direction = (0, 0, 1)
center         = (13, 0, 5)
radius         = 5.9 mm
height         = infinite / ignored
```

Because the cylinder axis is parallel to Z and height is treated as infinite,
avoidance is solved in the XY plane:

```text
if A1-D1 projected line does not enter circle:
    length = straight 3D distance

else:
    XY path = tangent segment + circular arc + tangent segment
    length  = sqrt(XY_path_length^2 + delta_z^2)
```

This is a geometric estimate, not a contact/friction simulation.

## M02 / ID3 PIP-Flexion Model

The M02 model has two named components:

```text
M02_length_delta =
    mcp_routing_delta(theta1, theta2)
  + pip_flexion_delta(theta3)
```

Where:

```text
theta1 = J00 / MCP-AA
theta2 = J01 / MCP-FE
theta3 = J02 / PIP-FE
```

The MCP routing part uses the user-provided point:

```text
A = (-6.66, 2.75, 4.00) mm
```

and reconstructs the moving point `D` from:

```text
AD = [
  cos(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) - l3 * sin(theta1) - x1,
  -l2 * sin(-theta2 + theta_o) - y1,
  sin(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) + l3 * cos(theta1) - z1
]
```

With the current `A`, the absolute point `D` becomes:

```text
D = [
  cos(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) - l3 * sin(theta1),
  -l2 * sin(-theta2 + theta_o),
  sin(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) + l3 * cos(theta1)
]
```

The current parameters are:

```text
l1 = 13.00 mm
l2 = sqrt(8.1^2 + 0.18^2) = 8.1020 mm
l3 = 1.50 mm
theta_o = -atan(0.18 / 8.1) = -1.2730 deg
```

The `-theta2` sign is intentional. In the current mechanism M02 crosses below
the MCP-FE axis, so increasing MCP-FE should shorten this tendon.

The PIP flexion part is:

```text
pip_flexion_delta(theta3) = -6 * theta3_rad
```

Meaning:

```text
PIP-FE increases by 1 rad -> M02 tendon length decreases by 6 mm
```

### M02 Obstacle Model

The M02 routing part avoids two infinite-Z keepout regions in the XY plane:

```text
1. Cylinder:
   center = (13.00, 0.00) in XY
   radius = 3.50 mm

2. Origin keepout:
   -5.00 < x < 5.00
   -5.00 < y < 3.70
   left-side corner radius ~= 2.00 mm
```

The current implementation approximates the origin keepout as a rectangle with
both left-side corners rounded by radius 2 mm. The routing solver builds a
visibility graph around the two obstacles:

```text
if straight XY line from A to D is clear:
    XY path = straight line
else:
    XY path = shortest visible polyline around sampled obstacle boundaries

mcp_routing_length = sqrt(XY_path_length^2 + delta_z^2)
mcp_routing_delta  = mcp_routing_length(theta1, theta2)
                   - mcp_routing_length(0, 0)
```

This is still a geometric estimate. It does not model friction, tendon
thickness contact mechanics, or a forced wrap side.

### M02 Monotonicity Notes

The current M02 model should be treated as a feedforward estimate, not as a
precise calibrated tendon-length table. A quick numerical sweep of the current
geometry shows:

```text
theta3 / PIP-FE:
  strictly monotonic decreasing
  theta3 +1 deg -> tendon length -0.10472 mm
  theta3 +10 deg -> tendon length -1.047 mm

theta2 / MCP-FE, with theta1=0 and theta3=0:
  monotonic decreasing in the checked 0..90 deg range
  theta2 30 deg -> about -1.86 mm
  theta2 90 deg -> about -8.34 mm

theta1 / MCP-AA, with theta2=0 and theta3=0:
  non-monotonic and small magnitude
  should be treated as a routing correction, not the main drive term
```

Interpretation:

```text
M02 main drive term: theta3 / PIP-FE
M02 compensation terms: theta1 and theta2 routing corrections
```

The `theta1` effect and invalid/fallback regions should be checked against
later physical measurements. If the measured tendon path always wraps a fixed
side, this shortest-path geometric model may need a forced-side routing mode or
a calibrated lookup table.

## M03 / ID4 Distal-Flexion Model

The M03 model is currently written as three additive parts:

```text
M03_length_delta =
    upper_mcp_routing_delta(theta1, theta2)
  + pip_flexion_delta(theta3)
  + dip_flexion_delta(theta4)
```

Where:

```text
theta1 = J00 / MCP-AA
theta2 = J01 / MCP-FE
theta3 = J02 / PIP-FE
theta4 = J03 / DIP/distal-FE
```

The first part is an upper-side MCP-routing estimate. M03 crosses above the
MCP-FE axis, so increasing MCP-FE should lengthen this tendon. The current
anchor and moving-point model are:

```text
M03 anchor = (-6.93, 2.75, -4.00) mm

D = [
  cos(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) - l3 * sin(theta1),
  -l2 * sin(-theta2 + theta_o),
  -(sin(theta1) * (l1 + l2 * cos(-theta2 + theta_o)) + l3 * cos(theta1))
]

l1 = 13.00 mm
l2 = sqrt(8.1^2 + 0.18^2) = 8.1020 mm
l3 = 1.50 mm
theta_o = -atan(0.18 / 8.1) = -1.2730 deg
```

The `theta2` contribution is separated from the `theta1` zero line and sign
flipped so the upper-side MCP-FE effect has the correct physical direction.

The second and third parts are linear flexion terms:

```text
pip_flexion_delta(theta3) = -3.50 * theta3_rad
dip_flexion_delta(theta4) = -4.80 * theta4_rad
```

Meaning:

```text
PIP-FE increases by 1 rad -> M03 tendon length decreases by 3.50 mm
DIP/distal-FE increases by 1 rad -> M03 tendon length decreases by 4.80 mm
```

So the linear parts are strictly monotonic:

```text
theta2 / MCP-FE:
  monotonic increasing in the checked 0..90 deg range
  theta2 30 deg -> about +1.86 mm
  theta2 90 deg -> about +8.34 mm

theta3 +1 deg -> tendon length -0.06109 mm
theta4 +1 deg -> tendon length -0.08378 mm
```

## M04 / ID5 Common Return Model

The M04 model is currently written as four additive parts:

```text
M04_length_delta =
    theta1_return_delta(theta1)
  + mcp_fe_return_routing_delta(theta2)
  + pip_return_delta(theta3)
  + dip_return_delta(theta4)
```

Current implemented parts:

```text
theta1_return_delta(theta1) = AB(theta1) - AB(0)
pip_return_delta(theta3) = -9.90 * theta3_rad
dip_return_delta(theta4) = -6.45 * theta4_rad
```

The `theta1` part uses:

```text
A = (-6.95, 5.50, 0.00) mm
B(theta1) lies on a circle:
  center = (0.00, 5.50, 0.00) mm
  radius = 7.00 mm

theta1 = 0 means B is the closest point to A:
  B(0) = (-7.00, 5.50, 0.00) mm
```

The current point model is:

```text
B(theta1) = [
  -7.00 * cos(theta1),
   5.50 + 7.00 * sin(theta1),
   0.00
]
```

and the length contribution is:

```text
theta1_return_delta(theta1) = distance(A, B(theta1)) - distance(A, B(0))
```

This term is minimum at `theta1=0` and increases for both positive and negative
theta1.

Meaning:

```text
MCP-FE increases -> M04 MCP-FE routing lengthens
PIP-FE increases by 1 rad -> M04 tendon length decreases by 9.90 mm
DIP/distal-FE increases by 1 rad -> M04 tendon length decreases by 6.45 mm
```

The `theta2` part uses the user-provided routing:

```text
fixed point A = (3.50, 5.50, 0.00) mm
moving point D lies on a circle:
  center = (13.00, 0.00, 0.00) mm
  radius = 12.11 mm
  theta2 = 0 uses theta0 = -atan(9 / 8.1)
```

The current moving point is:

```text
D(theta2) = [
  13.00 + 12.11 * cos(theta2 + theta0),
  0.00  + 12.11 * sin(theta2 + theta0),
  0.00
]
```

The current keepout approximation is an infinite-Z region:

```text
x < 17.00
-5.00 < y < 3.70
right-side x=17 boundary has r=3.00 mm rounded corners
```

This is interpreted as a horizontal band extending toward negative X. The
routing solver samples the right-side rounded boundary and finds the shortest
visible XY path around it. This assumption should be checked later, because
the phrase `x < 17 mm` makes the keepout open toward negative X.

Current sign note:

```text
theta2 / MCP-FE is monotonic increasing in the checked 0..90 deg range:
  theta2 30 deg -> about +2.11 mm
  theta2 90 deg -> about +7.18 mm

theta3 and theta4 terms are strictly monotonic decreasing:
  theta3 +1 deg -> tendon length -0.17279 mm
  theta4 +1 deg -> tendon length -0.11257 mm
```

The scripts support different wrap modes:

```text
shortest             = choose the shorter of cw/ccw when blocked
cw                   = always wrap clockwise
ccw                  = always wrap counter-clockwise
direct-if-clear-cw   = use direct path when clear, otherwise clockwise
direct-if-clear-ccw  = use direct path when clear, otherwise counter-clockwise
```

Based on the physical observation:

```text
pulling ID1/ID2 increases MCP-FE
MCP-FE increases -> ID1/ID2 flexion tendons should shorten monotonically
```

the recommended mode for these two MCP flexion tendons is:

```text
--wrap-mode direct-if-clear-ccw
```

The default remains `shortest` for comparison, but `shortest` can switch sides
near small MCP-FE angles and produce a non-physical local length increase.

## Usage

Main five-tendon feedforward entry:

```powershell
python .\tools\tendon_forward_model\tendon_length_feedforward_model.py --theta1 10 --theta2 30 --theta3 45 --theta4 60
```

JSON output from the unified model:

```powershell
python .\tools\tendon_forward_model\tendon_length_feedforward_model.py --theta1 10 --theta2 30 --theta3 45 --theta4 60 --counts-per-mm 100 --json
```

Unified grid export:

```powershell
python .\tools\tendon_forward_model\tendon_length_feedforward_model.py --grid --theta1-min -20 --theta1-max 30 --theta1-step 1 --theta2-min 0 --theta2-max 90 --theta2-step 1 --theta3-min 0 --theta3-max 90 --theta3-step 1 --theta4-min 0 --theta4-max 90 --theta4-step 1 --out .\run_data\tendon_length_feedforward_grid.csv
```

Generate decomposed 0.1-degree LUTs for MCU lookup:

```powershell
python .\tools\tendon_forward_model\generate_tendon_lut_0p1deg.py --theta1-min -20 --theta1-max 30 --theta2-min 0 --theta2-max 90 --step 0.1 --out-dir .\tools\tendon_forward_model\generated_lut_0p1deg
```

Generated files:

```text
generated_lut_0p1deg/metadata.json
generated_lut_0p1deg/tendon_lut_0p1deg_float32.npz
generated_lut_0p1deg/tendon_lut_0p1deg_int16.npz
```

The LUT is decomposed for MCU use:

```text
M00 = LUT2D(theta1, theta2)
M01 = LUT2D(theta1, theta2)
M02 = M02_LUT2D(theta1, theta2) - 6.00 * theta3_rad
M03 = M03_LUT2D(theta1, theta2) - 3.50 * theta3_rad - 4.80 * theta4_rad
M04 = LUT1D(theta1) + LUT1D(theta2) - 9.90 * theta3_rad - 6.45 * theta4_rad
```

The int16 LUT stores:

```text
int16_value = round(length_delta_mm * 1000)
```

so one integer count is 0.001 mm. The metadata records invalid masks for the
regions where the current keepout model has endpoints inside the keepout; those
entries are filled with straight-line fallback and should be treated as
calibration warnings.

### LUT Size Notes

Generated LUT size comparison:

```text
0.1 deg:
  theta1 count = 501
  theta2 count = 901
  compressed int16 npz = about 1.05 MB
  raw MCU-style arrays, including full invalid masks = about 3.62 MB

0.2 deg:
  theta1 count = 251
  theta2 count = 451
  compressed int16 npz = about 0.46 MB
  raw MCU-style arrays, including full invalid masks = about 0.91 MB
```

For ESP32-P4 integration, 0.2 deg is the safer first target. With bilinear
interpolation, the effective command remains smooth, and the runtime cost is
only a few array reads and multiply-adds per tendon. The 0.1 deg table is still
useful as an offline reference, but it is large enough that firmware integration
should first check Flash/PSRAM partitioning and whether full invalid masks are
really needed on-device.

Current generated folders:

```text
generated_lut_0p1deg/
generated_lut_0p2deg/
```

### Firmware LUT Export

The current recommended firmware candidate is the `0.2 deg` LUT. Export it to
the ESP32-P4 firmware folder with:

```powershell
python .\tools\tendon_forward_model\export_tendon_lut_to_firmware.py
```

This generates:

```text
ESP32-P4/ServoBoardMain/src/control/TendonFeedforwardLut.h
ESP32-P4/ServoBoardMain/src/control/TendonFeedforwardLut.cpp
```

The generated module provides lookup functions used by the P4 control solver:

```cpp
float lengthDeltaMm[5];
bool clamped = false;
TendonFeedforwardLut::computeLengthDeltaMm(theta1Deg,
                                           theta2Deg,
                                           theta3Deg,
                                           theta4Deg,
                                           lengthDeltaMm,
                                           &clamped);
```

The firmware-side decomposition is:

```text
M00 = LUT2D(theta1, theta2)
M01 = LUT2D(theta1, theta2)
M02 = M02_LUT2D(theta1, theta2) - 6.00 * theta3_rad
M03 = M03_LUT2D(theta1, theta2) - 3.50 * theta3_rad - 4.80 * theta4_rad
M04 = LUT1D(theta1) + LUT1D(theta2) - 9.90 * theta3_rad - 6.45 * theta4_rad
```

The generated source does not include invalid masks on-device. The invalid
counts remain exposed as constants for awareness:

```text
kM00InvalidCount
kM01InvalidCount
kM02InvalidCount
kM03InvalidCount
```

#### Invalid Region Meaning

`invalid` does not mean the LUT generation failed. It means the current
geometry/obstacle model judged that, at that sampled joint pose, the modeled
tendon endpoint or path is inside a keepout region.

Important correction:

```text
The keepout coordinates supplied by the user are zero-pose coordinates.
They are not always fixed in the global/world coordinate frame.
```

The current model therefore treats the MCP-related keepouts as moving with the
corresponding joint frame:

```text
M00/M01 cylinder:
  defined at theta1 = 0
  follows the MCP-AA theta1 frame

M02/M03 cylinder and origin keepout:
  defined at theta1 = 0
  follows the MCP-AA theta1 frame

M04 MCP-FE keepout:
  defined at theta2 = 0
  follows the MCP-FE theta2 frame
```

Implementation detail:

```text
Instead of rotating every obstacle into world coordinates, the model transforms
the tendon endpoints back into the obstacle's zero-pose local frame, then runs
the same XY keepout calculation there.
```

If a point is still marked invalid after this moving-frame correction, it means
the endpoint falls inside the keepout in that obstacle's own local frame. In
that case the tangent-wrap calculation is not physically meaningful, so the
generator:

```text
1. marks that sample in the invalid mask
2. fills the length value with straight-line fallback because invalid_policy=straight
```

So the table still has a numeric value, but that value should be treated as a
calibration warning. It is a sign that the real tendon path, obstacle radius,
axis convention, or forced wrap side may need to be corrected later with
measurement.

For the regenerated 0.2 deg LUT after the moving-frame correction:

```text
M00 invalid count = 0
M01 invalid count = 0
M02 invalid count = 18942
M03 invalid count = 27962
```

Recommended handling:

```text
early firmware test:
  use the filled 0.2 deg table without invalid masks

offline validation:
  inspect invalid masks and avoid trusting those regions too much

later calibration:
  replace suspicious invalid/fallback regions with measured data or a better
  fixed-side routing model
```

The unified model outputs one row for each tendon:

```text
length_delta_mm:
  positive -> physical tendon lengthens
  negative -> physical tendon shortens

motor_abs_delta_counts:
  preview conversion using motor_abs increases -> tendon shortens
  motor_abs_delta_counts = -length_delta_mm * counts_per_mm
```

The older per-motor files remain useful for single-tendon debugging and
geometry checks.

M00/ID1 single point:

```powershell
python .\tools\tendon_forward_model\m00_id1_right_mcp_tendon_model.py --theta1 0 --theta2 0
```

Left-swing ID2 single point:

```powershell
python .\tools\tendon_forward_model\m01_id2_left_mcp_tendon_model.py --theta1 0 --theta2 0
```

PIP-flexion ID3 single point:

```powershell
python .\tools\tendon_forward_model\m02_id3_pip_flex_tendon_model.py --theta1 0 --theta2 0 --theta3 30
```

Distal-flexion ID4 single point:

```powershell
python .\tools\tendon_forward_model\m03_id4_dip_flex_tendon_model.py --theta1 0 --theta2 0 --theta3 30 --theta4 30
```

Common-return ID5 single point:

```powershell
python .\tools\tendon_forward_model\m04_id5_return_tendon_model.py --theta1 0 --theta2 0 --theta3 30 --theta4 30
```

Recommended physical mode with tendon radius padding:

```powershell
python .\tools\tendon_forward_model\m00_id1_right_mcp_tendon_model.py --theta1 0 --theta2 30 --radius-padding 0.5 --wrap-mode direct-if-clear-ccw
python .\tools\tendon_forward_model\m01_id2_left_mcp_tendon_model.py --theta1 0 --theta2 30 --radius-padding 0.5 --wrap-mode direct-if-clear-ccw
```

JSON output:

```powershell
python .\tools\tendon_forward_model\m00_id1_right_mcp_tendon_model.py --theta1 10 --theta2 30 --json
```

Left-swing ID2 JSON output:

```powershell
python .\tools\tendon_forward_model\m01_id2_left_mcp_tendon_model.py --theta1 10 --theta2 30 --json
```

PIP-flexion ID3 JSON output:

```powershell
python .\tools\tendon_forward_model\m02_id3_pip_flex_tendon_model.py --theta1 10 --theta2 30 --theta3 30 --json
```

Distal-flexion ID4 JSON output:

```powershell
python .\tools\tendon_forward_model\m03_id4_dip_flex_tendon_model.py --theta1 10 --theta2 30 --theta3 45 --theta4 60 --json
```

Common-return ID5 JSON output:

```powershell
python .\tools\tendon_forward_model\m04_id5_return_tendon_model.py --theta1 0 --theta2 45 --theta3 30 --theta4 60 --json
```

Grid export:

```powershell
python .\tools\tendon_forward_model\m00_id1_right_mcp_tendon_model.py --grid --theta1-min -20 --theta1-max 30 --theta1-step 1 --theta2-min -20 --theta2-max 90 --theta2-step 1 --out .\run_data\m00_id1_right_mcp_tendon_length_grid.csv
```

Left-swing ID2 grid export:

```powershell
python .\tools\tendon_forward_model\m01_id2_left_mcp_tendon_model.py --grid --theta1-min -20 --theta1-max 30 --theta1-step 1 --theta2-min -20 --theta2-max 90 --theta2-step 1 --out .\run_data\m01_id2_left_mcp_tendon_length_grid.csv
```

PIP-flexion ID3 grid export:

```powershell
python .\tools\tendon_forward_model\m02_id3_pip_flex_tendon_model.py --grid --theta1-min -20 --theta1-max 30 --theta1-step 1 --theta2-min 0 --theta2-max 90 --theta2-step 1 --theta3-min 0 --theta3-max 90 --theta3-step 1 --out .\run_data\m02_id3_pip_flex_tendon_delta_grid.csv
```

Distal-flexion ID4 grid export:

```powershell
python .\tools\tendon_forward_model\m03_id4_dip_flex_tendon_model.py --grid --theta1-min -20 --theta1-max 30 --theta1-step 1 --theta2-min 0 --theta2-max 90 --theta2-step 1 --theta3-min 0 --theta3-max 90 --theta3-step 1 --theta4-min 0 --theta4-max 90 --theta4-step 1 --out .\run_data\m03_id4_dip_flex_tendon_delta_grid.csv
```

Common-return ID5 grid export:

```powershell
python .\tools\tendon_forward_model\m04_id5_return_tendon_model.py --grid --theta1 0 --theta2-min 0 --theta2-max 90 --theta2-step 1 --theta3-min 0 --theta3-max 90 --theta3-step 1 --theta4-min 0 --theta4-max 90 --theta4-step 1 --out .\run_data\m04_id5_return_tendon_delta_grid.csv
```

If `--theta2-min/--theta2-max` are omitted during M02 grid export, the grid
uses the single `--theta2` value. This is useful when you only want to sweep
MCP-AA and PIP-FE.

## Current Defaults

```text
A1 = (-3.66, -2.25, 8.58) mm
A2 = (-3.66, -2.25, -8.58) mm
l1 = 13.00 mm
l2 = sqrt(8.1^2 + 2^2) = 8.3433 mm
l3 = 4.50 mm
theta_o = -atan(2 / 8.1) = -13.8750 deg
cylinder center = (13.00, 0.00, 5.00) mm
cylinder radius = 5.90 mm
```

## Still Needed From User

Before this can become a firmware feedforward model, confirm:

1. `theta1` sign: J00 left swing positive or right swing positive.
2. `theta2` sign: J01 flexion positive or extension positive.
3. Whether `l1=13.00 mm` is still correct for M00/ID1 and M01/ID2.
4. Whether the cylinder should be treated as truly infinite in Z.
5. Whether wrapping direction should always choose shortest path, or force a fixed side.
6. Whether output should stay as `length_mm`, or convert to `motor_abs counts`.
7. Counts/mm conversion for ID1 if firmware will use motor counts.
8. Whether M02 origin keepout should have both left corners rounded, or only one.
9. Whether M02 should use shortest visible path, or force a fixed physical wrap side.

## Important Sign Convention

The current hardware convention is:

```text
motor_abs increases -> tendon shortens/tightens
motor_abs decreases -> tendon lengthens/releases
```

So if the model outputs physical tendon length `L_mm`, motor feedforward should
usually use:

```text
motor_ff_counts = -(L_target - L_zero) * counts_per_mm
```

unless the user provides a direct motor-count model.
