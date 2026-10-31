# Override competition robot

V5 competition program using local ozton 0.4.0. Run `./check.sh` for formatting,
mechanism tests, and the V5 target check. Build with `./build.sh`; upload with
`./upload.sh`. Build and upload honor `CARGO_TARGET_DIR`.

## Hardware

- Left drivetrain: 5.5 W green P5, 11 W blue P4/P3; all reversed.
- Right drivetrain: 5.5 W P20, 11 W blue P19/P18; all forward.
- Cascade lift: 11 W red P1.
- Main intake: 5.5 W P10; forward draws pins inward.
- Match-load roller: 11 W green P2, reversed motor direction; logical reverse assists inward loading.
- Intake pivot: 5.5 W P9, reversed; positive controller commands raise the pivot.
- Rotation sensor: P8 on the 12T motor shaft. Raw positive rotation means downward;
  `Direction::Reverse` exposes positive upward travel to the controller.
- No inertial sensor, tracking wheels, odometry, or anti-tip control.

## Startup and angle references

Start with the cascade at its bottom and the intake physically flat down in its
stowed pose. The lift encoder and pivot rotation sensor are referenced to zero.
This is a startup reference, not automatic mechanical homing. Successful pivot
zeroing prints `Pivot rotation sensor zeroed: 0 degrees`.

P8 uses accumulated `position()`, never wrapped `angle()`. A stowed absolute
reading near 0 or a full turn does not change startup zeroing. After zeroing,
360 shaft degrees means a complete shaft turn, not another stowed position.
The 12T shaft drives the 48T intake gear: 360 shaft degrees / 4 = 90 intake
output degrees. Normal cascade targets remain within 0–90 output degrees. Manual scoring trim
allows up to 45 additional degrees, capped at 135 output degrees in live control
and playback. P8 is the sole source of pivot position.

The pivot follows twice the original 145/135 shaft-to-lift ratio, 60% faster
than the preceding setting: -67.5 lift encoder degrees gives 145 upward pivot
shaft degrees before alignment. A one-tooth upward offset on the 12T gear adds
30 shaft degrees, or 7.5 intake output degrees, to the target. Sensor zero stays
at the physical stowed pose. The normal cascade target still clamps at 90 degrees,
so the offset raises the bottom target to 7.5 output degrees and reaches the
90-degree cap slightly earlier. It then
holds that angle as the cascade rises. Lowering follows the same mapping back.
These geometric settings and control gains still need a physical robot check;
software tests cannot measure mounting alignment, backlash, or stopping distance.

The pivot uses smoothed velocity feedforward and PID position feedback while
moving. Speed increases at at most 900 RPM/s, capped at 200 RPM. Speed reductions
and endpoint caps apply immediately. Once settled, P9 latches active `BrakeMode::Hold` once. External velocity control
does not restart for sensor drift or load deflection. The motor maintains its
held position; this removes the passive-brake drop followed by repeated catch-up
commands. Only a target change greater than 6 shaft degrees (1.5 output degrees)
resumes movement. Small target changes remain filtered before velocity
feedforward. Cascade release also uses Hold. Loaded holding still needs a
physical robot check.

## Driver controls

- Left Y drives; right X turns. Normal drivetrain output is 70% of rated voltage.
- Hold Y for full drivetrain output. Recording input is capped at 45% unless Y
  is held. Release, stops, and mode changes clear the boost.
- R1 raises the cascade; R2 lowers it. Release or press both to Hold.
- L1 runs P10 forward. L2 reverses P10 to eject. Release or press both for passive Brake. P10 does not use active position Hold.
- P2 follows L1/L2 inward/outward, with measured cascade travel within
  50 motor degrees of bottom and pivot within 10 output degrees of stowed.
  L2 reverses P2 from its loading direction. Otherwise P2 uses passive Brake, except for the departure pulse below. Both references and both valid
  position readings are required for P2. The same rule applies during playback.
- Starting a new raise within 50 lift degrees of bottom runs P2 and P10 inward for
  600 ms, even without L1. The pulse continues after leaving the loading region.
  Holding Raise does not retrigger it. Release, lowering, ejection, linked faults,
  and explicit stops cancel it. Live control and playback use the same behavior.
- Hold X (or Up) for extra upward scoring tilt at 45 output degrees/s, up to
  45 extra degrees. At full extension this reaches 135 output degrees.
  Release X to latch Hold at that angle, including while L2 reverses the intake.
  Hold A (or Down) to remove extra tilt at 45 output degrees/s, returning to the
  normal cascade-following angle in at most one second. Release or press both to
  retain trim. Normal cascade travel still ends at 90 degrees; only scoring trim
  extends beyond it. Restart resets trim. Verify physical clearance for 135 degrees.

## Cascade limits and smoothing

Lift encoder range is 0 degrees at bottom to -1100 degrees at maximum extension.
Negative voltage raises; positive voltage lowers. Internal control uses positive
upward travel. The previous 1800-degree mechanical-stop magnitude remains in
calibration; the stricter 1100-degree software travel limit takes precedence.

Upward power now caps at 100% (12 V on the 11 W cascade motor), up from 90%.
The ramp remains 450%/s, reaching the new cap in about 0.22 seconds. It tapers
from 100% at -1000 encoder
degrees toward 12% before -1100, then stops and holds. Descent ramps to at most
35% over 0.2 seconds and tapers toward 8% within 180 motor degrees of bottom.
Endpoint power reductions apply immediately. Commands beyond either endpoint
are blocked; commands back into range remain available. Live and playback share
these limits. Playback stores raw signed lift positions and clamps its targets
to [-1100, 0]. Position error on the pivot does not throttle cascade voltage.

## Recovery and autonomous

Missing devices do not abort startup or recorded playback. Outputs are retried
on control frames. A lift or pivot read/write failure attempts to hold both linked
mechanisms. P10 continues independently. A missing reference or invalid pose
stops the conditional P2 roller. Intake and roller writes are attempted
independently; failure of either does not prevent the other output.
Established references are not re-zeroed after temporary disconnects.
Stops attempt Hold on the lift and P9 pivot; P2 and P10 use passive Brake.
P10 braking slows the roller without actively correcting its encoder position,
removing position-hold hunting but providing less static pin retention.

At boot, touch the upper screen half for recording or lower half for playback;
playback is selected after four seconds. Both retain the route selector.
`Move Off Wall` drives forward at 35% for 750 ms, subject to the normal 70%
drivetrain output scale, then coasts.

Re-record routes saved with old lift signs, gearing, or trim frame layouts.
Remove incompatible route files and their `.routes` entries before selection.
Changes are not uploaded automatically.
