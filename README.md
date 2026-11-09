# Override competition robot

Modular V5 competition program using local ozton and vexide 0.9.0-beta.1.
All robot tuning and wiring lives in `src/config.rs`. The existing ports, motor
cartridges, directions, drive scaling and anti-tip settings are preserved.

Run inside the Nix development shell:

- `./check.sh`: formatting, host tests and V5 target check.
- `./build.sh`: release build.
- `./upload.sh`: build and upload to slot 1.

These scripts honor `CARGO_TARGET_DIR`. Building does not upload firmware.

## Modules and references

- `config.rs`: wheel size, gearing, port assignments, motor types/directions,
  lift/pivot limits, follow ratio, control gains, timing and autonomous targets.
- `forward_auton.rs`: relative wheel travel feedback, bounded output, static
  friction compensation, overshoot correction and settling on both sides.
- `autons/score.rs`: the premade scoring route, written as ordinary robot commands.
- `auton_commands.rs`: awaited drive/wait commands, retained mechanism targets,
  feedback checks, per-command deadlines and mode cancellation.
- `auton_selection.rs`: touch route selection, recording choices and save retry.
- `recording.rs`: bounded capture, SD storage validation and interpolated playback.
- `mechanism_control.rs`: pure lift/pivot coordination and endpoint limits.
- `pivot_control.rs`: continuous P8 position control with damping and anti-windup.
- `mechanisms.rs`: device reads/writes, lift ramping, intake/roller coordination,
  recording integration and linked fault handling.
- `low_score.rs`: the independent B-button scoring cycle.
- `resilient_drive.rs`: tolerant driver outputs and strict autonomous outputs.
- `recovery.rs`: startup references that survive temporary disconnections.

The structure follows [autons](https://github.com/vexide/autons)' separation of
robot and autonomous route, and [evian](https://github.com/vexide/evian)'s separate
motion/controller/settling model. Control primitives come from the existing
local ozton implementation derived from evian. This program does not add autons
or evian as dependencies: autons provides route selection, not distance control.
The screen offers the built-in route, Disable, recording and saved routes. The competition loop ticks the
route and mechanisms together. Each awaited premade command services the same
mechanism/drive feedback and checks mode every 10 ms; a mode change cancels it.
Recorded route playback retains its independent cursor and file format.

## Editing premade routes

Each premade route is a separate async function in `src/autons/`. The menu calls
`autons::run(&mut robot)` once per autonomous entry. The scoring function is a
short sequence such as:

```rust
robot.set_lift_height(raised)?;
robot.follow_pivot();
robot.drive_forward(AUTON_FORWARD_METERS).await?;
robot.set_pivot_angle(90.0)?;
robot.wait_for_mechanisms().await?;
robot.set_lift_height(lowered)?;
robot.wait_for_mechanisms().await?;
robot.start_eject();
robot.wait(AUTON_RELEASE_TIME).await?;
```

Lift targets use positive-up motor encoder degrees; pivot angles use output
gear degrees; distances use meters. `set_lift_height`, `set_pivot_angle` and
`start_eject`/`stop_eject` set retained targets. They continue updating throughout the next
awaited drive or wait command, so lifting and driving happen together.
`wait_for_mechanisms` waits for measured arrival; `wait` holds targets for a
specified duration. `drive_backward` takes a positive distance. All numeric
route settings remain in `config.rs`. No per-route phase enum or tick dispatcher
is required. `Commands` is implemented by the real `Robot` and by test hardware.
Recorded routes do not use these command functions. The command return type is
`AutonResult<T = ()>` with typed `AutonError` failures. SDK motor errors retain
their original source and port; the screen still uses the same status text.
Target validation and mutations are centralized in `AutonControl`
methods; stop/new-run functions clear retained targets. All awaited commands use
one `CommandClock` for intervals, timeout checks and cancellation. Both forward
and retreat configuration distances are nonnegative; the command supplies direction.

## Hardware

| Device | Port | Configuration |
| --- | --- | --- |
| Left drive | P5 / P4 / P3 | 5.5 W / 11 W blue / 11 W blue; reversed |
| Right drive | P20 / P19 / P18 | 5.5 W / 11 W blue / 11 W blue; forward |
| Cascade | P1 | 11 W red; negative voltage raises |
| Main intake | P10 | 5.5 W; forward draws pins inward |
| Match-load roller | P2 | 11 W green, reversed; logical reverse loads inward |
| Intake pivot | P9 | 5.5 W, reversed; positive commands raise |
| Rotation sensor | P8 | 12T shaft, reversed to expose positive upward travel |
| Inertial sensor | P7 | Rotated 180° about X: +Y rearward, +Z upward |

Static port ownership makes duplicate assignments a compile error. The 11 W drive
motors use 36:48 wheel gearing; the 5.5 W drive motors use 1:1 gearing. Wheel
diameter is 2.25 inches. There are no tracking wheels. Autonomous measures wheel
travel using P5/P20 and the configured wheel size/ratios, so slip affects actual
field distance.

## Startup and pivot measurement

Start with the lift at bottom, intake stowed and robot level and stationary.
Startup references the lift encoder and accumulated P8 `position()` to zero,
then calibrates P7. This is a known starting pose, not mechanical homing.
Missing references retry before linked motion; an established reference is never
re-zeroed on a temporary disconnection. Invalid configuration prevents startup
motion. The display shows route choices and a short status; no firmware slogan.

P8 is the only pivot feedback source. P9 motor position does not determine pivot
angle. The 12T shaft drives the 48T output, so four P8 degrees equal one output
degree. Normal following reaches **90° output / 360° P8**, without the old
7.5° compensation. Following uses measured lift travel at the configured constant
ratio (290 P8 degrees per 135 lift encoder degrees) until the pivot reaches 90°,
then holds that angle. Lowering restores following and returns toward stowed.

A live, held X button permits **120° output / 480° P8**. The extra 30° is
measured at the 48T output; it is 120 additional P8 degrees. Releasing X removes
the extra allowance immediately and returns to the ordinary target. The B scoring
routine has its own approved +5° output tilt, capped at **95° output / 380° P8**.
It restores the original pivot angle when returning. Up/A and ordinary autonomous
overrides remain capped at 90°. Recorded B poses retain the 95° B limit.

P8 controls pivot voltage throughout movement and holding, including correction
for load deflection. Software bounds targets and blocks outward voltage at the
measured limits. Physical overshoot, backlash, a slipped gear or incorrect startup
pose cannot be excluded by software. P8 cannot observe motion downstream of its
shaft. Verify actual output angle, clearance and stopping behavior on the robot.

## Autonomous route

While competition is disabled, tap a row on the Brain screen:

- **Disable**: no autonomous motion. This is the startup default.
- **Raise / Score / Retreat**: run the built-in scoring route.
- **Record New Route**: record the next driver period, up to 15 seconds.
- **Play: Route N**: replay a saved recording in the next autonomous period.
- **Record over: Route N**: record a replacement for that route.
- **Retry Save**: shown when an SD write failed and the capture remains in memory.

The original dark selector colors and five-row layout are retained. Use Prev/Next
for more routes. Selection uses screen taps only; B remains low-score during
driver control. Holding or dragging a touch does not repeatedly change selection.
Selections are locked while enabled. No startup timeout or controller confirmation
is required. Selection persists across mode changes, not program restarts.

## Recording and saved playback

Insert an SD card. With competition disabled, select Record New Route (or Record
over), place the robot at the intended start pose, then enter driver control and
drive the route. Disable to stop and save. At 15 seconds, capture stops while
ordinary driver control continues; saving still waits for Disabled. Recording
captures drive commands, boost, lift encoder position, P8 pivot position and the
actual intake and match-load roller commands, including the low-score cycle. Missing device feedback or
a controller fault discards the capture instead of saving an incomplete motion.

Saving selects the new **Play** row automatically. Return the robot to its start
position and heading, with lift and pivot matching the recorded start pose, then
enable autonomous. The program checks the starting lift/P8 pose; wheel slip and
field placement require physical alignment because there is no odometry.
Continuous drive, lift and pivot targets interpolate between samples. Discrete
intake/boost changes stay at their recorded timestamps. Recorded pivot travel
remains limited to 90 degrees except identified B scoring poses, which allow
95 degrees. A live held X alone permits the full 120 degrees.

The SD card stores numbered `.route` files and an append-only `.routes` index.
Route replacements write fresh data, sync and validate it before committing the
index entry; they never truncate the previous route. This avoids unsupported
VEXos rename operations. Failed saves retain capture in RAM for **Retry Save**;
restarting loses that pending capture. Old-format recordings are rejected with an
incompatible-file status. Back up the card before manually removing old versions.
Storage is bounded to 32 displayed routes, 1600 samples per capture, a 2 MiB
route-file limit and a 64 KiB index. File I/O occurs only after outputs stop.

The route starts once per autonomous mode entry. Disabled/driver transitions,
feedback faults, motor output faults and deadlines cancel it. Failed stops continue to
retry after completion or failure; successful mechanism holds are not reissued
on every tick, preserving their captured hold position; the route cannot silently restart while still
in autonomous mode.

1. Start driving forward **12 inches** while raising the lift toward 25% of its
   configured usable travel. Pivot follows measured lift at its configured rate
   and stops at 90°. After both wheels settle, the drive brakes and a separate
   alignment stage holds the raised lift and flat pivot. Later wheel drift cannot
   reopen the completed forward-travel gate.
2. Brake the drive. Lower the lift **35 encoder degrees** while keeping the pivot
   flat at 90°. Wait for the lift and pivot to settle. Autonomous arrival tolerances
   are 10 lift encoder degrees and 4 pivot output degrees; the actual mechanism
   controllers continue correcting to their original tighter targets.
3. Eject for **300 ms** at the lowered height to start releasing the pin.
4. Raise back to the previous lift height while continuing to eject for at least
   **1.5 seconds**. Wait for the lift and pivot to settle before retreating.
5. Stop ejection and reverse **8 inches**, maintaining the raised lift and flat
   pivot. Brake drive and hold mechanisms when complete.

The 35-degree lowering amount is initial tuning for “lower a bit”, not a measured
pin/tower dimension. Change `AUTON_LOWER_DEGREES` and `AUTON_RAISE_FRACTION` in
`config.rs` to match the mechanism and tower. Defaults correspond to lift targets
-237.5 and -202.5 motor encoder degrees. Ejection has no pin-presence feedback.

Drive control corrects signed distance error, including overshoot, and requires
both sides to remain within 5 mm and 0.025 m/s for 150 ms. Output ranges from 12%
to 35% outside the position window, before the existing 70% drive scale and
anti-tip correction. A stage timeout is a failure, never successful arrival.
Individual awaited commands time out after 8 seconds; the entire route after
15 seconds. Delays over
100 ms abort autonomous. Departure intake/roller assistance is suppressed during
the route so scoring never starts with an unintended inward pulse.

## Driver controls

- Left Y drives, right X turns. Normal output is 70%; hold Y for full output.
- R1 raises the cascade; R2 lowers. Release or press both to hold.
  Holding R2 returns the lift to its original bottom reference and the pivot to
  its original closed reference, including after autonomous. R2 takes priority
  over held X/placement buttons so they cannot prevent closure. Release X when
  releasing R2 to keep the pivot closed.
- L1 intakes; L2 ejects. Release or press both for passive brake.
- Hold X for the 120° pickup override. Release to return to ordinary control.
- Up or A selects 90° placement. Down restores measured-lift following. Lowering
  also restores following. Conflicting placement/follow buttons retain selection.
- P2 follows intake direction only within 50 lift degrees of bottom and 10 output
  degrees of stowed, with valid position feedback.
- A new manual raise near bottom runs P2/P10 inward for 600 ms. Holding Raise does
  not retrigger it. Release, lowering, ejection, faults and stops cancel it.

B captures the current height and pivot angle, lowers to **5 lift encoder degrees
above bottom** (previously 25), and tilts the pivot **+5° at the 48T output**, up
to **95°**. It ejects for 1.5 seconds, then returns to the captured height and
original pivot angle. If already below the low target, it does not raise before
ejection. `LOW_HEIGHT_ABOVE_BOTTOM_DEGREES` and `B_PIVOT_EXTRA_OUTPUT_DEGREES` affect
only B; autonomous retains its separate 35-degree lowering amount. Each travel leg times out after 8 seconds. Press B again
or use manual mechanism controls to cancel. Drive stays manual throughout.
Controller read failure stops outputs and cancels automatic mechanism cycles.
Stops, mode changes and faults clear retained scoring/placement targets without
re-zeroing either encoder. Autonomous adds no persistent driver-control offset.
The route menu uses the original selector layout without a version header.

Cascade software travel remains 0 to -950 motor encoder degrees. Existing ascent
and descent ramps and endpoint tapering remain active. Autonomous uses the same
limits and ramp with configurable position tolerance and a friction floor near
its target.

## Anti-tip and fault behavior

Driver and active autonomous drive share live roll correction relative to startup
level, including angle wrapping. Correction starts beyond 0.16 radians and is
capped at 35% before drive scaling. PID remains 1.35/0/0.03 with polarity -1.
Calibration failure disables anti-tip; unavailable live readings produce no
correction for that frame. Explicit stops brake drive instead of applying tilt
correction. Verify physical correction direction on hardware.

Driver control attempts all available drive motors and retries failed writes.
Autonomous aborts on drive write failure or missing P5/P20 feedback. Lift/P8
faults attempt hold on both linked mechanisms and brake P2. P10 remains independent
in manual control. Scripted autonomous faults stop every output. A disconnected
motor cannot receive a software brake command until communication recovers.

Host tests exercise route order, concurrent mechanisms, settling, overshoot,
X-only extra travel, P8 feedback, output limits and device faults. Software tests
cannot guarantee scoring accuracy or that hardware will never fail. Verify the
startup pose, loaded lift/pivot gains, wheel travel and clearance at reduced power
before running the full route on the field.

## Debug terminal logs

Run `./debug.sh` inside `nix develop`. It builds the **debug** profile, uploads it
to slot 1, starts the program and displays its serial output. The same output is
saved to `debug-run.log` (ignored by Git). `DEBUG_LOG_FILE=/tmp/robot.log ./debug.sh`
chooses another log file. Stop the robot through competition controls before
leaving the terminal; closing a terminal alone is not a robot stop command.
The script is supplied for operator use and is not run automatically.

Paste the log from startup through the test and a few seconds after stopping.
Label what you physically observed: short travel, missed pin, pivot jiggle, etc.
The program cannot measure pin contact or wheel slip. Do not trim `DBG,FAULT`,
`DBG,EVENT` or `DBG,LOOP` lines; those establish timing and fault context.

Debug samples default to **5 Hz**, controlled by `DEBUG_SAMPLE_INTERVAL` in
`config.rs`. Events report mode/selection/stage changes and save/load results.
`seq` groups lines from a sample; `t_ms` is program uptime in milliseconds.
`NaN` means missing feedback or no active target, never a measured zero.
Boolean values are `0` or `1`.

- `DBG,LOOP`: control interval and maximum interval since the previous sample.
  Logging overhead appears in the next interval; `max_dt_ms` reveals stalls.
- `DBG,STATE` / `DBG,INPUT`: competition mode, selected/active route, capture/save
  flags, controller/IMU health, startup-relative roll in degrees, stick inputs
  and individual buttons.
- `DBG,AUTON`: total route time, retained lift/pivot/ejection targets, relative
  left/right wheel travel and distance errors in meters. Command-start events
  identify drive/wait operations. `DBG,PLAYBACK` reports cursor and route timing.
- `DBG,DRIVE`: input commands in [-1,1], scale, signed wheel revolutions and
  integrated encoder travel in meters, write-failure state and cumulative count.
  Commands are **before anti-tip and output scaling**, not measured voltage.
- `DBG,MECH`: raw lift motor degrees, positive-up lift degrees and target/error,
  P8 shaft degrees and target/error, 48T output degrees, requested volts,
  hold/reference flags, intake/roller directions and fault count. A manual lift
  has no position target, so its target/error is `NaN`.
- `DBG,MOTOR` / `DBG,DRIVE_MOTOR`: measured volts, current in amps, temperature
  in °C, and drive motor speed in RPM. Ports distinguish all ten motors.
  Requested volts are separate from measured volts; firmware Hold can supply
  voltage even when the software request is zero. SDK motor readings use its
  polarity convention; compare sign with the recorded wiring directions.
- `DBG,RECORD`: target, sample count and captured duration. Save/load events
  include logical route ID, physical file ID, frame count and duration.

Mechanism fault codes: M01 invalid calibration; M02 lift/P8 read missing;
M03 reference or nonfinite feedback/target; M04 control gap over 100 ms;
M05 low-score timeout; M06 invalid pivot override; M07 coordination rejected;
M08 pivot controller rejected input; M09 pivot write failed; M10 lift write
failed; M11 intake write failed; M12 roller write failed. Counts accumulate per
failed control attempt; events print when the fault code changes and on recovery.
Drive faults also include the SDK `PortError` with its port number.

Numeric diagnostics compile out of **release** builds with `#[cfg(debug_assertions)]`:
there are no telemetry sensor reads, diagnostic timers, counters, formatting,
serial writes or diagnostic-only IMU reference clones. The startup banner and
robot status/initialization terminal messages are debug-only too. The screen,
control feedback, deadlines and safety checks still run normally.

Numeric diagnostics are enabled automatically by the debug profile. `./upload.sh` continues
to build release firmware. Use `./debug.sh`, not `./upload.sh`, for a log test.
