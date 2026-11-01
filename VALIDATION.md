# Validation status

## Verified in software

- Touch selection offers Disable, the scoring route, Record New Route, saved
  playback and replacement targets. Selection is locked while enabled. There is
  no controller-button selection and no firmware slogan.
- The premade route executes through a test robot: lift targets precede forward
  travel; lower/arrival precedes release; lifting occurs while ejecting; retreat
  follows release. A failed alignment command prevents all subsequent commands.
- Mechanism simulation exercises the route targets, no inward roller pulse,
  ejection and the driver-only return to starting references.
- Recording tests cover capture limits, actual roller/intake commands, boost,
  smooth continuous interpolation and discrete command timing.
- Storage tests capture, save, reload and replay a route; replace it under the
  same logical ID without truncating previous data; reject corruption; ignore
  incomplete index commits; and retry after a failed save.
- Wheel arrival and mechanism alignment are separate awaited commands. Pose
  checks accept configured loaded offsets and reject missing/large errors.
  Shared guards reject mode cancellation, timeouts and control delays.
- B-only tuning lowers from 25 to 5 lift encoder degrees above bottom and adds
  5 output degrees of pivot tilt, capped at the approved 95 degrees. Return restores
  the original pivot angle. Physical goal contact still requires a robot test.
- Driver-only R2 return after autonomous reaches the original lift/pivot
  references without re-zeroing. Stops clear retained scoring targets.
- Successful mechanism Hold commands are not reissued every tick. Failed stops
  retry. Joystick center noise is filtered by the configured deadband.
- Host tests, V5 Clippy with warnings denied and V5 optimized build pass.

## Still requires the physical robot

Software evidence cannot prove loaded stability, physical travel, scoring contact
or SD-card behavior on the actual Brain. No connected V5 serial device was
available in this environment. Firmware has not been uploaded.

1. Start with cascade down, pivot closed and robot level. Confirm the touch menu
   selects each route and Disable suppresses autonomous motion.
2. Record a short driver route using lift, intake and roller; disable to save.
   Restart, select the saved route and replay from the same starting pose.
3. Run the scoring route and measure actual forward 12-inch/reverse 8-inch
   travel. Confirm the configured 35 lift-encoder-degree lowering fits the pin.
4. After autonomous, hold R2 until cascade is down and pivot is closed; release
   and verify subsequent controls respond normally.
5. With normal load, observe stationary lift/pivot at bottom, 90 degrees and
   after X release. Observe the drive at rest and near distance targets. Record
   P8/target/voltage telemetry if oscillation remains; tune `config.rs` from those
   measurements rather than guessing gains.

Production and no-jiggle acceptance remain unproven until these physical checks
pass. Existing simulation results are not a substitute for those observations.
