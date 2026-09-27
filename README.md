# Override competition robot

Single V5 competition program using local ozton 0.4.0. Build with `./build.sh`;
upload with `./upload.sh` after hardware checks.

Edit configuration constants at top of `src/main.rs` before running. Current
sensor wiring: IMU P15, forward tracking wheel P1, sideways wheel P10. IMU sensor
Y points forward and Z points down. Raw heading decreases on a counterclockwise
turn; tracking converts it to counterclockwise-positive. Drivetrain
ports (left/right viewed from the back): left 11 W P11/P13, left 5.5 W P12;
right 11 W P20/P18, right 5.5 W P19. Lift motors are P17 (reversed) and P14;
the pneumatic claw relay is ADI A. Radio is on P21. Tracking sensor positions in
meters use robot X forward, Y left: forward wheel (+2 in, +0.25 in), sideways
wheel (-3 in, 0 in). Forward wheel uses its Y
position; sideways wheel uses negative X position. Tracking ratio is 1:1, and
both sensors increase for positive travel. Confirm tracking wheel diameter
(2 in remains provisional), 5.5 W compound gear ratio, lift cartridge, and
claw polarity on a safely supported robot. L1 toggles the pneumatic claw;
closed is ADI A high and open is ADI A low. Driver commands run at 10 ms intervals
without a drive command ramp. A full joystick command applies each drive motor's
full rated voltage in playback mode. Recording mode limits drive and turn inputs
to 45% for controlled route capture. Anti-tip reads IMU roll relative to its startup level reading,
starting correction at about 9 degrees and limiting correction to 35% drive power.
Keep the robot level during startup and verify correction direction on the robot.
Both mirrored
lift motors receive one shared command: 75% up, 33% down. Command ramps up at
150% per second and down at 90% per second (about 0.5 and 0.37 seconds to full
power). Releasing a button stops and holds immediately; reversing direction
starts a new ramp. There is no per-motor synchronization or end-stop slowdown.
Position limits remain active when both motor encoders can be read.

Playback selector includes `Move Off Wall`: drives both sides forward at 35%
power for 750 ms, then coasts. Select it before autonomous to use this built-in
route; adjust power and time constants in `src/main.rs` after field testing.
The `Red Toggle` and `Blue Toggle` routes are currently hidden because the
toggle hits a hard stop before the first 180-degree motor command finishes.
Measure the safe P16 motor encoder travel in the needed direction before
enabling either route. Their implementation stops the motor between half turns
and aborts if encoder progress stalls for 400 ms, but that cutoff is only a
backup for an unexpected jam.

At boot, touch upper screen half for recording or lower half for playback.
The choice appears on screen immediately; quick taps during menu drawing count.
Without touch, playback mode starts after four seconds. Both modes retain
ozton's route selection screen. Driver controls: left Y drive, right X turn,
R1/R2 lift, L1 claw toggle, X auxiliary motor on P16.
Recording requires
a calibrated IMU and ready odometry; it is discarded if tracking fails.
Playback uses recorded pose with
odometry correction and stops if tracking fails.
Recordings from motor-claw builds use a different frame format; re-record after switching.
