# Override competition robot

Single V5 competition program using local ozton 0.4.0. Build with `./build.sh`;
upload with `./upload.sh` after hardware checks.

Edit configuration constants at top of `src/main.rs` before running. Current
sensor wiring: IMU P15, forward tracking wheel P1, sideways wheel P10. Drivetrain
ports (left/right viewed from the back): left 11 W P11/P13, left 5.5 W P12;
right 11 W P20/P18, right 5.5 W P19. Lift motors are P17/P14 and the pneumatic
relay is ADI A. Radio is on P21. Supply
tracking sensor positions in meters using robot X forward, Y left; forward
wheel uses its Y position and sideways wheel uses negative X position.
Confirm rotation sensor signs, 5.5 W compound gear ratio, lift cartridge,
claw polarity, and anti-tip polarity on a safely supported robot.

At boot, touch upper screen half for recording or lower half for playback.
Without touch, playback mode starts after four seconds. Both modes retain
ozton's route selection screen. Driver controls: left Y drive, right X turn,
R1/R2 lift, L1 claw toggle. Recording requires a calibrated IMU and ready
odometry; it is discarded if tracking fails. Playback uses recorded pose with
odometry correction and stops if tracking fails.
