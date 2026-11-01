//! Evian-style relative motion: feedback controller, bounded output, settling and timeout.
//! The route owns timing; this module knows only wheel travel in meters.
use crate::config::*;
use ozton::control::loops::{Feedback, Pid};
use std::time::Duration;

pub struct StraightMotion {
    controllers: [Pid; 2],
    previous: Option<(f64, f64)>,
    settled: Duration,
}

pub struct MotionOutput {
    pub drive: (f64, f64),
    pub settled: bool,
}

impl StraightMotion {
    pub fn new() -> Self {
        let (kp, ki, kd) = AUTON_DRIVE_PID;
        let mut controller = Pid::new(kp, ki, kd, None);
        controller.set_output_limit(Some(AUTON_DRIVE_MAX_COMMAND));
        Self {
            controllers: [controller; 2],
            previous: None,
            settled: Duration::ZERO,
        }
    }

    /// Signed wheel revolutions relative to the beginning of this leg.
    pub fn update(
        &mut self,
        wheels: (f64, f64),
        diameter: f64,
        target: f64,
        dt: Duration,
    ) -> Result<MotionOutput, ()> {
        if ![wheels.0, wheels.1, diameter, target]
            .iter()
            .all(|v| v.is_finite())
            || diameter <= 0.0
            || dt.is_zero()
            || dt > MAX_CONTROL_GAP
        {
            return Err(());
        }
        let circumference = core::f64::consts::PI * diameter;
        let positions = (wheels.0 * circumference, wheels.1 * circumference);
        if !positions.0.is_finite() || !positions.1.is_finite() {
            return Err(());
        }
        let previous = self.previous.replace(positions).unwrap_or(positions);
        let velocities = (
            (positions.0 - previous.0) / dt.as_secs_f64(),
            (positions.1 - previous.1) / dt.as_secs_f64(),
        );
        let errors = (target - positions.0, target - positions.1);
        if ![errors.0, errors.1, velocities.0, velocities.1]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(());
        }
        let in_window = errors.0.abs() <= DRIVE_POSITION_TOLERANCE_METERS
            && errors.1.abs() <= DRIVE_POSITION_TOLERANCE_METERS
            && velocities.0.abs() <= DRIVE_VELOCITY_TOLERANCE
            && velocities.1.abs() <= DRIVE_VELOCITY_TOLERANCE;
        self.settled = if in_window {
            self.settled.saturating_add(dt)
        } else {
            Duration::ZERO
        };
        let mut command = |index: usize, position: f64, error: f64| {
            let output = self.controllers[index].update(position, target, dt);
            if error.abs() <= DRIVE_POSITION_TOLERANCE_METERS {
                0.0
            } else {
                // Static friction compensation, still bounded by the configured limit.
                // Follow position error when braking derivative reverses sign near the target.
                error.signum()
                    * (output * error.signum())
                        .clamp(AUTON_DRIVE_MIN_COMMAND, AUTON_DRIVE_MAX_COMMAND)
            }
        };
        Ok(MotionOutput {
            drive: (
                command(0, positions.0, errors.0),
                command(1, positions.1, errors.1),
            ),
            settled: self.settled >= AUTON_SETTLE_TIME,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn both_sides_must_settle_and_overshoot_is_corrected() {
        for target in [AUTON_FORWARD_METERS, -AUTON_RETREAT_METERS] {
            let mut motion = StraightMotion::new();
            let revs = target / (core::f64::consts::PI * DRIVE_WHEEL_DIAMETER_METERS);
            let first = motion
                .update(
                    (0.0, 0.0),
                    DRIVE_WHEEL_DIAMETER_METERS,
                    target,
                    CONTROL_INTERVAL,
                )
                .unwrap();
            assert!(first.drive.0 * target > 0.0);
            assert!(
                !motion
                    .update(
                        (revs, 0.0),
                        DRIVE_WHEEL_DIAMETER_METERS,
                        target,
                        CONTROL_INTERVAL
                    )
                    .unwrap()
                    .settled
            );
            let overshoot = motion
                .update(
                    (revs * 1.2, revs * 1.2),
                    DRIVE_WHEEL_DIAMETER_METERS,
                    target,
                    CONTROL_INTERVAL,
                )
                .unwrap();
            assert!(overshoot.drive.0 * target < 0.0);
            let mut settled = false;
            for _ in 0..20 {
                settled = motion
                    .update(
                        (revs, revs),
                        DRIVE_WHEEL_DIAMETER_METERS,
                        target,
                        CONTROL_INTERVAL,
                    )
                    .unwrap()
                    .settled;
            }
            assert!(settled);
        }
    }
    #[test]
    fn invalid_feedback_and_stale_control_stop_motion() {
        let mut motion = StraightMotion::new();
        for (wheels, diameter, dt) in [
            ((f64::NAN, 0.0), 0.05, CONTROL_INTERVAL),
            ((0.0, 0.0), 0.0, CONTROL_INTERVAL),
            ((0.0, 0.0), 0.05, Duration::ZERO),
            ((0.0, 0.0), 0.05, Duration::from_secs(1)),
        ] {
            assert!(motion.update(wheels, diameter, 0.3, dt).is_err());
        }
    }
}
