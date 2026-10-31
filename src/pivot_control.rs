//! Continuous velocity following for the geared intake pivot.

use std::time::Duration;

use ozton::control::loops::{Feedback, Pid};

const MAXIMUM_RPM: f64 = 200.0;
const ACCELERATION_RPM_PER_SECOND: f64 = 900.0;
const VELOCITY_FILTER_SECONDS: f64 = 0.04;
const SETTLED_MOTOR_DEGREES: f64 = 2.0;
const RESTART_MOTOR_DEGREES: f64 = 6.0;
pub const PIVOT_TRIM_MAX_OUTPUT_DEGREES: f64 = 45.0;

/// Retained manual alignment correction, measured at the intake output.
#[derive(Default)]
pub struct PivotTrim {
    output_degrees: f64,
}

impl PivotTrim {
    pub fn output_degrees(&self) -> f64 {
        self.output_degrees
    }

    /// X raises and A lowers scoring trim at 45 output degrees/s.
    /// Release or both buttons keep the retained angle; extra tilt caps at 45 degrees.
    pub fn update(&mut self, up: bool, down: bool, dt: Duration) -> f64 {
        let seconds = dt.min(Duration::from_millis(100)).as_secs_f64();
        let rate = match (up, down) {
            (true, false) => 45.0,
            (false, true) => -45.0,
            _ => 0.0,
        };
        self.output_degrees =
            (self.output_degrees + rate * seconds).clamp(0.0, PIVOT_TRIM_MAX_OUTPUT_DEGREES);
        self.output_degrees
    }

    /// Replay the recorded correction directly, without accumulating it again.
    pub fn restore(&mut self, output_degrees: f64) -> bool {
        if !output_degrees.is_finite()
            || !(0.0..=PIVOT_TRIM_MAX_OUTPUT_DEGREES).contains(&output_degrees)
        {
            return false;
        }
        self.output_degrees = output_degrees;
        true
    }
}

pub struct PivotController {
    feedback: Pid,
    previous_target: Option<f64>,
    target_velocity_rpm: f64,
    holding: bool,
    commanded_rpm: f64,
    held_target: Option<f64>,
}

impl PivotController {
    pub fn new() -> Self {
        Self {
            // RPM per motor degree. The motor's onboard velocity loop supplies damping.
            feedback: Pid::new(1.25, 0.0, 0.0, None),
            previous_target: None,
            target_velocity_rpm: 0.0,
            holding: true,
            commanded_rpm: 0.0,
            held_target: None,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// None requests motor Hold; Some requests continuous motor RPM, never another short move.
    pub fn update(
        &mut self,
        position: f64,
        target: f64,
        bottom: f64,
        top: f64,
        lift_moving: bool,
        dt: Duration,
    ) -> Option<i32> {
        let seconds = dt.as_secs_f64();
        if !seconds.is_finite()
            || seconds <= 0.0
            || !position.is_finite()
            || !target.is_finite()
            || !bottom.is_finite()
            || !top.is_finite()
            || bottom >= top
        {
            self.reset();
            return None;
        }
        let requested_target = target.clamp(bottom, top);
        // Lift encoder noise must not become alternating velocity feedforward
        // after the driver releases the controls. Keep an anchored target until
        // the requested angle changes enough to represent a real adjustment.
        let target = match self.previous_target {
            Some(previous)
                if !lift_moving && (requested_target - previous).abs() <= RESTART_MOTOR_DEGREES =>
            {
                previous
            }
            _ => requested_target,
        };
        let previous_target = self.previous_target.replace(target).unwrap_or(target);
        let velocity =
            ((target - previous_target) / (6.0 * seconds)).clamp(-MAXIMUM_RPM, MAXIMUM_RPM);
        let alpha = seconds / (VELOCITY_FILTER_SECONDS + seconds);
        self.target_velocity_rpm += alpha * (velocity - self.target_velocity_rpm);
        let error = target - position;
        if self.holding {
            // Latch onboard Hold once settled. Do not release it to chase sensor
            // drift: passive braking followed by catch-up creates a drop/raise cycle.
            // Only a deliberate target change resumes external velocity control.
            if self
                .held_target
                .is_some_and(|held| (target - held).abs() <= RESTART_MOTOR_DEGREES)
            {
                self.commanded_rpm = 0.0;
                self.target_velocity_rpm = 0.0;
                return None;
            }
            if error.abs() <= RESTART_MOTOR_DEGREES
                && (!lift_moving || self.target_velocity_rpm.abs() < 6.0)
            {
                self.commanded_rpm = 0.0;
                self.held_target.get_or_insert(target);
                return None;
            }
            self.holding = false;
        } else if error.abs() <= SETTLED_MOTOR_DEGREES
            && (!lift_moving || self.target_velocity_rpm.abs() < 3.0)
        {
            self.holding = true;
            self.held_target = Some(target);
            self.commanded_rpm = 0.0;
            return None;
        }

        let mut rpm = (self.target_velocity_rpm + self.feedback.update(position, target, dt))
            .clamp(-MAXIMUM_RPM, MAXIMUM_RPM);
        // Ramp increases in speed, including after reversals. Slowdowns remain
        // immediate so smoothing cannot delay endpoint protection.
        let previous = if rpm * self.commanded_rpm < 0.0 {
            0.0
        } else {
            self.commanded_rpm.abs()
        };
        rpm = rpm.signum()
            * rpm
                .abs()
                .min(previous + ACCELERATION_RPM_PER_SECOND * seconds.min(0.1));
        // Remove outward speed as the pivot reaches its calibrated range, even
        // while the velocity estimate is still decaying after the target stops.
        rpm = rpm.clamp(
            -((position - bottom).max(0.0) * 2.0),
            (top - position).max(0.0) * 2.0,
        );
        self.commanded_rpm = rpm;
        // Truncate toward zero so rounding cannot exceed the endpoint speed cap.
        Some(rpm as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: Duration = Duration::from_millis(10);

    #[test]
    fn trim_is_retained_after_release_and_down_reduces_it() {
        let mut trim = PivotTrim::default();
        for _ in 0..100 {
            trim.update(true, false, TICK);
        }
        assert!((trim.output_degrees() - 45.0).abs() < 1e-9);
        for _ in 0..100 {
            trim.update(true, false, TICK);
        }
        assert_eq!(trim.output_degrees(), 45.0);
        for _ in 0..500 {
            assert_eq!(trim.update(false, false, TICK), 45.0);
            assert_eq!(trim.update(true, true, TICK), 45.0);
        }
        assert!((trim.update(false, true, TICK) - 44.55).abs() < 1e-9);
        for _ in 0..299 {
            trim.update(false, true, TICK);
        }
        assert!(trim.output_degrees() < 1e-9);
        assert_eq!(trim.update(false, true, TICK), 0.0);
    }

    #[test]
    fn trim_can_resume_during_return_without_jumping_and_caps_long_ticks() {
        let mut trim = PivotTrim::default();
        assert_eq!(trim.update(true, false, Duration::from_secs(10)), 4.5);
        assert!((trim.update(false, true, TICK) - 4.05).abs() < 1e-9);
        assert!((trim.update(true, false, TICK) - 4.5).abs() < 1e-9);
    }

    #[test]
    fn playback_correction_is_absolute_and_invalid_values_are_rejected() {
        let mut trim = PivotTrim::default();
        for _ in 0..100 {
            assert!(trim.restore(22.5));
        }
        assert_eq!(trim.output_degrees(), 22.5);
        for invalid in [f64::NAN, f64::INFINITY, -1.0, 45.1] {
            assert!(!trim.restore(invalid));
            assert_eq!(trim.output_degrees(), 22.5);
        }
    }

    #[test]
    fn stationary_sensor_noise_does_not_restart_pivot() {
        let mut pivot = PivotController::new();
        for tick in 0..100 {
            let noise = if tick % 2 == 0 { 0.5 } else { -0.5 };
            assert_eq!(
                pivot.update(180.0 - noise, 180.0 + noise, 0.0, 360.0, false, TICK),
                None
            );
        }
    }

    #[test]
    fn held_lift_button_and_sensor_noise_do_not_restart_settled_pivot() {
        let mut controller = PivotController::new();
        assert_eq!(
            controller.update(180.0, 180.0, 0.0, 360.0, true, TICK),
            None
        );
        for tick in 0..500 {
            let noise = if tick % 2 == 0 { 3.0 } else { -3.0 };
            assert_eq!(
                controller.update(180.0 - noise, 180.0 + noise, 0.0, 360.0, true, TICK),
                None
            );
        }
        // A deliberate target adjustment still restarts movement.
        assert!(
            controller
                .update(180.0, 190.0, 0.0, 360.0, true, TICK)
                .is_some()
        );
    }

    #[test]
    fn stopped_lift_noise_cannot_create_velocity_feedforward_while_settling() {
        let mut controller = PivotController::new();
        let mut position = 0.0;
        let mut held = false;
        for tick in 0..400 {
            let noise = if tick % 2 == 0 { 3.0 } else { -3.0 };
            let command = controller.update(position, 180.0 + noise, 0.0, 360.0, false, TICK);
            if let Some(rpm) = command {
                assert!(!held, "stationary noise must not restart a settled pivot");
                assert!(rpm >= 0, "stationary noise must not reverse the pivot");
                position += f64::from(rpm) * 0.06;
            } else {
                held = true;
            }
        }
        assert!(held);
    }

    #[test]
    fn moving_target_has_continuous_velocity_in_both_directions() {
        for direction in [-1.0, 1.0] {
            let mut controller = PivotController::new();
            let mut position = if direction > 0.0 { 0.0 } else { 360.0 };
            let start = position;
            controller.update(position, start, 0.0, 360.0, true, TICK);
            for tick in 1..60 {
                let target = start + direction * f64::from(tick) * 5.0;
                let command = controller.update(position, target, 0.0, 360.0, true, TICK);
                if tick > 3 {
                    assert!(command.is_some_and(|rpm| f64::from(rpm) * direction > 0.0));
                }
                position += f64::from(command.unwrap_or(0)) * 6.0 * TICK.as_secs_f64();
            }
        }
    }

    #[test]
    fn catches_up_then_holds_without_repeated_reversals() {
        let mut controller = PivotController::new();
        let mut position = 0.0;
        let mut held = false;
        for _ in 0..300 {
            let command = controller.update(position, 180.0, 0.0, 360.0, false, TICK);
            if let Some(rpm) = command {
                assert!(rpm >= 0);
                position += f64::from(rpm) * 6.0 * TICK.as_secs_f64();
            } else {
                held = true;
            }
        }
        assert!(held);
        assert!((position - 180.0).abs() <= SETTLED_MOTOR_DEGREES);
    }

    #[test]
    fn latched_hold_does_not_restart_for_load_drift_only_for_new_targets() {
        let mut controller = PivotController::new();
        assert_eq!(
            controller.update(180.0, 180.0, 0.0, 360.0, false, TICK),
            None
        );
        for _ in 0..100 {
            assert_eq!(
                controller.update(172.0, 180.0, 0.0, 360.0, false, TICK),
                None
            );
        }
        for _ in 0..1000 {
            assert_eq!(
                controller.update(160.0, 180.0, 0.0, 360.0, false, TICK),
                None
            );
        }
        let mut controller = PivotController::new();
        controller.update(180.0, 180.0, 0.0, 360.0, false, TICK);
        assert!(
            controller
                .update(180.0, 200.0, 0.0, 360.0, false, TICK)
                .is_some()
        );
    }

    #[test]
    fn abrupt_target_and_reversal_ramp_speed_but_endpoints_stop_immediately() {
        let mut controller = PivotController::new();
        assert_eq!(
            controller.update(500.0, 1000.0, 0.0, 1000.0, true, TICK),
            Some(9)
        );
        assert_eq!(
            controller.update(500.0, 1000.0, 0.0, 1000.0, true, TICK),
            Some(18)
        );
        let reverse = controller
            .update(500.0, 0.0, 0.0, 1000.0, true, TICK)
            .unwrap();
        assert!((-9..=0).contains(&reverse));
        assert_eq!(
            controller
                .update(0.0, -100.0, 0.0, 1000.0, true, TICK)
                .unwrap_or(0),
            0
        );
        assert!(
            controller
                .update(1000.0, 1500.0, 0.0, 1000.0, true, TICK)
                .unwrap_or(0)
                <= 0
        );
    }

    #[test]
    fn endpoints_never_command_outward_velocity() {
        let mut controller = PivotController::new();
        controller.update(300.0, 300.0, 0.0, 360.0, true, TICK);
        assert!(
            controller
                .update(360.0, 360.0, 0.0, 360.0, true, TICK)
                .unwrap_or(0)
                <= 0
        );
        controller.reset();
        controller.update(60.0, 60.0, 0.0, 360.0, true, TICK);
        assert!(
            controller
                .update(0.0, 0.0, 0.0, 360.0, true, TICK)
                .unwrap_or(0)
                >= 0
        );
    }
}
