//! Robot commands shared by premade routes. Feedback, deadlines and cancellation
//! live here so a route reads as a short sequence of robot operations.
mod error;

pub(crate) use error::AutonError;

use crate::{
    Robot,
    config::{
        AUTON_LIFT_ARRIVAL_TOLERANCE, AUTON_PIVOT_ARRIVAL_OUTPUT_DEGREES, AUTON_SETTLE_TIME,
        AUTON_STAGE_TIMEOUT, AUTON_TOTAL_TIMEOUT, CONTROL_INTERVAL, DRIVE_WHEEL_DIAMETER_METERS,
        MAX_CONTROL_GAP, MECHANISM_CALIBRATION,
    },
    forward_auton::StraightMotion,
};
use std::time::{Duration, Instant};
use vexide::{
    competition::{self, CompetitionMode},
    time::sleep,
};

/// Result returned by premade autonomous commands; successful commands yield ().
pub(crate) type AutonResult<T = ()> = core::result::Result<T, AutonError>;

/// The concrete Robot implements these commands. Routes can also run against
/// test hardware; recording does not use this interface.
/// Lift heights are positive-up motor degrees, pivot angles are output gear
/// degrees, and drive distances are nonnegative meters. Setters retain targets;
/// awaited commands service all feedback loops until they finish or fail.
pub(crate) trait Commands {
    fn set_lift_height(&mut self, degrees: f64) -> AutonResult;
    fn follow_pivot(&mut self);
    fn set_pivot_angle(&mut self, output_degrees: f64) -> AutonResult;
    fn start_eject(&mut self);
    fn stop_eject(&mut self);
    async fn drive_forward(&mut self, meters: f64) -> AutonResult;
    async fn drive_backward(&mut self, meters: f64) -> AutonResult;
    async fn wait_for_mechanisms(&mut self) -> AutonResult;
    async fn wait(&mut self, duration: Duration) -> AutonResult;
}

pub(crate) struct AutonControl {
    lift: f64,
    pivot: Option<f64>,
    eject: bool,
    started: Instant,
    #[cfg(debug_assertions)]
    leg: Option<((f64, f64), f64)>,
}

impl Default for AutonControl {
    fn default() -> Self {
        Self {
            lift: MECHANISM_CALIBRATION.lift_bottom_degrees,
            pivot: None,
            eject: false,
            started: Instant::now(),
            #[cfg(debug_assertions)]
            leg: None,
        }
    }
}

impl AutonControl {
    fn begin(&mut self) {
        self.clear_targets();
        self.started = Instant::now();
    }

    pub(crate) fn clear_targets(&mut self) {
        self.lift = MECHANISM_CALIBRATION.lift_bottom_degrees;
        self.pivot = None;
        self.eject = false;
        #[cfg(debug_assertions)]
        self.finish_drive();
    }

    fn set_lift_height(&mut self, degrees: f64) -> AutonResult {
        if !degrees.is_finite() {
            return Err(AutonError::InvalidLiftTarget);
        }
        let c = MECHANISM_CALIBRATION;
        self.lift = degrees.clamp(c.lift_bottom_degrees, c.lift_upper_limit());
        Ok(())
    }

    fn follow_pivot(&mut self) {
        self.pivot = None;
    }

    fn set_pivot_angle(&mut self, output_degrees: f64) -> AutonResult {
        if !output_degrees.is_finite() {
            return Err(AutonError::InvalidPivotTarget);
        }
        let c = MECHANISM_CALIBRATION;
        self.pivot = Some(
            c.pivot_home_degrees
                + output_degrees.clamp(0.0, c.pivot_travel_output_degrees)
                    * c.pivot_sensor_per_output_rev,
        );
        Ok(())
    }

    fn start_eject(&mut self) {
        self.eject = true;
    }
    fn stop_eject(&mut self) {
        self.eject = false;
    }
    fn ejecting(&self) -> bool {
        self.eject
    }
    fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    fn update_mechanisms(&self, mechanisms: &mut crate::mechanisms::Mechanisms) -> AutonResult {
        if mechanisms.autonomous_target(self.lift, self.pivot, self.eject) {
            Ok(())
        } else {
            Err(AutonError::MechanismFault)
        }
    }

    fn pose_ready(&self, height: f64, pivot: f64) -> bool {
        pose_ready(height, pivot, self.lift, self.pivot)
    }

    #[cfg(debug_assertions)]
    fn start_drive(&mut self, start: (f64, f64), meters: f64) {
        self.leg = Some((start, meters));
    }
    #[cfg(debug_assertions)]
    fn finish_drive(&mut self) {
        self.leg = None;
    }

    #[cfg(debug_assertions)]
    pub(crate) fn debug_sample(&self, seq: u64, ms: u128, wheels: Option<(f64, f64)>) {
        let (start, target) = self.leg.unwrap_or(((f64::NAN, f64::NAN), f64::NAN));
        let wheels = wheels.unwrap_or((f64::NAN, f64::NAN));
        let circumference = core::f64::consts::PI * DRIVE_WHEEL_DIAMETER_METERS;
        let left = (wheels.0 - start.0) * circumference;
        let right = (wheels.1 - start.1) * circumference;
        println!(
            "DBG,AUTON,seq={seq},t_ms={ms},route_ms={},lift_target_deg={:.3},pivot_target_deg={:.3},eject={},leg_target_m={target:.5},leg_left_m={left:.5},leg_right_m={right:.5},left_error_m={:.5},right_error_m={:.5}",
            self.started.elapsed().as_millis(),
            self.lift,
            self.pivot.unwrap_or(f64::NAN),
            u8::from(self.eject),
            target - left,
            target - right
        );
    }
}

/// One clock implementation for all awaited commands. A tick measures real
/// elapsed time, including feedback and diagnostic work, rather than assuming
/// that sleep duration equals the controller interval.
struct CommandClock {
    started: Instant,
    previous: Instant,
    interval: Duration,
}

impl CommandClock {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            started: now,
            previous: now,
            interval: CONTROL_INTERVAL,
        }
    }
    fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }
    fn interval(&self) -> Duration {
        self.interval
    }
    fn validate(&self, enabled: bool, total: Duration) -> AutonResult {
        check_tick(enabled, total, self.elapsed(), self.interval())
    }
    async fn tick(&mut self) {
        sleep(CONTROL_INTERVAL).await;
        let now = Instant::now();
        self.interval = now.saturating_duration_since(self.previous);
        self.previous = now;
    }
}

fn check_tick(enabled: bool, total: Duration, step: Duration, dt: Duration) -> AutonResult {
    if !enabled {
        return Err(AutonError::Cancelled);
    }
    if total >= AUTON_TOTAL_TIMEOUT {
        return Err(AutonError::RouteTimeout);
    }
    if step >= AUTON_STAGE_TIMEOUT {
        return Err(AutonError::CommandTimeout);
    }
    if dt.is_zero() || dt > MAX_CONTROL_GAP {
        return Err(AutonError::ControlDelay);
    }
    Ok(())
}

fn pose_ready(height: f64, pivot: f64, target: f64, pivot_target: Option<f64>) -> bool {
    let c = MECHANISM_CALIBRATION;
    let pivot_target = pivot_target.unwrap_or_else(|| c.pivot_target(target));
    height.is_finite()
        && pivot.is_finite()
        && (height - target).abs() <= AUTON_LIFT_ARRIVAL_TOLERANCE
        && (pivot - pivot_target).abs()
            <= AUTON_PIVOT_ARRIVAL_OUTPUT_DEGREES * c.pivot_sensor_per_output_rev
}

impl Robot {
    pub(crate) fn begin_autonomous(&mut self) {
        self.auton.begin();
    }

    fn auton_status(&mut self, status: &'static str) {
        self.menu.status(status);
        crate::diagnostics::event!(
            "auton_command={},route_ms={}",
            status,
            self.auton.elapsed().as_millis()
        );
    }

    /// Every awaited command keeps the lift/P8 loops running and checks mode.
    fn auton_step(&mut self, drive: Option<(f64, f64)>, clock: &CommandClock) -> AutonResult {
        self.menu.poll(false);
        clock.validate(
            competition::mode() == CompetitionMode::Autonomous,
            self.auton.elapsed(),
        )?;
        self.auton.update_mechanisms(&mut self.mechanisms)?;
        let result = if let Some((left, right)) = drive {
            self.drivetrain.autonomous_drive(left, right)
        } else {
            self.drivetrain.autonomous_brake()
        };
        result.map_err(AutonError::DriveOutput)?;
        #[cfg(debug_assertions)]
        self.debug_tick(clock.interval(), true, false, false, None, None);
        Ok(())
    }

    async fn drive_distance(&mut self, meters: f64) -> AutonResult {
        if !meters.is_finite() {
            return Err(AutonError::InvalidDistance);
        }
        self.auton_status(if meters >= 0.0 {
            "Autonomous: drive forward"
        } else {
            "Autonomous: drive backward"
        });
        let start = self
            .drivetrain
            .wheel_positions()
            .ok_or(AutonError::EncoderUnavailable)?;
        #[cfg(debug_assertions)]
        self.auton.start_drive(start, meters);
        let mut motion = StraightMotion::new();
        let mut clock = CommandClock::new();
        loop {
            let wheels = self
                .drivetrain
                .wheel_positions()
                .ok_or(AutonError::EncoderUnavailable)?;
            let output = motion
                .update(
                    (wheels.0 - start.0, wheels.1 - start.1),
                    DRIVE_WHEEL_DIAMETER_METERS,
                    meters,
                    clock.interval(),
                )
                .map_err(|_| AutonError::InvalidWheelFeedback)?;
            self.auton_step(Some(output.drive), &clock)?;
            if output.settled {
                self.drivetrain
                    .autonomous_brake()
                    .map_err(AutonError::DriveBrake)?;
                #[cfg(debug_assertions)]
                self.auton.finish_drive();
                return Ok(());
            }
            clock.tick().await;
        }
    }
}

impl Commands for Robot {
    fn set_lift_height(&mut self, degrees: f64) -> AutonResult {
        self.auton.set_lift_height(degrees)
    }
    fn follow_pivot(&mut self) {
        self.auton.follow_pivot();
    }
    fn set_pivot_angle(&mut self, output_degrees: f64) -> AutonResult {
        self.auton.set_pivot_angle(output_degrees)
    }
    fn start_eject(&mut self) {
        self.auton.start_eject();
    }
    fn stop_eject(&mut self) {
        self.auton.stop_eject();
    }
    async fn drive_forward(&mut self, meters: f64) -> AutonResult {
        if meters < 0.0 {
            return Err(AutonError::NegativeForwardDistance);
        }
        self.drive_distance(meters).await
    }
    async fn drive_backward(&mut self, meters: f64) -> AutonResult {
        if meters < 0.0 {
            return Err(AutonError::NegativeBackwardDistance);
        }
        self.drive_distance(-meters).await
    }
    async fn wait_for_mechanisms(&mut self) -> AutonResult {
        self.auton_status("Autonomous: move lift and pivot");
        let mut clock = CommandClock::new();
        let mut settled = Duration::ZERO;
        loop {
            self.auton_step(None, &clock)?;
            let (height, pivot) = self
                .mechanisms
                .pose()
                .ok_or(AutonError::MechanismFeedbackUnavailable)?;
            settled = if self.auton.pose_ready(height, pivot) {
                settled.saturating_add(clock.interval())
            } else {
                Duration::ZERO
            };
            if settled >= AUTON_SETTLE_TIME {
                return Ok(());
            }
            clock.tick().await;
        }
    }
    async fn wait(&mut self, duration: Duration) -> AutonResult {
        self.auton_status(if self.auton.ejecting() {
            "Autonomous: eject"
        } else {
            "Autonomous: hold"
        });
        let mut clock = CommandClock::new();
        loop {
            self.auton_step(None, &clock)?;
            if clock.elapsed() >= duration {
                return Ok(());
            }
            clock.tick().await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::AUTON_RAISE_FRACTION;
    #[test]
    fn setters_validate_before_mutating_and_clamp_to_robot_limits() {
        let mut state = AutonControl::default();
        state.set_lift_height(200.0).unwrap();
        state.set_pivot_angle(45.0).unwrap();
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(state.set_lift_height(invalid).is_err());
            assert!(state.set_pivot_angle(invalid).is_err());
            assert_eq!(state.lift, 200.0);
            assert_eq!(state.pivot, Some(180.0));
        }
        state.set_lift_height(10_000.0).unwrap();
        assert_eq!(state.lift, MECHANISM_CALIBRATION.lift_upper_limit());
        state.set_lift_height(-10.0).unwrap();
        assert_eq!(state.lift, MECHANISM_CALIBRATION.lift_bottom_degrees);
        state.set_pivot_angle(120.0).unwrap();
        assert_eq!(state.pivot, Some(MECHANISM_CALIBRATION.pivot_limit(0.0)));
        state.follow_pivot();
        assert_eq!(state.pivot, None);
    }

    #[test]
    fn stop_and_new_run_clear_all_retained_commands() {
        let mut state = AutonControl::default();
        state.set_lift_height(500.0).unwrap();
        state.set_pivot_angle(90.0).unwrap();
        state.start_eject();
        assert!(state.ejecting());
        state.stop_eject();
        assert!(!state.ejecting());
        state.start_eject();
        state.clear_targets();
        assert_eq!(state.lift, MECHANISM_CALIBRATION.lift_bottom_degrees);
        assert_eq!(state.pivot, None);
        assert!(!state.ejecting());
        state.set_lift_height(100.0).unwrap();
        state.start_eject();
        state.begin();
        assert_eq!(state.lift, MECHANISM_CALIBRATION.lift_bottom_degrees);
        assert_eq!(state.pivot, None);
        assert!(!state.ejecting());
    }

    #[test]
    fn cancellation_deadlines_and_late_ticks_prevent_next_output() {
        assert!(check_tick(true, Duration::ZERO, Duration::ZERO, CONTROL_INTERVAL).is_ok());
        assert_eq!(
            check_tick(false, Duration::ZERO, Duration::ZERO, CONTROL_INTERVAL),
            Err(AutonError::Cancelled)
        );
        assert!(check_tick(true, AUTON_TOTAL_TIMEOUT, Duration::ZERO, CONTROL_INTERVAL).is_err());
        assert!(check_tick(true, Duration::ZERO, AUTON_STAGE_TIMEOUT, CONTROL_INTERVAL).is_err());
        assert!(check_tick(true, Duration::ZERO, Duration::ZERO, Duration::from_secs(1)).is_err());
    }
    #[test]
    fn loaded_offsets_are_accepted_but_missing_or_large_pose_errors_are_not() {
        let target = MECHANISM_CALIBRATION.lift_upper_limit() * AUTON_RAISE_FRACTION;
        let flat = MECHANISM_CALIBRATION.pivot_limit(0.0);
        assert!(pose_ready(target - 8.0, flat - 12.0, target, Some(flat)));
        assert!(!pose_ready(target - 100.0, flat, target, Some(flat)));
        assert!(!pose_ready(target, f64::NAN, target, Some(flat)));
    }
}
