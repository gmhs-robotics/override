//! Hardware-independent cascade and intake coordination. Lift angles use its motor
//! encoder; pivot angles use P8 shaft rotation, except named intake output angles.
//! Lift control uses positive upward travel, converted at the device boundary.

pub use crate::config::{
    LIFT_DOWN_APPROACH_COMMAND, LIFT_DOWN_MAXIMUM_COMMAND, LIFT_DOWN_SLOWDOWN_DEGREES,
    LIFT_ENDPOINT_TOLERANCE_DEGREES, LIFT_MAXIMUM_DEGREES, LIFT_RAISE_DIRECTION,
    LIFT_TOP_CLEARANCE_DEGREES, LIFT_UP_APPROACH_COMMAND, LIFT_UP_MAXIMUM_COMMAND,
    LIFT_UP_SLOWDOWN_DEGREES, MATCH_LOAD_LIFT_TOLERANCE_DEGREES,
    MATCH_LOAD_PIVOT_TOLERANCE_OUTPUT_DEGREES,
};

/// Brief inward roller assistance when a new raise begins near the bottom.
/// Releasing, lowering, or ejecting cancels it; holding Raise never retriggers it.
#[derive(Default)]
pub struct DepartureRollerPulse {
    was_raising: bool,
    remaining: std::time::Duration,
}

impl DepartureRollerPulse {
    pub fn cancel(&mut self) {
        self.remaining = std::time::Duration::ZERO;
    }

    pub fn update(
        &mut self,
        raising: bool,
        near_bottom: bool,
        ejecting: bool,
        elapsed: std::time::Duration,
    ) -> bool {
        let starting = raising && !self.was_raising && near_bottom && !ejecting;
        self.was_raising = raising;
        if !raising || ejecting {
            self.cancel();
        } else if starting {
            self.remaining = crate::config::DEPARTURE_PULSE_TIME;
        } else {
            self.remaining = self.remaining.saturating_sub(elapsed);
        }
        !self.remaining.is_zero()
    }
}

/// The 11 W roller follows intake direction near the bottom and stowed pose.
/// Roller wiring requires opposite logical voltage from the main intake.
pub fn match_load_roller_direction(
    calibration: Calibration,
    lift_degrees: Option<f64>,
    pivot_degrees: Option<f64>,
    intake_request: i8,
) -> i8 {
    let (Some(lift), Some(pivot)) = (lift_degrees, pivot_degrees) else {
        return 0;
    };
    if intake_request != 0
        && calibration.valid()
        && lift.is_finite()
        && pivot.is_finite()
        && (lift - calibration.lift_bottom_degrees).abs() <= MATCH_LOAD_LIFT_TOLERANCE_DEGREES
        && (pivot - calibration.pivot_home_degrees).abs()
            <= MATCH_LOAD_PIVOT_TOLERANCE_OUTPUT_DEGREES * calibration.pivot_sensor_per_output_rev
    {
        -intake_request.signum()
    } else {
        0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub lift_bottom_degrees: f64,
    /// Mechanical hard stop. Upward commands stop below this by the clearance above.
    pub lift_top_degrees: f64,
    /// Lift encoder position at which the intake must have completed its configured turn.
    pub lift_rotated_degrees: f64,
    pub pivot_home_degrees: f64,
    /// Intake rotation at the driven gear, measured from its stowed orientation.
    pub pivot_travel_output_degrees: f64,
    /// Positive P8 sensor revolutions per output revolution, including external gearing.
    pub pivot_sensor_per_output_rev: f64,
}

impl Calibration {
    pub fn valid(self) -> bool {
        [
            self.lift_bottom_degrees,
            self.lift_top_degrees,
            self.lift_rotated_degrees,
            self.pivot_home_degrees,
            self.pivot_travel_output_degrees,
            self.pivot_sensor_per_output_rev,
            self.pivot_home_degrees
                + self.pivot_travel_output_degrees * self.pivot_sensor_per_output_rev,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.lift_bottom_degrees < self.lift_rotated_degrees
            && self.lift_rotated_degrees <= self.lift_upper_limit()
            && self.pivot_travel_output_degrees > 0.0
            && self.pivot_travel_output_degrees <= 90.0
            && self.pivot_sensor_per_output_rev > 0.0
    }

    pub fn lift_upper_limit(self) -> f64 {
        (self.lift_top_degrees - LIFT_TOP_CLEARANCE_DEGREES).min(LIFT_MAXIMUM_DEGREES)
    }

    /// P8 endpoint, including any extra travel beyond normal placement.
    pub fn pivot_limit(self, extra_output_degrees: f64) -> f64 {
        self.pivot_home_degrees
            + (self.pivot_travel_output_degrees + extra_output_degrees)
                * self.pivot_sensor_per_output_rev
    }

    pub fn pivot_target(self, lift_degrees: f64) -> f64 {
        let progress = ((lift_degrees - self.lift_bottom_degrees)
            / (self.lift_rotated_degrees - self.lift_bottom_degrees))
            .clamp(0.0, 1.0);
        self.pivot_home_degrees
            + self.pivot_travel_output_degrees * progress * self.pivot_sensor_per_output_rev
    }
}

/// Apply the same endpoint clearance and position-based power caps to live and
/// playback commands. Reducing power is immediate, independent of the ramp state.
pub fn limit_lift_command(calibration: Calibration, position: f64, command: f64) -> f64 {
    if !calibration.valid() || !position.is_finite() || !command.is_finite() {
        return 0.0;
    }
    if command > 0.0 {
        let remaining = calibration.lift_upper_limit() - position;
        if remaining <= LIFT_ENDPOINT_TOLERANCE_DEGREES {
            return 0.0;
        }
        let progress = (remaining / LIFT_UP_SLOWDOWN_DEGREES).clamp(0.0, 1.0);
        command.min(
            LIFT_UP_APPROACH_COMMAND
                + (LIFT_UP_MAXIMUM_COMMAND - LIFT_UP_APPROACH_COMMAND) * progress,
        )
    } else if command < 0.0 {
        let remaining = position - calibration.lift_bottom_degrees;
        if remaining <= LIFT_ENDPOINT_TOLERANCE_DEGREES {
            return 0.0;
        }
        let progress = (remaining / LIFT_DOWN_SLOWDOWN_DEGREES).clamp(0.0, 1.0);
        command.max(
            -(LIFT_DOWN_APPROACH_COMMAND
                + (LIFT_DOWN_MAXIMUM_COMMAND - LIFT_DOWN_APPROACH_COMMAND) * progress),
        )
    } else {
        0.0
    }
}

#[derive(Debug, PartialEq)]
pub struct Plan {
    pub lift_direction: i8,
    pub pivot_target_degrees: f64,
}

/// Fails closed on invalid calibration or encoder values. Actual encoder positions,
/// rather than button duration, determine pivot alignment.
pub fn plan(
    calibration: Calibration,
    lift_degrees: f64,
    pivot_degrees: f64,
    lift_request: i8,
) -> Option<Plan> {
    if !calibration.valid() || !lift_degrees.is_finite() || !pivot_degrees.is_finite() {
        return None;
    }
    let target = calibration.pivot_target(lift_degrees);
    let mut lift_direction = lift_request.signum();
    if (lift_direction > 0
        && lift_degrees >= calibration.lift_upper_limit() - LIFT_ENDPOINT_TOLERANCE_DEGREES)
        || (lift_direction < 0
            && lift_degrees <= calibration.lift_bottom_degrees + LIFT_ENDPOINT_TOLERANCE_DEGREES)
    {
        lift_direction = 0;
    }
    Some(Plan {
        lift_direction,
        pivot_target_degrees: target,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration() -> Calibration {
        // Test-only measurements, not robot defaults.
        Calibration {
            lift_bottom_degrees: 20.0,
            lift_top_degrees: 620.0,
            lift_rotated_degrees: 420.0,
            pivot_home_degrees: 10.0,
            pivot_travel_output_degrees: 90.0,
            pivot_sensor_per_output_rev: 2.0,
        }
    }

    #[test]
    fn departure_pulse_is_brief_and_never_retriggers_while_raise_is_held() {
        use std::time::Duration;
        let tick = Duration::from_millis(10);
        let mut pulse = DepartureRollerPulse::default();
        assert!(pulse.update(true, true, false, tick));
        // Once started, keep feeding while the lift leaves the loading region.
        for _ in 0..59 {
            assert!(pulse.update(true, false, false, tick));
        }
        assert!(!pulse.update(true, false, false, tick));
        for _ in 0..100 {
            assert!(!pulse.update(true, true, false, tick));
        }
        assert!(!pulse.update(false, true, false, tick));
        assert!(pulse.update(true, true, false, tick));
        assert!(!pulse.update(true, true, true, tick));
        assert!(!pulse.update(true, true, false, tick));
        pulse.update(false, false, false, tick);
        assert!(!pulse.update(true, false, false, tick));
        pulse.update(false, true, false, tick);
        assert!(pulse.update(true, true, false, tick));
        assert!(!pulse.update(true, true, false, Duration::from_secs(1)));
    }

    #[test]
    fn roller_follows_intake_in_both_directions_with_valid_bottom_and_stowed_feedback() {
        let c = cascade_calibration();
        assert_eq!(match_load_roller_direction(c, Some(0.0), Some(0.0), 1), -1);
        assert_eq!(
            match_load_roller_direction(c, Some(50.0), Some(40.0), 1),
            -1
        );
        for direction in [-1i8, 0, 1] {
            assert_eq!(
                match_load_roller_direction(c, Some(0.0), Some(0.0), direction),
                -direction.signum()
            );
        }
        for (lift, pivot) in [
            (Some(50.1), Some(0.0)),
            (Some(0.0), Some(40.1)),
            (Some(500.0), Some(360.0)),
            (None, Some(0.0)),
            (Some(0.0), None),
            (Some(f64::NAN), Some(0.0)),
            (Some(0.0), Some(f64::INFINITY)),
        ] {
            for direction in [-1, 1] {
                assert_eq!(match_load_roller_direction(c, lift, pivot, direction), 0);
            }
        }
    }

    #[test]
    fn pivot_tracks_measured_lift_in_both_directions_and_clamps_to_90_output_degrees() {
        let c = calibration();
        for (lift, pivot) in [(20.0, 10.0), (220.0, 100.0), (420.0, 190.0), (620.0, 190.0)] {
            for direction in [-1, 0, 1] {
                assert_eq!(
                    plan(c, lift, pivot, direction)
                        .unwrap()
                        .pivot_target_degrees,
                    pivot
                );
            }
        }
        assert_eq!(c.pivot_target(-100.0), 10.0);
        assert_eq!(c.pivot_target(1000.0), 190.0);
    }

    #[test]
    fn endpoints_block_outward_travel_but_allow_return() {
        let c = calibration();
        assert_eq!(plan(c, 620.0, 190.0, 1).unwrap().lift_direction, 0);
        assert_eq!(plan(c, 620.0, 190.0, -1).unwrap().lift_direction, -1);
        assert_eq!(plan(c, 20.0, 10.0, -1).unwrap().lift_direction, 0);
        assert_eq!(plan(c, 20.0, 10.0, 1).unwrap().lift_direction, 1);
    }

    #[test]
    fn pivot_lag_does_not_throttle_or_pause_lift() {
        let c = calibration();
        for direction in [-1, 1] {
            for pivot in [10.0, 80.0, 100.0, 120.0, 190.0] {
                let output = plan(c, 220.0, pivot, direction).unwrap();
                assert_eq!(output.lift_direction, direction);
                assert_eq!(output.pivot_target_degrees, 100.0);
            }
        }
    }

    #[test]
    fn invalid_calibration_and_feedback_never_produce_commands() {
        let c = calibration();
        for invalid in [
            Calibration {
                lift_top_degrees: 20.0,
                ..c
            },
            Calibration {
                lift_rotated_degrees: 20.0,
                ..c
            },
            Calibration {
                lift_rotated_degrees: 700.0,
                ..c
            },
            Calibration {
                pivot_travel_output_degrees: 0.0,
                ..c
            },
            Calibration {
                pivot_travel_output_degrees: -145.0,
                ..c
            },
            Calibration {
                pivot_travel_output_degrees: f64::INFINITY,
                ..c
            },
            Calibration {
                pivot_sensor_per_output_rev: 0.0,
                ..c
            },
            Calibration {
                pivot_sensor_per_output_rev: -1.0,
                ..c
            },
            Calibration {
                pivot_home_degrees: f64::NAN,
                ..c
            },
        ] {
            assert!(plan(invalid, 20.0, 10.0, 1).is_none());
        }
        assert!(plan(c, f64::NAN, 10.0, 1).is_none());
        assert!(plan(c, 20.0, f64::INFINITY, 1).is_none());
    }

    fn cascade_calibration() -> Calibration {
        Calibration {
            lift_bottom_degrees: 0.0,
            lift_top_degrees: 1800.0,
            lift_rotated_degrees: 135.0 * 360.0 / 145.0,
            pivot_home_degrees: 0.0,
            pivot_travel_output_degrees: 90.0,
            pivot_sensor_per_output_rev: 48.0 / 12.0,
        }
    }

    #[test]
    fn lift_135_degrees_maps_to_145_raw_pivot_degrees_then_continues_to_90_output_degrees() {
        let c = cascade_calibration();
        assert_eq!(c.pivot_target(c.lift_bottom_degrees), 0.0);
        assert!((c.pivot_target(67.5) - 72.5).abs() < 1e-10);
        assert!((c.pivot_target(135.0) - 145.0).abs() < 1e-10);
        assert!((c.pivot_target(270.0) - 290.0).abs() < 1e-10);
        assert!((c.lift_rotated_degrees - 335.172413793).abs() < 1e-9);
        assert_eq!(c.pivot_target(c.lift_rotated_degrees), 360.0);
        for direction in [-1, 0, 1] {
            assert!(
                (plan(c, 135.0, 145.0, direction)
                    .unwrap()
                    .pivot_target_degrees
                    - 145.0)
                    .abs()
                    < 1e-10
            );
        }
        assert_eq!(c.pivot_target(c.lift_top_degrees), 360.0);
        assert_eq!(c.pivot_target(-100.0), 0.0);
    }

    #[test]
    fn pivot_maintains_calibrated_rate_and_stays_within_one_motor_turn() {
        let c = cascade_calibration();
        let mut previous = 0.0;
        for lift in 0..=1800 {
            let target = c.pivot_target(f64::from(lift));
            assert!(target >= previous && target <= 360.0);
            previous = target;
        }
        for lift in 0..335 {
            let step = c.pivot_target(f64::from(lift + 1)) - c.pivot_target(f64::from(lift));
            assert!((step - 145.0 / 135.0).abs() < 1e-10);
        }
    }

    #[test]
    fn continuous_travel_does_not_stop_to_wait_for_a_slower_pivot() {
        let c = cascade_calibration();
        // A rate-limited encoder model exercises coordination over many 10 ms ticks.
        // The pivot is deliberately slower than the unscaled lift target would be.
        for direction in [-1, 1] {
            let mut lift = if direction > 0 { 0.0 } else { 500.0 };
            let mut pivot = c.pivot_target(lift);
            let mut reached_end = false;
            for _ in 0..1000 {
                let output = plan(c, lift, pivot, direction).unwrap();
                assert_eq!(output.lift_direction, direction);
                lift += f64::from(direction) * 540.0 * 0.01;
                pivot += (output.pivot_target_degrees - pivot).clamp(-3.6, 3.6);
                if (direction > 0 && lift >= 500.0) || (direction < 0 && lift <= 0.0) {
                    reached_end = true;
                    break;
                }
            }
            assert!(reached_end);
        }
    }

    #[test]
    fn upward_power_tapers_before_software_stop_without_increasing_small_commands() {
        let c = cascade_calibration();
        assert_eq!(limit_lift_command(c, 500.0, 1.0), 1.0);
        assert_eq!(limit_lift_command(c, 900.0, 1.0), 1.0);
        assert!((limit_lift_command(c, 925.0, 1.0) - 0.56).abs() < 1e-12);
        assert!((limit_lift_command(c, 949.0, 1.0) - 0.1376).abs() < 1e-12);
        assert_eq!(limit_lift_command(c, 949.0, 0.05), 0.05);
        let mut previous = 1.0;
        for position in 900..=1800 {
            let command = limit_lift_command(c, f64::from(position), 1.0);
            assert!(command >= 0.0 && command <= previous);
            previous = command;
        }
    }

    #[test]
    fn software_stop_blocks_ascent_before_hard_stop_but_allows_descent() {
        let c = cascade_calibration();
        assert_eq!(c.lift_upper_limit(), 950.0);
        for position in [950.0, 951.0, 1770.0, 1800.0, 1850.0] {
            assert_eq!(limit_lift_command(c, position, 0.75), 0.0);
            assert_eq!(plan(c, position, 360.0, 1).unwrap().lift_direction, 0);
            assert_eq!(limit_lift_command(c, position, -1.0), -0.35);
            assert_eq!(plan(c, position, 360.0, -1).unwrap().lift_direction, -1);
        }
    }

    #[test]
    fn descent_stays_low_power_and_tapers_to_bottom_hold() {
        let c = cascade_calibration();
        assert_eq!(limit_lift_command(c, 1000.0, -1.0), -0.35);
        assert_eq!(limit_lift_command(c, 180.0, -1.0), -0.35);
        assert!((limit_lift_command(c, 90.0, -1.0) + 0.215).abs() < 1e-12);
        assert!(limit_lift_command(c, 1.0, -1.0).abs() < 0.082);
        assert_eq!(limit_lift_command(c, 0.0, -1.0), 0.0);
        assert_eq!(limit_lift_command(c, -1.0, -1.0), 0.0);
        assert_eq!(limit_lift_command(c, 0.0, 0.0), 0.0);
        assert_eq!(limit_lift_command(c, 0.0, 0.75), 0.75);
    }

    #[test]
    fn invalid_limits_or_feedback_cannot_bypass_power_caps() {
        let c = cascade_calibration();
        assert_eq!(limit_lift_command(c, f64::NAN, 1.0), 0.0);
        assert_eq!(limit_lift_command(c, 1000.0, f64::INFINITY), 0.0);
        assert_eq!(
            limit_lift_command(
                Calibration {
                    lift_top_degrees: f64::NAN,
                    ..c
                },
                0.0,
                1.0
            ),
            0.0
        );
        assert!(
            !Calibration {
                lift_rotated_degrees: 1800.0,
                ..c
            }
            .valid()
        );
    }
}
