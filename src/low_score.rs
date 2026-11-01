//! B-only low scoring: lower, tilt upward, eject, then restore the captured pose.

pub use crate::config::{
    EJECT_TIME, LOW_HEIGHT_ABOVE_BOTTOM_DEGREES, POSITION_TOLERANCE, TRAVEL_TIMEOUT,
};

use crate::mechanism_control::Calibration;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Lower,
    Eject,
    Return,
}

struct Cycle {
    phase: Phase,
    elapsed: Duration,
    original_height: f64,
    low_height: f64,
    pivot_angle: f64,
    original_pivot: f64,
}

pub struct LowScoreCommand {
    pub lift_target: f64,
    pub pivot_target: f64,
    pub intake_direction: i8,
}

#[derive(Default)]
pub struct LowScore {
    pressed: bool,
    cycle: Option<Cycle>,
}

impl LowScore {
    pub fn cancel(&mut self) {
        self.cycle = None;
    }

    /// Heights are positive upward; P8 angles are positive toward pickup.
    /// An error cancels timed-out travel, allowing the caller to stop all outputs.
    pub fn update(
        &mut self,
        button: bool,
        manual: bool,
        height: f64,
        pivot: f64,
        calibration: Calibration,
        elapsed: Duration,
    ) -> Result<Option<LowScoreCommand>, ()> {
        let bottom = calibration.lift_bottom_degrees;
        let top = calibration.lift_upper_limit();
        if !calibration.valid()
            || ![height, pivot, bottom, top].iter().all(|v| v.is_finite())
            || bottom >= top
        {
            self.cancel();
            return Err(());
        }
        let low_height = (bottom + LOW_HEIGHT_ABOVE_BOTTOM_DEGREES).min(top);
        let edge = button && !self.pressed;
        self.pressed = button;
        if manual {
            self.cancel();
            return Ok(None);
        }
        if edge {
            if self.cycle.is_some() {
                self.cancel();
                return Ok(None);
            }
            crate::diagnostics::event!(
                "b_start,lift_up_deg={:.3},low_target_deg={:.3},p8_deg={:.3},extra_output_deg={:.3}",
                height,
                low_height.min(height.clamp(bottom, top)),
                pivot,
                crate::config::B_PIVOT_EXTRA_OUTPUT_DEGREES
            );
            self.cycle = Some(Cycle {
                phase: Phase::Lower,
                elapsed: Duration::ZERO,
                original_height: height.clamp(bottom, top),
                low_height: low_height.min(height.clamp(bottom, top)),
                pivot_angle: (pivot
                    + crate::config::B_PIVOT_EXTRA_OUTPUT_DEGREES
                        * calibration.pivot_sensor_per_output_rev)
                    .clamp(
                        calibration.pivot_home_degrees,
                        calibration.pivot_limit(crate::config::B_PIVOT_EXTRA_OUTPUT_DEGREES),
                    ),
                original_pivot: pivot
                    .clamp(calibration.pivot_home_degrees, calibration.pivot_limit(0.0)),
            });
        }
        let Some(cycle) = self.cycle.as_mut() else {
            return Ok(None);
        };
        let low_height = cycle.low_height;
        cycle.elapsed = cycle.elapsed.saturating_add(elapsed);
        let previous_phase = cycle.phase;
        match cycle.phase {
            Phase::Lower if (height - low_height).abs() <= POSITION_TOLERANCE => {
                cycle.phase = Phase::Eject;
                cycle.elapsed = Duration::ZERO;
            }
            Phase::Eject if cycle.elapsed >= EJECT_TIME => {
                cycle.phase = Phase::Return;
                cycle.elapsed = Duration::ZERO;
            }
            Phase::Return if (height - cycle.original_height).abs() <= POSITION_TOLERANCE => {
                let command = LowScoreCommand {
                    lift_target: cycle.original_height,
                    pivot_target: cycle.original_pivot,
                    intake_direction: 0,
                };
                self.cancel();
                return Ok(Some(command));
            }
            _ => {}
        }
        if previous_phase != cycle.phase {
            crate::diagnostics::event!(
                "b_phase_from={:?},b_phase_to={:?},lift_up_deg={:.3},low_target_deg={:.3},original_lift_deg={:.3},p8_deg={:.3},score_p8_target_deg={:.3},return_p8_target_deg={:.3}",
                previous_phase,
                cycle.phase,
                height,
                low_height,
                cycle.original_height,
                pivot,
                cycle.pivot_angle,
                cycle.original_pivot
            );
        }
        if !matches!(cycle.phase, Phase::Eject) && cycle.elapsed >= TRAVEL_TIMEOUT {
            self.cancel();
            return Err(());
        }
        Ok(Some(LowScoreCommand {
            lift_target: if matches!(cycle.phase, Phase::Return) {
                cycle.original_height
            } else {
                low_height
            },
            pivot_target: if matches!(cycle.phase, Phase::Return) {
                cycle.original_pivot
            } else {
                cycle.pivot_angle
            },
            intake_direction: if matches!(cycle.phase, Phase::Eject) {
                -1
            } else {
                0
            },
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const TICK: Duration = Duration::from_millis(10);

    #[test]
    fn lowers_tilts_ejects_then_restores_pose_without_repeating_held_button() {
        let mut score = LowScore::default();
        let first = score
            .update(
                true,
                false,
                500.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap()
            .unwrap();
        assert_eq!(first.lift_target, 5.0);
        assert_eq!(first.pivot_target, 380.0);
        assert_eq!(first.intake_direction, 0);
        let eject = score
            .update(
                true,
                false,
                5.0,
                355.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap()
            .unwrap();
        assert_eq!(eject.intake_direction, -1);
        assert_eq!(eject.pivot_target, 380.0);
        let still_ejecting = score
            .update(
                true,
                false,
                5.0,
                350.0,
                crate::config::MECHANISM_CALIBRATION,
                Duration::from_millis(1490),
            )
            .unwrap()
            .unwrap();
        assert_eq!(still_ejecting.intake_direction, -1);
        assert_eq!(still_ejecting.lift_target, 5.0);
        let returning = score
            .update(
                true,
                false,
                5.0,
                350.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap()
            .unwrap();
        assert_eq!(returning.lift_target, 500.0);
        assert_eq!(returning.intake_direction, 0);
        score
            .update(
                true,
                false,
                500.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        assert!(
            score
                .update(
                    true,
                    false,
                    500.0,
                    360.0,
                    crate::config::MECHANISM_CALIBRATION,
                    TICK
                )
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn b_tilt_is_five_output_degrees_capped_at_95_and_restores_original_on_return() {
        let c = crate::config::MECHANISM_CALIBRATION;
        for pivot in [300.0, 360.0] {
            let mut score = LowScore::default();
            let first = score
                .update(true, false, 500.0, pivot, c, TICK)
                .unwrap()
                .unwrap();
            assert_eq!(first.lift_target, 5.0);
            assert_eq!(first.pivot_target, pivot + 20.0); // +5 at 48T, +20 at P8.
            let eject = score
                .update(false, false, 5.0, first.pivot_target, c, TICK)
                .unwrap()
                .unwrap();
            assert_eq!(eject.intake_direction, -1);
            let returning = score
                .update(false, false, 5.0, first.pivot_target, c, EJECT_TIME)
                .unwrap()
                .unwrap();
            assert_eq!(returning.lift_target, 500.0);
            assert_eq!(returning.pivot_target, pivot);
            assert_eq!(returning.intake_direction, 0);
        }
    }

    #[test]
    fn second_press_manual_input_and_timeout_cancel() {
        let mut score = LowScore::default();
        score
            .update(
                true,
                false,
                500.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        score
            .update(
                false,
                false,
                400.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        assert!(
            score
                .update(
                    true,
                    false,
                    400.0,
                    360.0,
                    crate::config::MECHANISM_CALIBRATION,
                    TICK
                )
                .unwrap()
                .is_none()
        );
        score
            .update(
                false,
                false,
                400.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        score
            .update(
                true,
                false,
                400.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        assert!(
            score
                .update(
                    false,
                    true,
                    400.0,
                    360.0,
                    crate::config::MECHANISM_CALIBRATION,
                    TICK
                )
                .unwrap()
                .is_none()
        );
        score
            .update(
                true,
                false,
                400.0,
                360.0,
                crate::config::MECHANISM_CALIBRATION,
                TICK,
            )
            .unwrap();
        assert!(
            score
                .update(
                    false,
                    false,
                    400.0,
                    360.0,
                    crate::config::MECHANISM_CALIBRATION,
                    TRAVEL_TIMEOUT
                )
                .is_err()
        );
    }
}
