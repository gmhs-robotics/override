//! One P8 position loop drives pivot voltage during movement and holding.

pub use crate::config::{
    INTEGRAL_GAIN, MAXIMUM_VOLTS, PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES, POSITION_GAIN,
    VELOCITY_DAMPING, VELOCITY_FILTER_SECONDS,
};

use std::time::Duration;

pub struct PivotController {
    holding_voltage: f64,
    previous_position: Option<f64>,
    velocity: f64,
}

impl PivotController {
    pub fn new() -> Self {
        Self {
            holding_voltage: 0.0,
            previous_position: None,
            velocity: 0.0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// P8 shaft degrees in, volts out. Keep this loop active at the target so
    /// integral action supports load and any subsequent P8 error is corrected.
    /// Damping uses measured motion rather than target changes to avoid a kick
    /// when X/A selects another preset. None means invalid feedback: stop.
    pub fn update(
        &mut self,
        position: f64,
        target: f64,
        bottom: f64,
        top: f64,
        dt: Duration,
    ) -> Option<f64> {
        let seconds = dt.as_secs_f64();
        if seconds <= 0.0
            || dt > crate::config::MAX_CONTROL_GAP
            || !position.is_finite()
            || !target.is_finite()
            || !bottom.is_finite()
            || !top.is_finite()
            || bottom >= top
        {
            self.reset();
            return None;
        }
        let target = target.clamp(bottom, top);
        let previous = self.previous_position.replace(position).unwrap_or(position);
        let alpha = seconds / (VELOCITY_FILTER_SECONDS + seconds);
        self.velocity += alpha * ((position - previous) / seconds - self.velocity);
        let error = target - position;
        // If already beyond the allowed range, discard only torque that fights
        // the required return direction. Do not clear load support at the target.
        if (position < bottom && self.holding_voltage < 0.0)
            || (position > top && self.holding_voltage > 0.0)
        {
            self.holding_voltage = 0.0;
        }
        let proportional_and_damping = POSITION_GAIN * error - VELOCITY_DAMPING * self.velocity;
        // Holding torque must survive zero crossings and larger load deflection.
        let candidate = (self.holding_voltage + INTEGRAL_GAIN * error * seconds)
            .clamp(-MAXIMUM_VOLTS, MAXIMUM_VOLTS);
        let demand = proportional_and_damping + candidate;
        let outward = (position <= bottom && demand < 0.0) || (position >= top && demand > 0.0);
        // Conditional integration prevents windup while saturated or at a stop.
        if !outward && (demand.abs() <= MAXIMUM_VOLTS || error * demand < 0.0) {
            self.holding_voltage = candidate;
        }
        let mut voltage =
            (proportional_and_damping + self.holding_voltage).clamp(-MAXIMUM_VOLTS, MAXIMUM_VOLTS);
        if (position <= bottom && voltage < 0.0) || (position >= top && voltage > 0.0) {
            voltage = 0.0;
        }
        Some(voltage)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: Duration = Duration::from_millis(10);

    #[test]
    fn target_changes_and_load_deflection_always_use_current_p8_position() {
        let mut controller = PivotController::new();
        assert_eq!(controller.update(360.0, 360.0, 0.0, 540.0, TICK), Some(0.0));
        assert!(controller.update(340.0, 360.0, 0.0, 540.0, TICK).unwrap() > 0.0);
        controller.reset();
        assert!(controller.update(380.0, 360.0, 0.0, 540.0, TICK).unwrap() < 0.0);
        assert!(controller.update(380.0, 0.0, 0.0, 540.0, TICK).unwrap() < 0.0);
    }

    #[test]
    fn voltage_limits_and_endpoints_apply_in_both_directions() {
        for (position, target) in [(0.0, 540.0), (540.0, 0.0)] {
            let mut controller = PivotController::new();
            assert!(
                controller
                    .update(position, target, 0.0, 540.0, TICK)
                    .unwrap()
                    .abs()
                    <= 8.0
            );
        }
        let mut controller = PivotController::new();
        assert!(controller.update(-5.0, -100.0, 0.0, 540.0, TICK).unwrap() >= 0.0);
        controller.reset();
        assert!(controller.update(545.0, 700.0, 0.0, 540.0, TICK).unwrap() <= 0.0);
        assert_eq!(controller.update(f64::NAN, 0.0, 0.0, 540.0, TICK), None);
        assert_eq!(controller.update(0.0, 0.0, 1.0, 1.0, TICK), None);
        assert_eq!(
            controller.update(0.0, 0.0, 0.0, 540.0, Duration::ZERO),
            None
        );
    }

    #[test]
    fn small_p8_noise_produces_small_voltage_without_minimum_speed_kicks() {
        let mut controller = PivotController::new();
        for tick in 0..1000 {
            let position = if tick % 2 == 0 { 360.25 } else { 359.75 };
            let voltage = controller
                .update(position, 360.0, 0.0, 540.0, TICK)
                .unwrap();
            assert!(voltage.abs() < 0.15);
        }
    }

    #[test]
    fn sustained_load_above_old_integral_window_does_not_leave_an_offset() {
        let mut controller = PivotController::new();
        let mut position = 360.0;
        let mut velocity = 0.0;
        for _ in 0..3000 {
            let voltage = controller
                .update(position, 360.0, 0.0, 540.0, TICK)
                .unwrap();
            // Exercise sustained load support beyond a small position error.
            velocity += (voltage * 150.0 - 300.0 - velocity) * 0.15;
            position += velocity * 0.01;
        }
        assert!(
            (position - 360.0).abs() < 1.0,
            "loaded P8 position {position}"
        );
    }

    #[test]
    fn holding_torque_survives_noise_crossing_the_target_and_saturation_does_not_wind_up() {
        let mut controller = PivotController::new();
        // Simulate a sustained deflection while load torque builds.
        for _ in 0..1000 {
            controller.update(350.0, 360.0, 0.0, 540.0, TICK);
        }
        for tick in 0..1000 {
            let position = if tick % 2 == 0 { 360.25 } else { 359.75 };
            let voltage = controller
                .update(position, 360.0, 0.0, 540.0, TICK)
                .unwrap();
            if tick > 100 {
                assert!(voltage > 3.8 && voltage < 4.2, "holding volts {voltage}");
            }
        }
        controller.reset();
        for _ in 0..10000 {
            controller.update(0.0, 540.0, 0.0, 540.0, TICK);
        }
        assert_eq!(controller.update(0.0, 0.0, 0.0, 540.0, TICK), Some(0.0));
    }

    #[test]
    fn repeated_trips_recover_the_same_p8_targets_with_inertia_and_load() {
        let mut controller = PivotController::new();
        let mut position = 0.0;
        let mut velocity = 0.0;
        // Simple inertial plant with a constant downward load equivalent to 0.4 V.
        for _ in 0..4 {
            for target in [540.0, 360.0, 0.0] {
                let mut low = f64::INFINITY;
                let mut high = f64::NEG_INFINITY;
                for step in 0..1500 {
                    let voltage = controller
                        .update(position, target, 0.0, 540.0, TICK)
                        .unwrap();
                    velocity += (voltage * 150.0 - 60.0 - velocity) * 0.15;
                    position = (position + velocity * 0.01).clamp(0.0, 540.0);
                    if position == 0.0 && velocity < 0.0 {
                        velocity = 0.0;
                    }
                    if step >= 1200 {
                        low = low.min(position);
                        high = high.max(position);
                    }
                }
                assert!(high - low < 0.5, "holding excursion {}", high - low);
                assert!(
                    (position - target).abs() < 1.0,
                    "target {target}, position {position}"
                );
            }
        }
    }
}
