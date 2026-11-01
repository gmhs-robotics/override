//! Robot wiring, gearing, limits, gains, timings and route tuning.

use crate::mechanism_control::Calibration;
use std::time::Duration;

// Competition robot configuration.
pub const CONTROL_INTERVAL: Duration = Duration::from_millis(10);
pub const ELEVEN_W_WHEEL_PER_MOTOR_REV: f64 = 36.0 / 48.0;
pub const FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV: f64 = 1.0;
pub const DRIVE_OUTPUT_SCALE: f64 = 0.70;
pub const TIP_THRESHOLD_RADIANS: f64 = 0.16;
pub const TIP_PID: (f64, f64, f64) = (1.35, 0.0, 0.03);
// P7 rotated 180 degrees about sensor X: +Y rearward, +Z upward.
// X/roll direction is unchanged; startup level subtraction handles the half-turn.
pub const TIP_POLARITY: f64 = -1.0;
pub const TIP_MAXIMUM_CORRECTION: f64 = 0.35;

// Pivot reaches 90 output degrees at about -167.59 lift encoder degrees.
// Start with the lift at zero and the intake stowed so startup references match the mechanism.
// Pivot feedback uses the rotation sensor on the motor shaft, before external gearing.
// P8 is on the 12T shaft; the intake is on the 48T output.
pub const PIVOT_SENSOR_PER_OUTPUT_REV: f64 = 48.0 / 12.0;
pub const PIVOT_SENSOR_DEGREES_PER_LIFT_DEGREE: f64 = 145.0 * 2.0 / 135.0;
pub const PIVOT_NORMAL_OUTPUT_DEGREES: f64 = 90.0;
pub const MECHANISM_CALIBRATION: Calibration = Calibration {
    lift_bottom_degrees: 0.0,
    lift_top_degrees: 1800.0,
    lift_rotated_degrees: PIVOT_NORMAL_OUTPUT_DEGREES * PIVOT_SENSOR_PER_OUTPUT_REV
        / PIVOT_SENSOR_DEGREES_PER_LIFT_DEGREE,
    pivot_home_degrees: 0.0,
    pivot_travel_output_degrees: PIVOT_NORMAL_OUTPUT_DEGREES,
    pivot_sensor_per_output_rev: PIVOT_SENSOR_PER_OUTPUT_REV, // 12T motor gear drives the 48T intake gear
};
// 2.25-inch omniwheels.
pub const DRIVE_WHEEL_DIAMETER_METERS: f64 = 2.25 * 0.0254;

pub const LIFT_TOP_CLEARANCE_DEGREES: f64 = 30.0;
pub const LIFT_MAXIMUM_DEGREES: f64 = 950.0;
pub const LIFT_ENDPOINT_TOLERANCE_DEGREES: f64 = 0.01;
pub const LIFT_RAISE_DIRECTION: f64 = -1.0;
pub const LIFT_UP_SLOWDOWN_DEGREES: f64 = 50.0;
pub const LIFT_DOWN_SLOWDOWN_DEGREES: f64 = 180.0;
pub const LIFT_UP_MAXIMUM_COMMAND: f64 = 1.0;
pub const LIFT_DOWN_MAXIMUM_COMMAND: f64 = 0.35;
pub const LIFT_UP_APPROACH_COMMAND: f64 = 0.12;
pub const LIFT_DOWN_APPROACH_COMMAND: f64 = 0.08;
pub const MATCH_LOAD_LIFT_TOLERANCE_DEGREES: f64 = 50.0;
pub const MATCH_LOAD_PIVOT_TOLERANCE_OUTPUT_DEGREES: f64 = 10.0;
pub const PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES: f64 = 30.0;
pub const MAXIMUM_VOLTS: f64 = 8.0;
pub const VELOCITY_FILTER_SECONDS: f64 = 0.04;
pub const VELOCITY_DAMPING: f64 = 0.002;
pub const POSITION_GAIN: f64 = 0.05;
pub const INTEGRAL_GAIN: f64 = 0.04;
pub const POSITION_TOLERANCE: f64 = 3.0;
pub const EJECT_TIME: Duration = Duration::from_millis(1500);
pub const LOW_HEIGHT_ABOVE_BOTTOM_DEGREES: f64 = 5.0;
pub const TRAVEL_TIMEOUT: Duration = Duration::from_secs(8);

// Initial selection; the operator can toggle this while competition is disabled.
pub const AUTON_ENABLED_DEFAULT: bool = false;

// Autonomous distances are nonnegative meters; commands supply direction.
// Lift targets are positive upward encoder degrees.
pub const AUTON_FORWARD_METERS: f64 = 12.0 * 0.0254;
pub const AUTON_RETREAT_METERS: f64 = 8.0 * 0.0254;
// Autonomous arrival checks are separate from the motor controller deadband.
// A few encoder degrees of loaded deflection must not deadlock the route.
pub const AUTON_LIFT_ARRIVAL_TOLERANCE: f64 = 10.0;
pub const AUTON_PIVOT_ARRIVAL_OUTPUT_DEGREES: f64 = 4.0;
pub const AUTON_RAISE_FRACTION: f64 = 0.25;
// Initial tuning for “lower a bit”; lift by this amount again during ejection.
pub const AUTON_LOWER_DEGREES: f64 = 35.0;
pub const AUTON_RELEASE_TIME: Duration = Duration::from_millis(300);
pub const AUTON_STAGE_TIMEOUT: Duration = Duration::from_secs(8);
pub const AUTON_TOTAL_TIMEOUT: Duration = Duration::from_secs(15);
pub const AUTON_SETTLE_TIME: Duration = Duration::from_millis(150);
pub const LIFT_POSITION_TOLERANCE: f64 = 3.0;
pub const PIVOT_OUTPUT_TOLERANCE: f64 = 2.0;
pub const DRIVE_POSITION_TOLERANCE_METERS: f64 = 0.005;
pub const DRIVE_VELOCITY_TOLERANCE: f64 = 0.025;
pub const AUTON_DRIVE_PID: (f64, f64, f64) = (3.0, 0.0, 0.08);
pub const AUTON_DRIVE_MAX_COMMAND: f64 = 0.35;
pub const AUTON_DRIVE_MIN_COMMAND: f64 = 0.12;
pub const LIFT_UP_RISE_PER_SECOND: f64 = 4.5;
pub const LIFT_DOWN_RISE_PER_SECOND: f64 = 1.75;
pub const LIFT_POSITION_GAIN: f64 = 0.01;
pub const DEPARTURE_PULSE_TIME: Duration = Duration::from_millis(600);
pub const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// Exclusive static tokens make duplicate port assignments a compile error.
pub fn hardware(
    peripherals: vexide::prelude::Peripherals,
) -> (
    vexide::prelude::Controller,
    vexide::display::Display,
    vexide::prelude::InertialSensor,
    [ozton::drivetrain::model::DriveMotor; 3],
    [ozton::drivetrain::model::DriveMotor; 3],
    crate::mechanisms::Mechanisms,
) {
    use crate::mechanisms::Mechanisms;
    use ozton::drivetrain::model::DriveMotor;
    use vexide::prelude::*;
    let left = [
        DriveMotor::new(
            Motor::new_exp(peripherals.port_5, Direction::Reverse),
            FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
    ];
    let right = [
        DriveMotor::new(
            Motor::new_exp(peripherals.port_20, Direction::Forward),
            FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
    ];
    let mechanisms = Mechanisms::new(
        Motor::new(peripherals.port_1, Gearset::Red, Direction::Forward),
        Motor::new_exp(peripherals.port_9, Direction::Reverse),
        // P10 main intake: forward draws pins inward.
        Motor::new_exp(peripherals.port_10, Direction::Forward),
        // P2 match-load roller: reversed wiring; loading direction is logical reverse.
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
        // Raw P8 increases downward; expose positive upward travel to control.
        RotationSensor::new(peripherals.port_8, Direction::Reverse),
        MECHANISM_CALIBRATION,
    );
    (
        peripherals.primary_controller,
        peripherals.display,
        InertialSensor::new(peripherals.port_7),
        left,
        right,
        mechanisms,
    )
}

/// Reject invalid tuning before constructing controllers or writing motor outputs.
pub fn valid() -> bool {
    MECHANISM_CALIBRATION.valid()
        && [
            DRIVE_WHEEL_DIAMETER_METERS,
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
            FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
            LIFT_POSITION_GAIN,
            LIFT_UP_RISE_PER_SECOND,
            LIFT_DOWN_RISE_PER_SECOND,
            DRIVE_POSITION_TOLERANCE_METERS,
            DRIVE_VELOCITY_TOLERANCE,
            LIFT_POSITION_TOLERANCE,
            AUTON_LIFT_ARRIVAL_TOLERANCE,
            AUTON_PIVOT_ARRIVAL_OUTPUT_DEGREES,
            PIVOT_OUTPUT_TOLERANCE,
            MAXIMUM_VOLTS,
            VELOCITY_FILTER_SECONDS,
            LIFT_UP_SLOWDOWN_DEGREES,
            LIFT_DOWN_SLOWDOWN_DEGREES,
            MATCH_LOAD_LIFT_TOLERANCE_DEGREES,
            MATCH_LOAD_PIVOT_TOLERANCE_OUTPUT_DEGREES,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        && [
            TIP_PID.0,
            TIP_PID.1,
            TIP_PID.2,
            TIP_POLARITY,
            TIP_THRESHOLD_RADIANS,
            TIP_MAXIMUM_CORRECTION,
            AUTON_DRIVE_PID.0,
            AUTON_DRIVE_PID.1,
            AUTON_DRIVE_PID.2,
            AUTON_FORWARD_METERS,
            AUTON_RETREAT_METERS,
            AUTON_LOWER_DEGREES,
            POSITION_GAIN,
            INTEGRAL_GAIN,
            VELOCITY_DAMPING,
        ]
        .iter()
        .all(|v| v.is_finite())
        && [
            DRIVE_OUTPUT_SCALE,
            AUTON_DRIVE_MAX_COMMAND,
            AUTON_DRIVE_MIN_COMMAND,
            LIFT_UP_MAXIMUM_COMMAND,
            LIFT_DOWN_MAXIMUM_COMMAND,
            LIFT_UP_APPROACH_COMMAND,
            LIFT_DOWN_APPROACH_COMMAND,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0 && *v <= 1.0)
        && AUTON_DRIVE_MIN_COMMAND <= AUTON_DRIVE_MAX_COMMAND
        && LIFT_UP_APPROACH_COMMAND <= LIFT_UP_MAXIMUM_COMMAND
        && LIFT_DOWN_APPROACH_COMMAND <= LIFT_DOWN_MAXIMUM_COMMAND
        && LIFT_RAISE_DIRECTION.abs() == 1.0
        && POSITION_GAIN >= 0.0
        && INTEGRAL_GAIN >= 0.0
        && VELOCITY_DAMPING >= 0.0
        && TIP_THRESHOLD_RADIANS >= 0.0
        && (0.0..=1.0).contains(&TIP_MAXIMUM_CORRECTION)
        && (0.0..=1.0).contains(&AUTON_RAISE_FRACTION)
        && AUTON_FORWARD_METERS >= 0.0
        && AUTON_RETREAT_METERS >= 0.0
        && AUTON_LOWER_DEGREES >= 0.0
        && LOW_HEIGHT_ABOVE_BOTTOM_DEGREES.is_finite()
        && LOW_HEIGHT_ABOVE_BOTTOM_DEGREES >= 0.0
        && B_PIVOT_EXTRA_OUTPUT_DEGREES.is_finite()
        && B_PIVOT_EXTRA_OUTPUT_DEGREES >= 0.0
        && (0.0..=30.0).contains(&PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES)
        && MAXIMUM_VOLTS <= 12.0
        && !CONTROL_INTERVAL.is_zero()
        && CONTROL_INTERVAL <= MAX_CONTROL_GAP
        && !AUTON_SETTLE_TIME.is_zero()
        && AUTON_SETTLE_TIME < AUTON_STAGE_TIMEOUT
        && !EJECT_TIME.is_zero()
        && EJECT_TIME < AUTON_STAGE_TIMEOUT
        && !AUTON_RELEASE_TIME.is_zero()
        && AUTON_RELEASE_TIME < AUTON_STAGE_TIMEOUT
        && AUTON_STAGE_TIMEOUT <= AUTON_TOTAL_TIMEOUT
        && MAX_RECORDING_FRAMES > 1
        && MAX_SAVED_ROUTES > 0
        && !MAX_RECORDING_DURATION.is_zero()
        && RECORDING_START_LIFT_TOLERANCE.is_finite()
        && RECORDING_START_LIFT_TOLERANCE > 0.0
        && RECORDING_START_PIVOT_TOLERANCE.is_finite()
        && RECORDING_START_PIVOT_TOLERANCE > 0.0
        && (0.0..1.0).contains(&DRIVE_INPUT_DEADBAND)
}

#[cfg(test)]
mod tests {
    #[test]
    fn robot_configuration_is_valid() {
        assert!(super::valid());
    }
}

// Recording storage and operator controls. SD writes happen only with outputs stopped.
pub const RECORDING_FORMAT_VERSION: u32 = 1;
pub const MAX_RECORDING_DURATION: Duration = Duration::from_secs(15);
pub const MAX_RECORDING_FRAMES: usize = 1600;
pub const MAX_RECORDING_BYTES: u64 = 2 * 1024 * 1024;
pub const MAX_ROUTE_INDEX_BYTES: u64 = 64 * 1024;
pub const MAX_SAVED_ROUTES: usize = 32;
pub const MAX_ROUTE_ID: u32 = 4096;
pub const RECORDING_START_LIFT_TOLERANCE: f64 = 6.0;
pub const RECORDING_START_PIVOT_TOLERANCE: f64 = 8.0;
pub const DRIVE_INPUT_DEADBAND: f64 = 0.03;

#[cfg(debug_assertions)]
pub const DEBUG_SAMPLE_INTERVAL: Duration = Duration::from_millis(200);

// B routine only: +5 degrees at the 48T output; B may reach 95 degrees.
pub const B_PIVOT_EXTRA_OUTPUT_DEGREES: f64 = 5.0;
