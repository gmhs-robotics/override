//! Recordable cascade, sensor-controlled intake pivot, main intake, and match-load roller.

use std::time::{Duration, Instant};

use ozton::{
    control::lift::{LiftController, LiftProfile},
    record::{
        self, Interpolate, RecordField,
        frame::RecordMode,
        rkyv::{Archive, Deserialize, Serialize},
    },
};
use vexide::{
    math::Angle,
    prelude::*,
    smart::{PortError, motor::BrakeMode},
};

use crate::mechanism_control::{
    Calibration, DepartureRollerPulse, LIFT_DOWN_MAXIMUM_COMMAND, LIFT_RAISE_DIRECTION,
    LIFT_UP_MAXIMUM_COMMAND, MATCH_LOAD_LIFT_TOLERANCE_DEGREES, limit_lift_command,
    match_load_roller_direction, plan,
};
use crate::pivot_control::{PIVOT_TRIM_MAX_OUTPUT_DEGREES, PivotController, PivotTrim};
use crate::recovery::StartupReference;

fn stop_once<M: MechanismMotor>(
    motor: &mut M,
    holding: &mut bool,
    mode: BrakeMode,
) -> Result<(), PortError> {
    if *holding {
        return Ok(());
    }
    let result = motor.brake(mode);
    *holding = result.is_ok();
    result
}

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = record::rkyv)]
pub struct MechanismFrame {
    pub lift_direction: i8,
    pub intake_direction: i8,
    pub pivot_raise: bool,
    pub pivot_lower: bool,
    pub pivot_trim_output_degrees: f64,
    pub lift_position_degrees: f64,
}

impl Interpolate for MechanismFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        let command = if amount < 0.5 { from } else { to };
        Self {
            lift_direction: command.lift_direction,
            intake_direction: command.intake_direction,
            pivot_raise: command.pivot_raise,
            pivot_lower: command.pivot_lower,
            pivot_trim_output_degrees: f64::interpolate(
                &from.pivot_trim_output_degrees,
                &to.pivot_trim_output_degrees,
                amount,
            ),
            lift_position_degrees: f64::interpolate(
                &from.lift_position_degrees,
                &to.lift_position_degrees,
                amount,
            ),
        }
    }
}

/// Motor operations used by the mechanism controller, also implemented by fault-test devices.
pub trait MechanismMotor {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError>;
    fn set_velocity(&mut self, rpm: i32) -> Result<(), PortError>;
    fn max_voltage(&self) -> f64;
    fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError>;
}

impl MechanismMotor for Motor {
    fn position(&self) -> Result<Angle, PortError> {
        Motor::position(self)
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        Motor::set_position(self, position)
    }
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError> {
        Motor::brake(self, mode)
    }
    fn set_velocity(&mut self, rpm: i32) -> Result<(), PortError> {
        Motor::set_velocity(self, rpm)
    }
    fn max_voltage(&self) -> f64 {
        Motor::max_voltage(self)
    }
    fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
        Motor::set_voltage(self, voltage)
    }
}

/// Accumulated rotation measured on the pivot motor shaft, before the 12T:48T gears.
pub trait PivotSensor {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
}

impl PivotSensor for RotationSensor {
    fn position(&self) -> Result<Angle, PortError> {
        RotationSensor::position(self)
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        RotationSensor::set_position(self, position)
    }
}

pub struct Mechanisms<M: MechanismMotor = Motor, S: PivotSensor = RotationSensor> {
    lift: M,
    pivot: M,
    intake: M,
    roller: M,
    pivot_sensor: S,
    calibration: Calibration,
    controller: LiftController,
    pivot_controller: PivotController,
    lift_holding: bool,
    pivot_holding: bool,
    pivot_trim: PivotTrim,
    intake_holding: bool,
    roller_holding: bool,
    departure_pulse: DepartureRollerPulse,
    last_update: Instant,
    lift_reference: StartupReference,
    pivot_reference: StartupReference,
}

impl<M: MechanismMotor, S: PivotSensor> Mechanisms<M, S> {
    pub fn new(
        lift: M,
        pivot: M,
        intake: M,
        roller: M,
        pivot_sensor: S,
        calibration: Calibration,
    ) -> Self {
        let mut hardware = Self {
            lift,
            pivot,
            intake,
            roller,
            pivot_sensor,
            calibration,
            controller: LiftController::new(LiftProfile {
                bottom_degrees: calibration.lift_bottom_degrees,
                top_degrees: calibration.lift_upper_limit(),
                up_maximum_command: LIFT_UP_MAXIMUM_COMMAND,
                down_maximum_command: LIFT_DOWN_MAXIMUM_COMMAND,
                up_rise_per_second: 4.5,
                down_rise_per_second: 1.75,
                position_gain: 0.01,
            }),
            pivot_controller: PivotController::new(),
            lift_holding: false,
            pivot_holding: false,
            pivot_trim: PivotTrim::default(),
            intake_holding: false,
            roller_holding: false,
            departure_pulse: DepartureRollerPulse::default(),
            last_update: Instant::now(),
            lift_reference: StartupReference::default(),
            pivot_reference: StartupReference::default(),
        };
        hardware.stop();
        hardware.initialize_references();
        hardware
    }

    fn initialize_references(&mut self) -> bool {
        if !self.calibration.valid() {
            return false;
        }
        // Both attempts run independently. An absent motor never blocks startup.
        // Hold before assigning a reference; retry only references never established.
        let lift = self.lift_reference.initialize(|| {
            self.lift.brake(BrakeMode::Hold).is_ok()
                && self
                    .lift
                    .set_position(Angle::from_degrees(
                        self.calibration.lift_bottom_degrees * LIFT_RAISE_DIRECTION,
                    ))
                    .is_ok()
        });
        let pivot_was_initialized = self.pivot_reference.is_initialized();
        // Reference accumulated position at the physically stowed pose. Do not
        // use wrapped angle(): one complete 12T turn is the full 90-degree pivot.
        let pivot = self.pivot_reference.initialize(|| {
            self.pivot.brake(BrakeMode::Hold).is_ok()
                && self
                    .pivot_sensor
                    .set_position(Angle::from_degrees(self.calibration.pivot_home_degrees))
                    .is_ok()
        });
        if pivot && !pivot_was_initialized {
            println!("Pivot rotation sensor zeroed: 0 degrees (start with intake stowed)");
        }
        lift && pivot
    }

    pub fn hold(&mut self) {
        self.stop();
    }

    fn stop_linked(&mut self) {
        self.departure_pulse.cancel();
        self.controller.reset();
        self.pivot_controller.reset();
        self.last_update = Instant::now();
        self.lift_holding = self.lift.brake(BrakeMode::Hold).is_ok();
        self.pivot_holding = self.pivot.brake(BrakeMode::Hold).is_ok();
        self.roller_holding = self.roller.brake(BrakeMode::Brake).is_ok();
    }

    fn stop(&mut self) {
        self.stop_linked();
        self.intake_holding = self.intake.brake(BrakeMode::Brake).is_ok();
    }

    fn run_intake(&mut self, direction: i8) {
        let _ = if direction == 0 {
            stop_once(&mut self.intake, &mut self.intake_holding, BrakeMode::Brake)
        } else {
            self.intake_holding = false;
            self.intake
                .set_voltage(f64::from(direction.signum()) * self.intake.max_voltage())
        };
    }

    fn apply(&mut self, frame: &MechanismFrame, mode: RecordMode) {
        let references_ready = self.initialize_references();
        // Read the lift and P8 independently. Never invent missing position readings.
        let lift_position = self.lift.position().ok().map(|value| value.as_degrees());
        let pivot_position = self
            .pivot_sensor
            .position()
            .ok()
            .map(|value| value.as_degrees());
        let (Some(lift_position), Some(pivot_position)) = (lift_position, pivot_position) else {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            return;
        };
        if !references_ready
            || !lift_position.is_finite()
            || !pivot_position.is_finite()
            || (mode == RecordMode::Playback && !frame.lift_position_degrees.is_finite())
        {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            return;
        }
        let lift_position = lift_position * LIFT_RAISE_DIRECTION;
        let playback_target = (mode == RecordMode::Playback).then(|| {
            (frame.lift_position_degrees * LIFT_RAISE_DIRECTION).clamp(
                self.calibration.lift_bottom_degrees,
                self.calibration.lift_upper_limit(),
            )
        });
        let requested_direction = match playback_target {
            Some(target) if (target - lift_position).abs() <= 3.0 => 0,
            Some(target) if target > lift_position => 1,
            Some(_) => -1,
            None => frame.lift_direction.signum(),
        };
        let Some(output) = plan(
            self.calibration,
            lift_position,
            pivot_position,
            requested_direction,
            frame.intake_direction,
        ) else {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            return;
        };

        let now = Instant::now();
        let elapsed = now.saturating_duration_since(self.last_update);
        let dt = elapsed.min(Duration::from_millis(100));
        self.last_update = now;
        let departing = self.departure_pulse.update(
            output.lift_direction > 0,
            (lift_position - self.calibration.lift_bottom_degrees).abs()
                <= MATCH_LOAD_LIFT_TOLERANCE_DEGREES,
            frame.intake_direction < 0,
            elapsed,
        );
        let intake_direction = if departing && frame.intake_direction == 0 {
            1
        } else {
            frame.intake_direction
        };
        self.run_intake(intake_direction);
        let roller_direction = if departing {
            -1
        } else {
            match_load_roller_direction(
                self.calibration,
                Some(lift_position),
                Some(pivot_position),
                frame.intake_direction,
            )
        };
        // Main intake failures do not prevent this roller command.
        let _ = if roller_direction == 0 {
            stop_once(&mut self.roller, &mut self.roller_holding, BrakeMode::Brake)
        } else {
            self.roller_holding = false;
            self.roller
                .set_voltage(f64::from(roller_direction) * self.roller.max_voltage())
        };
        let previous_trim = self.pivot_trim.output_degrees();
        let trim = if mode == RecordMode::Playback {
            if !self.pivot_trim.restore(frame.pivot_trim_output_degrees) {
                self.run_intake(frame.intake_direction);
                self.stop_linked();
                return;
            }
            self.pivot_trim.output_degrees()
        } else {
            self.pivot_trim
                .update(frame.pivot_raise, frame.pivot_lower, dt)
        };
        let pivot_offset = trim * self.calibration.pivot_motor_per_output_rev;
        let trim_moving = (trim - previous_trim).abs() > f64::EPSILON;
        let pivot_result = match self.pivot_controller.update(
            pivot_position,
            output.pivot_target_degrees + pivot_offset,
            self.calibration.pivot_home_degrees,
            self.calibration
                .pivot_target(self.calibration.lift_rotated_degrees)
                + PIVOT_TRIM_MAX_OUTPUT_DEGREES * self.calibration.pivot_motor_per_output_rev,
            output.lift_direction != 0 || trim_moving,
            dt,
        ) {
            Some(rpm) => {
                self.pivot_holding = false;
                self.pivot.set_velocity(rpm)
            }
            None => stop_once(&mut self.pivot, &mut self.pivot_holding, BrakeMode::Hold),
        };
        if pivot_result.is_err() {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            return;
        }
        let lift_result = if output.lift_direction == 0 {
            self.controller.reset();
            stop_once(&mut self.lift, &mut self.lift_holding, BrakeMode::Hold)
        } else {
            let command = self.controller.update(
                output.lift_direction,
                playback_target,
                Some(lift_position),
                dt,
            );
            let command = limit_lift_command(self.calibration, lift_position, command);
            if command == 0.0 {
                stop_once(&mut self.lift, &mut self.lift_holding, BrakeMode::Hold)
            } else {
                self.lift_holding = false;
                self.lift
                    .set_voltage(command * LIFT_RAISE_DIRECTION * self.lift.max_voltage())
            }
        };
        if lift_result.is_err() {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
        }
    }
}

#[record::async_trait(?Send)]
impl<M: MechanismMotor, S: PivotSensor> RecordField for Mechanisms<M, S> {
    type Output = MechanismFrame;

    async fn finalize_frame_value(&self, frame: &MechanismFrame) -> MechanismFrame {
        let mut output = frame.clone();
        output.pivot_trim_output_degrees = self.pivot_trim.output_degrees();
        // A missing encoder invalidates this playback target rather than recording
        // an invented bottom position that could command an unexpected descent.
        output.lift_position_degrees = if self.lift_reference.is_initialized() {
            self.lift
                .position()
                .map_or(f64::NAN, |position| position.as_degrees())
        } else {
            f64::NAN
        };
        output
    }

    async fn apply_frame_value(
        &mut self,
        frame: &MechanismFrame,
        mode: RecordMode,
    ) -> Result<(), PortError> {
        self.apply(frame, mode);
        // Missing hardware is retried on the next frame, including during playback.
        Ok(())
    }

    async fn stop_playback(&mut self) -> Result<(), PortError> {
        self.hold();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct FakeMotor {
        connected: bool,
        position: f64,
        voltage: f64,
        zero_count: usize,
        writes: usize,
        fail_writes: bool,
        velocity: i32,
        brakes: usize,
        last_brake: Option<BrakeMode>,
    }

    impl FakeMotor {
        fn connected() -> Self {
            Self {
                connected: true,
                ..Self::default()
            }
        }
        fn write(&mut self) -> Result<(), PortError> {
            self.writes += 1;
            if self.connected && !self.fail_writes {
                Ok(())
            } else {
                Err(PortError::Disconnected { port: 1 })
            }
        }
    }

    impl MechanismMotor for FakeMotor {
        fn position(&self) -> Result<Angle, PortError> {
            if self.connected {
                Ok(Angle::from_degrees(self.position))
            } else {
                Err(PortError::Disconnected { port: 1 })
            }
        }
        fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
            self.write()?;
            self.position = position.as_degrees();
            self.zero_count += 1;
            Ok(())
        }
        fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError> {
            self.write()?;
            self.voltage = 0.0;
            self.velocity = 0;
            self.brakes += 1;
            self.last_brake = Some(mode);
            Ok(())
        }
        fn set_velocity(&mut self, rpm: i32) -> Result<(), PortError> {
            self.write()?;
            self.velocity = rpm;
            Ok(())
        }
        fn max_voltage(&self) -> f64 {
            12.0
        }
        fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
            self.write()?;
            self.voltage = voltage;
            Ok(())
        }
    }

    struct FakeSensor {
        connected: bool,
        position: f64,
        zero_count: usize,
    }

    impl FakeSensor {
        fn advance(&mut self, rpm: i32) {
            self.position += f64::from(rpm) * 0.06;
        }
    }

    impl PivotSensor for FakeSensor {
        fn position(&self) -> Result<Angle, PortError> {
            if self.connected {
                Ok(Angle::from_degrees(self.position))
            } else {
                Err(PortError::Disconnected { port: 3 })
            }
        }
        fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
            self.position()?;
            self.position = position.as_degrees();
            self.zero_count += 1;
            Ok(())
        }
    }

    fn hardware(missing: Option<usize>) -> Mechanisms<FakeMotor, FakeSensor> {
        let mut motors = std::array::from_fn::<_, 3, _>(|_| FakeMotor::connected());
        if let Some(index) = missing {
            motors[index].connected = false;
        }
        let [lift, pivot, intake] = motors;
        Mechanisms::new(
            lift,
            pivot,
            intake,
            FakeMotor::connected(),
            FakeSensor {
                connected: true,
                position: 123.0,
                zero_count: 0,
            },
            // Baseline control fixtures exclude the separately tested alignment offset.
            Calibration {
                pivot_alignment_output_degrees: 0.0,
                ..crate::MECHANISM_CALIBRATION
            },
        )
    }

    fn frame() -> MechanismFrame {
        MechanismFrame {
            lift_direction: 1,
            intake_direction: 1,
            pivot_raise: false,
            pivot_lower: false,
            pivot_trim_output_degrees: 0.0,
            lift_position_degrees: -100.0,
        }
    }

    fn apply_test_frame(
        hardware: &mut Mechanisms<FakeMotor, FakeSensor>,
        command: &MechanismFrame,
        mode: RecordMode,
    ) {
        hardware.apply(command, mode);
    }

    fn tick(
        hardware: &mut Mechanisms<FakeMotor, FakeSensor>,
        command: &MechanismFrame,
        mode: RecordMode,
    ) {
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        apply_test_frame(hardware, command, mode);
        hardware.pivot_sensor.advance(hardware.pivot.velocity);
    }

    #[test]
    fn departure_roller_pulses_without_l1_in_live_and_playback_and_stops_on_fault() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            let command = MechanismFrame {
                lift_direction: 1,
                lift_position_degrees: -500.0,
                ..MechanismFrame::default()
            };
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, -12.0);
            assert_eq!(hardware.intake.voltage, 12.0);
            hardware.lift.position = -100.0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, -12.0);
            for _ in 0..65 {
                tick(&mut hardware, &command, mode);
            }
            assert_eq!(hardware.roller.voltage, 0.0);
            assert_eq!(hardware.intake.voltage, 0.0);
            // Still holding raise at bottom must not start another pulse.
            hardware.lift.position = 0.0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, 0.0);
            tick(&mut hardware, &MechanismFrame::default(), mode);
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, -12.0);
            let eject = MechanismFrame {
                intake_direction: -1,
                ..command.clone()
            };
            tick(&mut hardware, &eject, mode);
            // Pivot is no longer stowed: ejection cancels the pulse and the
            // ordinary reverse-assist gate keeps the roller stopped here.
            assert_eq!(hardware.roller.voltage, 0.0);
            tick(&mut hardware, &MechanismFrame::default(), mode);
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, -12.0);
            hardware.pivot_sensor.connected = false;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, 0.0);
            assert_eq!(hardware.intake.voltage, 0.0);
        }
    }

    #[test]
    fn departure_feeds_both_without_brake_toggling_and_cancels_on_output_fault() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            for failed in [0, 1] {
                let mut hardware = hardware(None);
                let command = MechanismFrame {
                    lift_direction: 1,
                    lift_position_degrees: -500.0,
                    ..MechanismFrame::default()
                };
                let intake_brakes = hardware.intake.brakes;
                for _ in 0..10 {
                    tick(&mut hardware, &command, mode);
                    assert_eq!(hardware.intake.voltage, 12.0);
                    assert_eq!(hardware.roller.voltage, -12.0);
                    assert_eq!(hardware.intake.brakes, intake_brakes);
                }
                if failed == 0 {
                    hardware.pivot.fail_writes = true;
                    // Force a changed target so the failing output is attempted.
                    hardware.lift.position = -100.0;
                } else {
                    hardware.lift.fail_writes = true;
                }
                tick(&mut hardware, &command, mode);
                assert_eq!(hardware.intake.voltage, 0.0);
                assert_eq!(hardware.roller.voltage, 0.0);
                assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
            }
        }
    }

    #[test]
    fn intake_uses_passive_brake_while_lift_and_pivot_hold() {
        let mut hardware = hardware(None);
        assert_eq!(hardware.intake.last_brake, Some(BrakeMode::Brake));
        assert_eq!(hardware.lift.last_brake, Some(BrakeMode::Hold));
        assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
        for mode in [RecordMode::Live, RecordMode::Playback] {
            for direction in [-1, 1] {
                let command = MechanismFrame {
                    intake_direction: direction,
                    ..MechanismFrame::default()
                };
                tick(&mut hardware, &command, mode);
                assert_eq!(hardware.intake.voltage, f64::from(direction) * 12.0);
                tick(&mut hardware, &MechanismFrame::default(), mode);
                assert_eq!(hardware.intake.voltage, 0.0);
                assert_eq!(hardware.intake.last_brake, Some(BrakeMode::Brake));
                let writes = hardware.intake.writes;
                for _ in 0..100 {
                    hardware.intake.position += 1.0;
                    tick(&mut hardware, &MechanismFrame::default(), mode);
                }
                assert_eq!(hardware.intake.writes, writes);
            }
        }
        hardware.hold();
        assert_eq!(hardware.intake.last_brake, Some(BrakeMode::Brake));
        assert_eq!(hardware.lift.last_brake, Some(BrakeMode::Hold));
        assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
    }

    #[test]
    fn main_intake_and_conditional_roller_work_independently_in_both_modes() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            let mut command = MechanismFrame {
                lift_direction: 0,
                lift_position_degrees: 0.0,
                ..frame()
            };
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.intake.voltage, 12.0);
            assert_eq!(hardware.roller.voltage, -12.0);
            hardware.lift.position = -500.0;
            hardware.pivot_sensor.position = 360.0;
            command.lift_position_degrees = -500.0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.intake.voltage, 12.0);
            assert_eq!(hardware.roller.voltage, 0.0);
            hardware.lift.position = 0.0;
            hardware.pivot_sensor.position = 0.0;
            command.intake_direction = -1;
            command.lift_position_degrees = 0.0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.intake.voltage, -12.0);
            assert_eq!(hardware.roller.voltage, 12.0);
            command.intake_direction = 0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, 0.0);
            command.intake_direction = 1;
            hardware.intake.fail_writes = true;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, -12.0);
            hardware.pivot_sensor.connected = false;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.roller.voltage, 0.0);
        }
    }

    #[test]
    fn scoring_sequence_retains_extra_tilt_through_ejection_then_a_returns_to_normal() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        let mut command = MechanismFrame::default();
        for _ in 0..400 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert!((hardware.pivot_sensor.position / 4.0 - 90.0).abs() < 0.5);
        command.pivot_raise = true;
        for _ in 0..200 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        command.pivot_raise = false;
        for _ in 0..300 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 45.0);
        assert!((hardware.pivot_sensor.position / 4.0 - 135.0).abs() < 0.5);
        assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
        command.intake_direction = -1;
        for _ in 0..100 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert_eq!(hardware.intake.voltage, -12.0);
        assert_eq!(hardware.pivot.velocity, 0);
        assert_eq!(hardware.pivot_trim.output_degrees(), 45.0);
        command.pivot_lower = true;
        for _ in 0..101 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        command.pivot_lower = false;
        for _ in 0..300 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 0.0);
        assert!((hardware.pivot_sensor.position / 4.0 - 90.0).abs() < 0.5);
        assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
    }

    #[test]
    fn scoring_trim_allows_135_output_degrees_and_clamps_in_live_or_playback() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            hardware.lift.position = -500.0;
            let command = MechanismFrame {
                pivot_raise: true,
                pivot_trim_output_degrees: 45.0,
                lift_direction: 0,
                lift_position_degrees: -500.0,
                ..frame()
            };
            for _ in 0..700 {
                tick(&mut hardware, &command, mode);
                assert!(hardware.pivot_sensor.position <= 540.0);
            }
            assert!((hardware.pivot_sensor.position - 540.0).abs() < 2.0);
            assert_eq!(hardware.pivot.velocity, 0);
        }
    }

    #[test]
    fn extended_pivot_latches_hold_without_repeated_drop_and_catchup_commands() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            hardware.lift.position = -500.0;
            let command = MechanismFrame {
                lift_position_degrees: -500.0,
                ..MechanismFrame::default()
            };
            for _ in 0..400 {
                tick(&mut hardware, &command, mode);
            }
            assert_eq!(hardware.pivot.velocity, 0);
            assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
            let writes = hardware.pivot.writes;
            for tick_index in 0..1000 {
                // Load deflection or sensor noise must never release onboard Hold.
                hardware.pivot_sensor.position = if tick_index % 2 == 0 { 340.0 } else { 357.0 };
                tick(&mut hardware, &command, mode);
                assert_eq!(hardware.pivot.velocity, 0);
                assert_eq!(hardware.pivot.writes, writes);
                assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
            }
            hardware.hold();
            assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
            assert_eq!(hardware.roller.last_brake, Some(BrakeMode::Brake));
            assert_eq!(hardware.intake.last_brake, Some(BrakeMode::Brake));
        }
    }

    #[test]
    fn startup_near_wrap_zeroes_once_and_full_shaft_turn_is_not_stowed() {
        for raw_start in [0.0, -359.99, 359.99, 720.0] {
            let mut hardware = Mechanisms::new(
                FakeMotor::connected(),
                FakeMotor::connected(),
                FakeMotor::connected(),
                FakeMotor::connected(),
                FakeSensor {
                    connected: true,
                    position: raw_start,
                    zero_count: 0,
                },
                crate::MECHANISM_CALIBRATION,
            );
            assert_eq!(hardware.pivot_sensor.position, 0.0);
            hardware.lift.position = -500.0;
            hardware.pivot_sensor.position = 360.0;
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            hardware.apply(&frame(), RecordMode::Live);
            assert_eq!(hardware.pivot.velocity, 0);
            assert_eq!(hardware.roller.voltage, 0.0);
            assert_eq!(hardware.pivot_sensor.position, 360.0);
            assert_eq!(hardware.pivot_sensor.zero_count, 1);
        }
    }

    #[test]
    fn p8_zeroes_once_and_is_the_only_pivot_position_source() {
        let mut hardware = hardware(None);
        assert_eq!(hardware.pivot_sensor.position, 0.0);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
        assert_eq!(hardware.pivot.zero_count, 0);
        hardware.lift.position = -(500.0);
        // P8 alone determines whether the target needs further upward movement.
        hardware.pivot_sensor.position = 180.0;
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        hardware.apply(&frame(), RecordMode::Live);
        assert!(hardware.pivot.velocity > 0);
        hardware.pivot_sensor.position = 360.0;
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        hardware.apply(&frame(), RecordMode::Live);
        assert_eq!(hardware.pivot.velocity, 0);
        assert!(hardware.lift.voltage < 0.0);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
    }

    #[test]
    fn missing_rotation_sensor_holds_linked_motion_and_recovers_without_rezero() {
        let mut hardware = hardware(None);
        hardware.lift.position = -(135.0);
        hardware.pivot_sensor.position = 145.0;
        hardware.pivot_sensor.connected = false;
        hardware.apply(&frame(), RecordMode::Live);
        assert_eq!(hardware.lift.voltage, 0.0);
        assert_eq!(hardware.pivot.velocity, 0);
        assert_eq!(hardware.intake.voltage, 12.0);
        hardware.pivot_sensor.connected = true;
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        hardware.apply(&frame(), RecordMode::Live);
        assert!(hardware.lift.voltage < 0.0);
        assert_eq!(hardware.pivot_sensor.position, 145.0);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
    }

    #[test]
    fn cascade_negative_limit_blocks_raising_but_allows_lowering() {
        for position in [-1100.0, -1101.0, -1800.0] {
            let mut hardware = hardware(None);
            hardware.lift.position = position;
            tick(&mut hardware, &frame(), RecordMode::Live);
            assert_eq!(hardware.lift.voltage, 0.0);
            let lower = MechanismFrame {
                lift_direction: -1,
                ..frame()
            };
            tick(&mut hardware, &lower, RecordMode::Live);
            assert!(hardware.lift.voltage > 0.0);
        }
        let mut hardware = hardware(None);
        let lower = MechanismFrame {
            lift_direction: -1,
            ..frame()
        };
        tick(&mut hardware, &lower, RecordMode::Live);
        assert_eq!(hardware.lift.voltage, 0.0);
    }

    #[test]
    fn cascade_ramps_and_tapers_before_negative_limit_in_live_and_playback() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            hardware.lift.position = -500.0;
            let command = MechanismFrame {
                lift_position_degrees: -2000.0,
                ..frame()
            };
            tick(&mut hardware, &command, mode);
            let first = hardware.lift.voltage;
            assert!(first < 0.0 && first > -0.6);
            tick(&mut hardware, &command, mode);
            assert!(hardware.lift.voltage < first);
            for _ in 0..30 {
                tick(&mut hardware, &command, mode);
            }
            assert!((hardware.lift.voltage + 12.0).abs() < 1e-9);
            hardware.lift.position = -1050.0;
            tick(&mut hardware, &command, mode);
            // Playback also slows for its position target (50 degrees away).
            let expected = if mode == RecordMode::Playback {
                6.0
            } else {
                6.72
            };
            assert!((hardware.lift.voltage + expected).abs() < 1e-9);
            hardware.lift.position = -1100.0;
            tick(&mut hardware, &command, mode);
            assert_eq!(hardware.lift.voltage, 0.0);
            assert_eq!(hardware.pivot_sensor.zero_count, 1);
        }
    }

    #[test]
    fn absent_rotation_sensor_at_boot_retries_zero_before_motion() {
        let mut hardware = Mechanisms::new(
            FakeMotor::connected(),
            FakeMotor::connected(),
            FakeMotor::connected(),
            FakeMotor::connected(),
            FakeSensor {
                connected: false,
                position: 123.0,
                zero_count: 0,
            },
            crate::MECHANISM_CALIBRATION,
        );
        hardware.apply(&frame(), RecordMode::Live);
        assert_eq!(hardware.lift.voltage, 0.0);
        assert_eq!(hardware.intake.voltage, 12.0);
        hardware.pivot_sensor.connected = true;
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        hardware.apply(&frame(), RecordMode::Live);
        assert_eq!(hardware.pivot_sensor.position, 0.0);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
        assert!(hardware.lift.voltage < 0.0);
    }

    #[test]
    fn up_adjustment_stays_after_release_and_down_reduces_it() {
        let mut hardware = hardware(None);
        hardware.lift.position = -67.5;
        hardware.pivot_sensor.position = 145.0;
        let mut command = MechanismFrame {
            pivot_raise: true,
            ..MechanismFrame::default()
        };
        for _ in 0..300 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 45.0);
        command.pivot_raise = false;
        for _ in 0..400 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 45.0);
        assert!((hardware.pivot_sensor.position / 4.0 - 81.25).abs() < 0.5);
        command.pivot_lower = true;
        for index in 1..=400 {
            tick(&mut hardware, &command, RecordMode::Live);
            if index == 33 {
                assert!((hardware.pivot_trim.output_degrees() - 30.0).abs() < 0.2);
                assert!((hardware.pivot_sensor.position / 4.0 - 66.25).abs() < 1.0);
            }
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 0.0);
        assert!((hardware.pivot_sensor.position / 4.0 - 36.25).abs() < 0.5);
    }

    #[test]
    fn fifteen_degree_alignment_correction_does_not_fall_back_after_release_or_stop() {
        let mut hardware = hardware(None);
        hardware.lift.position = -67.5;
        hardware.pivot_sensor.position = 145.0;
        let mut command = MechanismFrame {
            pivot_raise: true,
            ..MechanismFrame::default()
        };
        for _ in 0..33 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        command.pivot_raise = false;
        for _ in 0..200 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        let retained = hardware.pivot_trim.output_degrees();
        assert!((retained - 15.0).abs() < 0.2);
        // A physical mount sitting 15 degrees low now stays flat after Up is released.
        assert!((hardware.pivot_sensor.position / 4.0 - 15.0 - 36.25).abs() < 0.5);
        hardware.hold();
        assert_eq!(hardware.pivot_trim.output_degrees(), retained);
        hardware.pivot.connected = false;
        tick(&mut hardware, &command, RecordMode::Live);
        assert_eq!(hardware.pivot_trim.output_degrees(), retained);
        hardware.pivot.connected = true;
        for _ in 0..100 {
            tick(&mut hardware, &command, RecordMode::Live);
        }
        assert!((hardware.pivot_sensor.position / 4.0 - 15.0 - 36.25).abs() < 0.5);
    }

    #[test]
    fn playback_uses_recorded_trim_without_accumulating_existing_correction() {
        let mut hardware = hardware(None);
        hardware.lift.position = -67.5;
        hardware.pivot_sensor.position = 145.0;
        assert!(hardware.pivot_trim.restore(30.0));
        let command = MechanismFrame {
            pivot_raise: true,
            pivot_trim_output_degrees: 15.0,
            lift_position_degrees: -67.5,
            ..MechanismFrame::default()
        };
        for _ in 0..200 {
            tick(&mut hardware, &command, RecordMode::Playback);
        }
        assert_eq!(hardware.pivot_trim.output_degrees(), 15.0);
        assert!((hardware.pivot_sensor.position / 4.0 - 51.25).abs() < 0.5);
    }

    #[test]
    fn one_12t_tooth_adds_30_shaft_degrees_without_changing_zero_or_upper_limit() {
        let calibration = crate::MECHANISM_CALIBRATION;
        assert_eq!(calibration.pivot_alignment_output_degrees, 7.5);
        assert_eq!(calibration.pivot_home_degrees, 0.0);
        assert_eq!(calibration.pivot_target(0.0), 30.0);
        assert_eq!(calibration.pivot_target(500.0), 360.0);
        let mut hardware = hardware(None);
        hardware.calibration = calibration;
        for _ in 0..400 {
            tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        }
        assert!((hardware.pivot_sensor.position - 30.0).abs() <= 2.0);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
        assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
    }

    #[test]
    fn pivot_reaches_flat_twice_as_fast_without_changing_output_limit() {
        let c = crate::MECHANISM_CALIBRATION;
        assert!((c.pivot_target(67.5) - 175.0).abs() < 1e-9);
        assert!((c.pivot_target(135.0) - 320.0).abs() < 1e-9);
        assert!((c.lift_rotated_degrees - 167.5862068966).abs() < 1e-8);
        assert_eq!(c.pivot_target(500.0), 360.0);
    }

    #[test]
    fn pivot_raise_interpolates_as_a_button_without_partial_offsets() {
        let released = MechanismFrame::default();
        let pressed = MechanismFrame {
            pivot_raise: true,
            ..MechanismFrame::default()
        };
        assert!(!MechanismFrame::interpolate(&released, &pressed, 0.49).pivot_raise);
        assert!(MechanismFrame::interpolate(&released, &pressed, 0.5).pivot_raise);
    }

    #[test]
    fn each_missing_motor_allows_startup_and_other_outputs() {
        for missing in 0..3 {
            let mut hardware = hardware(Some(missing));
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
            if missing != 2 {
                assert_eq!(hardware.intake.voltage, 12.0);
            }
            if missing >= 2 {
                assert!(hardware.lift.voltage < 0.0);
            }
            if missing <= 1 {
                assert_eq!(hardware.lift.voltage, 0.0);
            }
        }
    }

    #[test]
    fn absent_encoders_recover_without_restarting_robot() {
        for missing in 0..2 {
            let mut hardware = hardware(Some(missing));
            apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
            hardware.lift.connected = true;
            hardware.pivot.connected = true;
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
            assert!(hardware.lift.voltage < 0.0);
            assert_eq!(hardware.lift.zero_count, 1);
            assert_eq!(hardware.pivot.zero_count, 0);
            assert_eq!(hardware.pivot_sensor.zero_count, 1);
        }
    }

    #[test]
    fn mid_travel_disconnect_preserves_encoder_reference() {
        let mut hardware = hardware(None);
        hardware.lift.position = -(135.0);
        hardware.pivot_sensor.position = 145.0;
        hardware.lift.connected = false;
        apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
        assert_eq!(hardware.intake.voltage, 12.0);
        hardware.lift.connected = true;
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
        assert!(hardware.lift.voltage < 0.0);
        assert_eq!(hardware.lift.position, -135.0);
        assert_eq!(hardware.lift.zero_count, 1);
    }

    #[test]
    fn invalid_calibration_and_recording_do_not_panic_or_block_roller() {
        let mut hardware = hardware(None);
        hardware.calibration.lift_bottom_degrees = f64::NAN;
        apply_test_frame(&mut hardware, &frame(), RecordMode::Playback);
        assert_eq!(hardware.intake.voltage, 12.0);
        assert_eq!(hardware.lift.voltage, 0.0);
        hardware.calibration = crate::MECHANISM_CALIBRATION;
        let mut invalid = frame();
        invalid.lift_position_degrees = f64::NAN;
        apply_test_frame(&mut hardware, &invalid, RecordMode::Playback);
        assert_eq!(hardware.intake.voltage, 12.0);
        assert_eq!(hardware.lift.voltage, 0.0);
    }

    #[test]
    fn all_stop_outputs_are_attempted_despite_failures() {
        let mut hardware = hardware(None);
        hardware.lift.connected = false;
        hardware.pivot.connected = false;
        hardware.intake.voltage = 12.0;
        hardware.hold();
        assert_eq!(hardware.intake.voltage, 0.0);
    }

    #[test]
    fn held_mechanisms_do_not_repeat_brake_commands_for_encoder_noise() {
        let mut hardware = hardware(None);
        let before = (
            hardware.lift.brakes,
            hardware.pivot.brakes,
            hardware.intake.brakes,
        );
        let neutral = MechanismFrame::default();
        for tick in 0..100 {
            hardware.lift.position = -(if tick % 2 == 0 { 0.5 } else { 0.0 });
            hardware.pivot_sensor.position = 0.25;
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &neutral, RecordMode::Live);
        }
        assert_eq!(
            (
                hardware.lift.brakes,
                hardware.pivot.brakes,
                hardware.intake.brakes
            ),
            before
        );
    }

    #[test]
    fn held_raise_button_and_noisy_stationary_lift_do_not_toggle_pivot_commands() {
        let mut hardware = hardware(None);
        hardware.lift.position = -67.5;
        hardware.pivot_sensor.position = 145.0;
        let command = frame();
        tick(&mut hardware, &command, RecordMode::Live);
        let writes = hardware.pivot.writes;
        let brakes = hardware.pivot.brakes;
        for tick_index in 0..500 {
            hardware.lift.position = if tick_index % 2 == 0 { -69.5 } else { -65.5 };
            hardware.pivot_sensor.position = if tick_index % 2 == 0 { 147.0 } else { 143.0 };
            tick(&mut hardware, &command, RecordMode::Live);
            assert_eq!(hardware.pivot.velocity, 0);
        }
        assert_eq!(hardware.pivot.writes, writes);
        assert_eq!(hardware.pivot.brakes, brakes);
    }

    #[test]
    fn lift_reaches_full_command_without_pivot_error_throttling() {
        for direction in [-1, 1] {
            let mut hardware = hardware(None);
            hardware.lift.position = -(500.0);
            hardware.pivot_sensor.position = 300.0;
            let command = MechanismFrame {
                lift_direction: direction,
                ..MechanismFrame::default()
            };
            let before = hardware.lift.brakes;
            for _ in 0..25 {
                hardware.last_update = Instant::now() - Duration::from_millis(10);
                apply_test_frame(&mut hardware, &command, RecordMode::Live);
                assert!(hardware.lift.voltage * f64::from(direction) < 0.0);
            }
            let expected = if direction > 0 {
                LIFT_UP_MAXIMUM_COMMAND
            } else {
                -LIFT_DOWN_MAXIMUM_COMMAND
            };
            assert!((hardware.lift.voltage + expected * hardware.lift.max_voltage()).abs() < 1e-9);
            assert_eq!(hardware.lift.brakes, before);
        }
    }

    #[test]
    fn pivot_tracks_moving_lift_with_velocity_commands_and_holds_after_release() {
        let mut hardware = hardware(None);
        let command = frame();
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        apply_test_frame(&mut hardware, &command, RecordMode::Live);
        let before = hardware.pivot.brakes;
        for tick in 1..50 {
            hardware.lift.position = -(f64::from(tick) * 4.0);
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &command, RecordMode::Live);
            if tick > 2 {
                assert!(hardware.pivot.velocity > 0);
            }
            hardware.pivot_sensor.advance(hardware.pivot.velocity);
        }
        assert_eq!(hardware.pivot.brakes, before);
        let neutral = MechanismFrame::default();
        for _ in 0..150 {
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &neutral, RecordMode::Live);
            hardware.pivot_sensor.advance(hardware.pivot.velocity);
        }
        assert_eq!(hardware.pivot.velocity, 0);
        assert_eq!(hardware.pivot.brakes, before + 1);
        assert!(
            (hardware.pivot_sensor.position
                - hardware.calibration.pivot_target(-hardware.lift.position))
            .abs()
                <= 2.0
        );
    }

    #[test]
    fn pivot_write_failure_holds_linked_outputs_but_not_intake() {
        let mut hardware = hardware(None);
        hardware.pivot_sensor.position = 20.0;
        hardware.pivot.fail_writes = true;
        apply_test_frame(&mut hardware, &frame(), RecordMode::Live);
        assert_eq!(hardware.lift.voltage, 0.0);
        assert_eq!(hardware.intake.voltage, 12.0);
    }
    #[test]
    fn all_devices_missing_does_not_return_recording_errors() {
        use std::task::{Context, Poll, Waker};
        let mut hardware = hardware(None);
        hardware.lift.connected = false;
        hardware.pivot.connected = false;
        hardware.intake.connected = false;
        let command = frame();
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut future = hardware.apply_frame_value(&command, mode);
            assert!(matches!(
                future
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop())),
                Poll::Ready(Ok(()))
            ));
        }
        let mut future = hardware.stop_playback();
        assert!(matches!(
            future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop())),
            Poll::Ready(Ok(()))
        ));
    }
}
