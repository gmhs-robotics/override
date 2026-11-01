//! Recordable cascade, sensor-controlled intake pivot, main intake, and match-load roller.

#[cfg(test)]
use std::time::Duration;
use std::time::Instant;

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

use crate::config::{
    LIFT_DOWN_RISE_PER_SECOND, LIFT_POSITION_GAIN, LIFT_POSITION_TOLERANCE,
    LIFT_UP_RISE_PER_SECOND, MAX_CONTROL_GAP,
};
use crate::low_score::LowScore;
use crate::mechanism_control::{
    Calibration, DepartureRollerPulse, LIFT_DOWN_MAXIMUM_COMMAND, LIFT_RAISE_DIRECTION,
    LIFT_UP_MAXIMUM_COMMAND, MATCH_LOAD_LIFT_TOLERANCE_DEGREES, limit_lift_command,
    match_load_roller_direction, plan,
};
use crate::pivot_control::{PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES, PivotController};
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

/// Retained P8 angle preset, independent of cascade height once selected.
#[derive(Archive, Serialize, Deserialize, Default, Clone, Copy, Debug, PartialEq)]
#[rkyv(crate = record::rkyv)]
pub enum PivotSelection {
    #[default]
    Follow,
    Pickup,
    Place,
}

impl PivotSelection {
    fn select(&mut self, pickup: bool, place: bool, follow: bool) {
        match (pickup, place, follow) {
            (true, false, false) => *self = Self::Pickup,
            (false, true, false) => *self = Self::Place,
            (false, false, true) => *self = Self::Follow,
            _ => {}
        }
    }

    fn target(self, calibration: Calibration, following: f64) -> f64 {
        match self {
            Self::Follow => following,
            Self::Pickup => following, // Legacy recordings never grant extra travel.
            Self::Place => calibration.pivot_limit(0.0),
        }
    }
}

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = record::rkyv)]
pub struct MechanismFrame {
    pub roller_override_direction: Option<i8>,
    pub lift_direction: i8,
    pub intake_direction: i8,
    pub pivot_raise: bool,
    pub pivot_lower: bool,
    pub pivot_follow: bool,
    pub pivot_selection: PivotSelection,
    pub low_score: bool,
    pub low_score_cancel: bool,
    pub pivot_override_degrees: Option<f64>,
    pub lift_position_degrees: f64,
}

impl Interpolate for MechanismFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        let command = if amount < 0.5 { from } else { to };
        Self {
            lift_position_degrees: f64::interpolate(
                &from.lift_position_degrees,
                &to.lift_position_degrees,
                amount,
            ),
            ..command.clone()
        }
    }
}

/// Motor operations used by the mechanism controller, also implemented by fault-test devices.
pub trait MechanismMotor {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError>;
    fn max_voltage(&self) -> f64;
    #[cfg(debug_assertions)]
    fn debug_measurements(&self) -> (f64, f64, f64) {
        (f64::NAN, f64::NAN, f64::NAN)
    }
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
    fn max_voltage(&self) -> f64 {
        Motor::max_voltage(self)
    }
    #[cfg(debug_assertions)]
    fn debug_measurements(&self) -> (f64, f64, f64) {
        (
            self.voltage().unwrap_or(f64::NAN),
            self.current().unwrap_or(f64::NAN),
            self.temperature().unwrap_or(f64::NAN),
        )
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
    pivot_selection: PivotSelection,
    intake_holding: bool,
    roller_holding: bool,
    departure_pulse: DepartureRollerPulse,
    low_score: LowScore,
    low_score_pivot: Option<f64>,
    intake_direction: i8,
    roller_direction: i8,
    last_update: Instant,
    #[cfg(debug_assertions)]
    debug: crate::diagnostics::MechanismDebug,
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
                up_rise_per_second: LIFT_UP_RISE_PER_SECOND,
                down_rise_per_second: LIFT_DOWN_RISE_PER_SECOND,
                position_gain: LIFT_POSITION_GAIN,
            }),
            pivot_controller: PivotController::new(),
            lift_holding: false,
            pivot_holding: false,
            pivot_selection: PivotSelection::Follow,
            intake_holding: false,
            roller_holding: false,
            departure_pulse: DepartureRollerPulse::default(),
            low_score: LowScore::default(),
            low_score_pivot: None,
            intake_direction: 0,
            roller_direction: 0,
            last_update: Instant::now(),
            #[cfg(debug_assertions)]
            debug: crate::diagnostics::MechanismDebug::default(),
            lift_reference: StartupReference::default(),
            pivot_reference: StartupReference::default(),
        };
        hardware.stop();
        hardware.initialize_references();
        hardware
    }

    fn initialize_references(&mut self) -> bool {
        if !self.calibration.valid() {
            #[cfg(debug_assertions)]
            self.debug.fault("M01");
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
        #[cfg(debug_assertions)]
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
        #[cfg(debug_assertions)]
        if pivot && !pivot_was_initialized {
            crate::diagnostics::event!(
                "pivot_reference_set,p8_zero_deg={:.3}",
                self.calibration.pivot_home_degrees
            );
        }
        lift && pivot
    }

    fn stop_linked(&mut self) {
        // Stops and mode transitions discard command state, never sensor references.
        // An earlier scoring pose must not survive into ordinary driver control.
        self.roller_direction = 0;
        self.low_score_pivot = None;
        self.pivot_selection = PivotSelection::Follow;
        self.low_score.cancel();
        self.departure_pulse.cancel();
        self.controller.reset();
        self.pivot_controller.reset();
        self.last_update = Instant::now();
        #[cfg(debug_assertions)]
        {
            self.debug.lift_command_v = 0.0;
            self.debug.pivot_command_v = 0.0;
            self.debug.lift_target = None;
            self.debug.pivot_target = None;
        }
        let _ = stop_once(&mut self.lift, &mut self.lift_holding, BrakeMode::Hold);
        let _ = stop_once(&mut self.pivot, &mut self.pivot_holding, BrakeMode::Hold);
        let _ = stop_once(&mut self.roller, &mut self.roller_holding, BrakeMode::Brake);
    }

    pub fn stop(&mut self) {
        self.stop_linked();
        let _ = stop_once(&mut self.intake, &mut self.intake_holding, BrakeMode::Brake);
        self.intake_direction = 0;
    }

    fn run_intake(&mut self, direction: i8) -> bool {
        self.intake_direction = direction.signum();
        let result = if direction == 0 {
            stop_once(&mut self.intake, &mut self.intake_holding, BrakeMode::Brake)
        } else {
            self.intake_holding = false;
            self.intake
                .set_voltage(f64::from(direction.signum()) * self.intake.max_voltage())
        };
        result.is_ok()
    }

    /// Positive upward lift travel and accumulated P8 angle, never motor pivot feedback.
    pub fn pose(&self) -> Option<(f64, f64)> {
        if !self.lift_reference.is_initialized() || !self.pivot_reference.is_initialized() {
            return None;
        }
        let height = self.lift.position().ok()?.as_degrees() * LIFT_RAISE_DIRECTION;
        let pivot = self.pivot_sensor.position().ok()?.as_degrees();
        (height.is_finite() && pivot.is_finite()).then_some((height, pivot))
    }

    /// Debug-only sample: sensed voltage/current/temperature are distinct from requested volts.
    #[cfg(debug_assertions)]
    pub fn debug_sample(&self, seq: u64, ms: u128) {
        let raw_lift = self
            .lift
            .position()
            .ok()
            .map(|v| v.as_degrees())
            .unwrap_or(f64::NAN);
        let pivot = self
            .pivot_sensor
            .position()
            .ok()
            .map(|v| v.as_degrees())
            .unwrap_or(f64::NAN);
        let height = raw_lift * LIFT_RAISE_DIRECTION;
        let target = self.debug.pivot_target.unwrap_or(f64::NAN);
        let output = (pivot - self.calibration.pivot_home_degrees)
            / self.calibration.pivot_sensor_per_output_rev;
        println!(
            "DBG,MECH,seq={seq},t_ms={ms},lift_raw_deg={raw_lift:.3},lift_up_deg={height:.3},lift_target_deg={:.3},lift_err_deg={:.3},p8_deg={pivot:.3},pivot_output_deg={output:.3},p8_target_deg={target:.3},p8_err_deg={:.3},lift_request_v={:.3},pivot_request_v={:.3},lift_hold={},pivot_hold={},intake_dir={},roller_dir={},lift_ref={},pivot_ref={},fault={},fault_count={}",
            self.debug.lift_target.unwrap_or(f64::NAN),
            self.debug
                .lift_target
                .map(|t| t - height)
                .unwrap_or(f64::NAN),
            target - pivot,
            self.debug.lift_command_v,
            self.debug.pivot_command_v,
            u8::from(self.lift_holding),
            u8::from(self.pivot_holding),
            self.intake_direction,
            self.roller_direction,
            u8::from(self.lift_reference.is_initialized()),
            u8::from(self.pivot_reference.is_initialized()),
            self.debug.fault,
            self.debug.fault_count
        );
        for (name, motor) in [
            ("lift_P1", &self.lift),
            ("pivot_P9", &self.pivot),
            ("intake_P10", &self.intake),
            ("roller_P2", &self.roller),
        ] {
            let (v, a, c) = motor.debug_measurements();
            println!(
                "DBG,MOTOR,seq={seq},t_ms={ms},name={name},measured_v={v:.3},current_a={a:.3},temp_c={c:.2}"
            );
        }
    }

    /// Persist actual sensor pose and applied intake state after the live control tick.
    pub fn recording_snapshot(&self, frame: &MechanismFrame) -> Option<MechanismFrame> {
        let (height, pivot) = self.pose()?;
        Some(MechanismFrame {
            roller_override_direction: Some(self.roller_direction),
            low_score: self.low_score_pivot.is_some(),
            lift_position_degrees: height * LIFT_RAISE_DIRECTION,
            pivot_override_degrees: Some(pivot),
            pivot_selection: self.pivot_selection,
            intake_direction: self.intake_direction,
            ..frame.clone()
        })
    }

    pub fn autonomous_target(&mut self, height: f64, pivot: Option<f64>, eject: bool) -> bool {
        self.apply(
            &MechanismFrame {
                lift_position_degrees: height * LIFT_RAISE_DIRECTION,
                pivot_override_degrees: pivot,
                intake_direction: if eject { -1 } else { 0 },
                ..MechanismFrame::default()
            },
            RecordMode::Playback,
        )
    }

    /// Returns false on invalid feedback or failed outputs. Recording tolerates
    /// device failures; a scripted autonomous can use this result to abort.
    pub fn apply(&mut self, frame: &MechanismFrame, mode: RecordMode) -> bool {
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
            #[cfg(debug_assertions)]
            self.debug.fault("M02");
            return false;
        };
        if !references_ready
            || !lift_position.is_finite()
            || !pivot_position.is_finite()
            || (mode == RecordMode::Playback && !frame.lift_position_degrees.is_finite())
        {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            #[cfg(debug_assertions)]
            self.debug.fault("M03");
            return false;
        }
        let lift_position = lift_position * LIFT_RAISE_DIRECTION;
        let now = Instant::now();
        let elapsed = now.saturating_duration_since(self.last_update);
        if elapsed > MAX_CONTROL_GAP {
            self.stop();
            #[cfg(debug_assertions)]
            self.debug.fault("M04");
            return false;
        }
        let dt = elapsed;
        self.last_update = now;
        let manual = frame.low_score_cancel
            || frame.lift_direction != 0
            || frame.intake_direction != 0
            || frame.pivot_raise
            || frame.pivot_lower
            || frame.pivot_follow;
        if mode != RecordMode::Playback
            && (frame.lift_direction != 0
                || frame.pivot_raise
                || frame.pivot_lower
                || frame.pivot_follow)
        {
            self.low_score_pivot = None;
        }
        let low_score_command = if mode == RecordMode::Playback {
            self.low_score.cancel();
            None
        } else {
            match self.low_score.update(
                frame.low_score,
                manual,
                lift_position,
                pivot_position.clamp(
                    self.calibration.pivot_home_degrees,
                    self.calibration.pivot_limit(0.0),
                ),
                self.calibration,
                elapsed,
            ) {
                Ok(command) => command,
                Err(()) => {
                    self.run_intake(0);
                    self.stop_linked();
                    #[cfg(debug_assertions)]
                    self.debug.fault("M05");
                    return false;
                }
            }
        };
        if let Some(command) = &low_score_command {
            self.low_score_pivot = Some(command.pivot_target);
        }
        let pivot_override = if mode == RecordMode::Playback {
            frame.pivot_override_degrees
        } else {
            self.low_score_pivot
        };
        if pivot_override.is_some_and(|target| !target.is_finite()) {
            self.run_intake(0);
            self.stop_linked();
            #[cfg(debug_assertions)]
            self.debug.fault("M06");
            return false;
        }
        let b_scoring_pose = if mode == RecordMode::Playback {
            frame.low_score
        } else {
            self.low_score_pivot.is_some()
        };
        let scoring_extra = if b_scoring_pose {
            crate::config::B_PIVOT_EXTRA_OUTPUT_DEGREES
        } else {
            0.0
        };
        let pivot_override = pivot_override.map(|target| {
            target.clamp(
                self.calibration.pivot_home_degrees,
                self.calibration.pivot_limit(scoring_extra),
            )
        });
        let playback_target = low_score_command
            .as_ref()
            .map(|command| command.lift_target)
            .or_else(|| {
                (mode == RecordMode::Playback).then(|| {
                    (frame.lift_position_degrees * LIFT_RAISE_DIRECTION).clamp(
                        self.calibration.lift_bottom_degrees,
                        self.calibration.lift_upper_limit(),
                    )
                })
            });
        let requested_direction = match playback_target {
            Some(target) if (target - lift_position).abs() <= LIFT_POSITION_TOLERANCE => 0,
            Some(target) if target > lift_position => 1,
            Some(_) => -1,
            None => frame.lift_direction.signum(),
        };
        let Some(output) = plan(
            self.calibration,
            lift_position,
            pivot_position,
            requested_direction,
        ) else {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            #[cfg(debug_assertions)]
            self.debug.fault("M07");
            return false;
        };

        let pulse = self.departure_pulse.update(
            output.lift_direction > 0,
            (lift_position - self.calibration.lift_bottom_degrees).abs()
                <= MATCH_LOAD_LIFT_TOLERANCE_DEGREES,
            frame.intake_direction < 0,
            elapsed,
        );
        let departing = (mode != RecordMode::Playback || frame.lift_direction != 0)
            && low_score_command.is_none()
            && pivot_override.is_none()
            && pulse;
        if low_score_command.is_some() {
            self.departure_pulse.cancel();
        }
        let intake_direction = if let Some(command) = &low_score_command {
            command.intake_direction
        } else if departing && frame.intake_direction == 0 {
            1
        } else {
            frame.intake_direction
        };
        let intake_ok = self.run_intake(intake_direction);
        let roller_direction =
            if mode == RecordMode::Playback && frame.roller_override_direction.is_some() {
                frame.roller_override_direction.unwrap_or(0).signum()
            } else if low_score_command.is_some() || pivot_override.is_some() {
                0
            } else if departing {
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
        let roller_result = if roller_direction == 0 {
            stop_once(&mut self.roller, &mut self.roller_holding, BrakeMode::Brake)
        } else {
            self.roller_holding = false;
            self.roller
                .set_voltage(f64::from(roller_direction) * self.roller.max_voltage())
        };
        self.roller_direction = if roller_result.is_ok() {
            roller_direction
        } else {
            0
        };
        if mode == RecordMode::Playback {
            self.pivot_selection = match frame.pivot_selection {
                PivotSelection::Pickup => PivotSelection::Follow,
                selection => selection,
            };
        } else {
            // X is a held override, never a latched permission to exceed 90 degrees.
            if self.pivot_selection == PivotSelection::Pickup {
                self.pivot_selection = PivotSelection::Follow;
            }
            self.pivot_selection
                .select(false, frame.pivot_lower, frame.pivot_follow);
        }
        if requested_direction < 0 && pivot_override.is_none() {
            self.pivot_selection = PivotSelection::Follow;
        }
        // Manual lowering is the return-to-start command, including at bottom.
        // It wins over held placement/X inputs so those cannot prevent closure.
        let returning_home = mode == RecordMode::Live && frame.lift_direction < 0;
        let extra_allowed = mode == RecordMode::Live && frame.pivot_raise && !returning_home;
        let top = self.calibration.pivot_limit(if extra_allowed {
            PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES
        } else if b_scoring_pose
            && pivot_override.is_some_and(|target| target > self.calibration.pivot_limit(0.0))
        {
            crate::config::B_PIVOT_EXTRA_OUTPUT_DEGREES
        } else {
            0.0
        });
        let target = if extra_allowed {
            top
        } else {
            pivot_override.unwrap_or_else(|| {
                self.pivot_selection
                    .target(self.calibration, output.pivot_target_degrees)
            })
        }
        .clamp(self.calibration.pivot_home_degrees, top);
        #[cfg(debug_assertions)]
        {
            self.debug.lift_target = playback_target;
            self.debug.pivot_target = Some(target);
            self.debug.lift_command_v = 0.0;
        }
        let pivot_result = match self.pivot_controller.update(
            pivot_position,
            target,
            self.calibration.pivot_home_degrees,
            top,
            dt,
        ) {
            Some(voltage) => {
                #[cfg(debug_assertions)]
                {
                    self.debug.pivot_command_v = voltage;
                }
                self.pivot_holding = false;
                self.pivot.set_voltage(voltage)
            }
            None => {
                self.stop();
                #[cfg(debug_assertions)]
                self.debug.fault("M08");
                return false;
            }
        };
        if pivot_result.is_err() {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            #[cfg(debug_assertions)]
            self.debug.fault("M09");
            return false;
        }
        let lift_result = if output.lift_direction == 0 {
            self.controller.reset();
            stop_once(&mut self.lift, &mut self.lift_holding, BrakeMode::Hold)
        } else {
            let command =
                self.controller
                    .update(output.lift_direction, None, Some(lift_position), dt);
            // Keep ozton's ramp, but use this robot's configurable position tolerance
            // and friction floor rather than the library's fixed playback deadband.
            let command = if let Some(target) = playback_target {
                let floor = if command > 0.0 {
                    crate::config::LIFT_UP_APPROACH_COMMAND
                } else {
                    crate::config::LIFT_DOWN_APPROACH_COMMAND
                };
                command.signum()
                    * command
                        .abs()
                        .min(((target - lift_position).abs() * LIFT_POSITION_GAIN).max(floor))
            } else {
                command
            };
            let command = limit_lift_command(self.calibration, lift_position, command);
            if command == 0.0 {
                stop_once(&mut self.lift, &mut self.lift_holding, BrakeMode::Hold)
            } else {
                self.lift_holding = false;
                #[cfg(debug_assertions)]
                {
                    self.debug.lift_command_v =
                        command * LIFT_RAISE_DIRECTION * self.lift.max_voltage();
                }
                self.lift
                    .set_voltage(command * LIFT_RAISE_DIRECTION * self.lift.max_voltage())
            }
        };
        if lift_result.is_err() {
            self.run_intake(frame.intake_direction);
            self.stop_linked();
            #[cfg(debug_assertions)]
            self.debug.fault("M10");
            return false;
        }
        #[cfg(debug_assertions)]
        {
            if !intake_ok {
                self.debug.fault("M11_INTAKE_WRITE");
            } else if roller_result.is_err() {
                self.debug.fault("M12_ROLLER_WRITE");
            } else {
                self.debug.recovered();
            }
        }
        intake_ok && roller_result.is_ok()
    }
}

#[record::async_trait(?Send)]
impl<M: MechanismMotor, S: PivotSensor> RecordField for Mechanisms<M, S> {
    type Output = MechanismFrame;

    async fn finalize_frame_value(&self, frame: &MechanismFrame) -> MechanismFrame {
        let mut output = frame.clone();
        output.pivot_selection = self.pivot_selection;
        output.pivot_override_degrees = self.low_score_pivot;
        output.intake_direction = self.intake_direction;
        output.roller_override_direction = Some(self.roller_direction);
        output.low_score = self.low_score_pivot.is_some();
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
        self.stop();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autonomous_commands_keep_feedback_running_and_driver_can_return_home() {
        let mut hardware = hardware(None);
        let raised = crate::config::MECHANISM_CALIBRATION.lift_upper_limit()
            * crate::config::AUTON_RAISE_FRACTION;
        let flat = crate::config::MECHANISM_CALIBRATION.pivot_limit(0.0);
        for (target, pivot, eject) in [
            (raised, None, false),
            (raised, Some(flat), false),
            (
                raised - crate::config::AUTON_LOWER_DEGREES,
                Some(flat),
                false,
            ),
            (
                raised - crate::config::AUTON_LOWER_DEGREES,
                Some(flat),
                true,
            ),
            (raised, Some(flat), true),
            (raised, Some(flat), false),
        ] {
            for _ in 0..500 {
                hardware.last_update = Instant::now() - Duration::from_millis(10);
                assert!(hardware.autonomous_target(target, pivot, eject));
                hardware.pivot_sensor.advance(hardware.pivot.voltage);
                hardware.lift.position =
                    (hardware.lift.position + hardware.lift.voltage * 0.5).clamp(-950.0, 0.0);
                assert!(hardware.intake.voltage <= 0.0);
                assert_eq!(hardware.roller.voltage, 0.0);
            }
            assert!(
                (-hardware.lift.position - target).abs() <= crate::config::LIFT_POSITION_TOLERANCE
            );
            assert_eq!(hardware.intake.voltage, if eject { -12.0 } else { 0.0 });
        }
        hardware.stop();
        let lower = MechanismFrame {
            lift_direction: -1,
            ..Default::default()
        };
        for _ in 0..1800 {
            tick(&mut hardware, &lower, RecordMode::Live);
            hardware.lift.position =
                (hardware.lift.position + hardware.lift.voltage * 0.5).min(0.0);
        }
        assert!(hardware.lift.position.abs() < 0.02);
        assert!(hardware.pivot_sensor.position.abs() < 0.75);
        assert_eq!(hardware.lift.zero_count, 1);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
    }

    #[derive(Default)]
    struct FakeMotor {
        connected: bool,
        position: f64,
        voltage: f64,
        zero_count: usize,
        writes: usize,
        fail_writes: bool,
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
            self.brakes += 1;
            self.last_brake = Some(mode);
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
        fn advance(&mut self, voltage: f64) {
            self.position += voltage * 1.5;
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
            crate::MECHANISM_CALIBRATION,
        )
    }

    fn frame() -> MechanismFrame {
        MechanismFrame {
            roller_override_direction: None,
            lift_direction: 1,
            intake_direction: 1,
            pivot_raise: false,
            pivot_lower: false,
            pivot_follow: false,
            pivot_selection: PivotSelection::Follow,
            low_score: false,
            low_score_cancel: false,
            pivot_override_degrees: None,
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
        hardware.pivot_sensor.advance(hardware.pivot.voltage);
    }

    #[test]
    fn b_low_score_lowers_unloads_and_returns_without_folding_or_inward_assist() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 360.0;
        let mut command = MechanismFrame {
            low_score: true,
            ..MechanismFrame::default()
        };
        let mut ejected = false;
        let mut reached_low_height = false;
        for step in 0..1200 {
            tick(&mut hardware, &command, RecordMode::Live);
            hardware.lift.position =
                (hardware.lift.position + hardware.lift.voltage * 0.5).clamp(-950.0, 0.0);
            command.low_score = false;
            assert!(hardware.pivot_sensor.position <= 381.0);
            assert_eq!(hardware.roller.voltage, 0.0);
            assert!(hardware.lift.position <= 0.0);
            if (hardware.lift.position + crate::config::LOW_HEIGHT_ABOVE_BOTTOM_DEGREES).abs()
                <= crate::config::POSITION_TOLERANCE
            {
                reached_low_height = true;
            }
            if hardware.intake.voltage < 0.0 {
                ejected = true;
                assert!(reached_low_height);
            }
            assert!(
                hardware.intake.voltage <= 0.0,
                "departure intake assist must remain disabled"
            );
            if step == 1199 {
                assert!((hardware.lift.position + 500.0).abs() <= 3.0);
                assert_eq!(hardware.lift.voltage, 0.0);
                assert_eq!(hardware.intake.voltage, 0.0);
            }
        }
        assert!(reached_low_height && ejected);
        assert_eq!(hardware.low_score_pivot, Some(360.0));
    }

    #[test]
    fn b_allows_95_output_degrees_and_recorded_b_pose_preserves_that_limit() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 360.0;
        tick(
            &mut hardware,
            &MechanismFrame {
                low_score: true,
                ..Default::default()
            },
            RecordMode::Live,
        );
        for _ in 0..400 {
            tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        }
        assert!((hardware.pivot_sensor.position - 380.0).abs() < 1.0);
        let saved = hardware
            .recording_snapshot(&MechanismFrame::default())
            .unwrap();
        assert!(saved.low_score);
        assert!((saved.pivot_override_degrees.unwrap() - 380.0).abs() < 1.0);
        hardware.stop();
        for _ in 0..200 {
            tick(&mut hardware, &saved, RecordMode::Playback);
        }
        assert!((hardware.pivot_sensor.position - 380.0).abs() < 1.0);
        let normal = MechanismFrame {
            low_score: false,
            ..saved
        };
        tick(&mut hardware, &normal, RecordMode::Playback);
        assert!(hardware.pivot.voltage < 0.0); // Ordinary auton cannot inherit B permission.
        hardware.stop();
        tick(
            &mut hardware,
            &MechanismFrame {
                lift_direction: -1,
                ..Default::default()
            },
            RecordMode::Live,
        );
        assert_eq!(hardware.low_score_pivot, None);
        assert!(hardware.lift.voltage > 0.0);
    }

    #[test]
    fn b_cancel_holds_pivot_and_manual_tilt_restores_normal_controls() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 360.0;
        let button = MechanismFrame {
            low_score: true,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &button, RecordMode::Live);
        tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        hardware.lift.position = -300.0;
        tick(&mut hardware, &button, RecordMode::Live);
        assert_eq!(hardware.lift.voltage, 0.0);
        assert_eq!(hardware.low_score_pivot, Some(380.0));
        let follow = MechanismFrame {
            pivot_follow: true,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &follow, RecordMode::Live);
        assert_eq!(hardware.low_score_pivot, None);
    }

    #[test]
    fn low_score_recording_captures_automatic_ejection_and_pivot_and_both_buttons_can_cancel() {
        use std::task::{Context, Poll, Waker};
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 360.0;
        tick(
            &mut hardware,
            &MechanismFrame {
                low_score: true,
                ..MechanismFrame::default()
            },
            RecordMode::Live,
        );
        hardware.lift.position = -crate::config::LOW_HEIGHT_ABOVE_BOTTOM_DEGREES;
        let neutral = MechanismFrame::default();
        tick(&mut hardware, &neutral, RecordMode::Live);
        {
            let mut future = hardware.finalize_frame_value(&neutral);
            let Poll::Ready(recorded) = future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
            else {
                panic!("frame finalization must complete immediately");
            };
            assert_eq!(recorded.intake_direction, -1);
            assert_eq!(recorded.pivot_override_degrees, Some(380.0));
        }
        // Raw override input remains true even when opposing buttons make their
        // direction neutral, so either pair of shoulder buttons still cancels.
        tick(
            &mut hardware,
            &MechanismFrame {
                low_score_cancel: true,
                ..MechanismFrame::default()
            },
            RecordMode::Live,
        );
        assert_eq!(hardware.lift.voltage, 0.0);
        assert_eq!(hardware.intake.voltage, 0.0);
        assert_eq!(hardware.low_score_pivot, Some(380.0));
    }

    #[test]
    fn recorded_low_score_pose_keeps_pivot_out_and_never_restarts_departure_feeding() {
        let mut hardware = hardware(None);
        hardware.pivot_sensor.position = 360.0;
        let command = MechanismFrame {
            low_score: true,
            pivot_override_degrees: Some(360.0),
            lift_position_degrees: -500.0,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &command, RecordMode::Playback);
        assert!(hardware.lift.voltage < 0.0);
        assert_eq!(hardware.intake.voltage, 0.0);
        assert_eq!(hardware.roller.voltage, 0.0);
        assert!(hardware.pivot.voltage.abs() < 0.1);
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
        hardware.stop();
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
    fn lowering_cancels_presets_and_returns_to_p8_zero() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            for preset in [PivotSelection::Pickup, PivotSelection::Place] {
                let mut hardware = hardware(None);
                hardware.calibration = crate::MECHANISM_CALIBRATION;
                hardware.lift.position = -500.0;
                hardware.pivot_sensor.position = 360.0;
                hardware.pivot_selection = preset;
                let command = MechanismFrame {
                    lift_direction: -1,
                    lift_position_degrees: 0.0,
                    pivot_selection: preset,
                    ..MechanismFrame::default()
                };
                for step in (0..=100).rev() {
                    hardware.lift.position = -f64::from(step) * 5.0;
                    tick(&mut hardware, &command, mode);
                }
                let stow = MechanismFrame {
                    pivot_selection: PivotSelection::Follow,
                    ..MechanismFrame::default()
                };
                for _ in 0..600 {
                    tick(&mut hardware, &stow, mode);
                }
                assert_eq!(hardware.pivot_selection, PivotSelection::Follow);
                assert!(hardware.pivot_sensor.position.abs() <= 0.75);
                assert_eq!(hardware.pivot.last_brake, Some(BrakeMode::Hold));
            }
        }
    }

    #[test]
    fn manual_lowering_closes_pivot_despite_held_presets_and_old_scoring_pose() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 480.0;
        hardware.pivot_selection = PivotSelection::Place;
        hardware.low_score_pivot = Some(360.0);
        let lower = MechanismFrame {
            lift_direction: -1,
            pivot_raise: true,
            pivot_lower: true,
            low_score: true,
            ..MechanismFrame::default()
        };
        for _ in 0..2400 {
            tick(&mut hardware, &lower, RecordMode::Live);
            hardware.lift.position =
                (hardware.lift.position + hardware.lift.voltage * 0.5).min(0.0);
        }
        assert!(hardware.lift.position.abs() <= crate::config::LIFT_ENDPOINT_TOLERANCE_DEGREES);
        assert!(hardware.pivot_sensor.position.abs() < 0.75);
        assert_eq!(hardware.low_score_pivot, None);
        assert_eq!(hardware.pivot_selection, PivotSelection::Follow);
        // Once home, ordinary controls remain usable in both directions.
        let raise = MechanismFrame {
            lift_direction: 1,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &raise, RecordMode::Live);
        assert!(hardware.lift.voltage < 0.0);
        let intake = MechanismFrame {
            intake_direction: 1,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &intake, RecordMode::Live);
        assert!(hardware.intake.voltage > 0.0);
        let eject = MechanismFrame {
            intake_direction: -1,
            ..MechanismFrame::default()
        };
        tick(&mut hardware, &eject, RecordMode::Live);
        assert!(hardware.intake.voltage < 0.0);
    }

    #[test]
    fn stopping_discards_scoring_pose_without_referencing_sensors_again() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        hardware.pivot_sensor.position = 360.0;
        tick(
            &mut hardware,
            &MechanismFrame {
                low_score: true,
                ..MechanismFrame::default()
            },
            RecordMode::Live,
        );
        assert_eq!(hardware.low_score_pivot, Some(380.0));
        hardware.pivot_selection = PivotSelection::Place;
        hardware.stop();
        assert_eq!(hardware.low_score_pivot, None);
        assert_eq!(hardware.pivot_selection, PivotSelection::Follow);
        hardware.lift.position = 0.0;
        tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        assert!(hardware.pivot.voltage < 0.0);
        assert_eq!(hardware.lift.zero_count, 1);
        assert_eq!(hardware.pivot_sensor.zero_count, 1);
    }

    #[test]
    fn x_adds_30_output_degrees_only_while_held_then_returns_to_follow() {
        let mut hardware = hardware(None);
        hardware.lift.position = -500.0;
        let held = MechanismFrame {
            pivot_raise: true,
            ..MechanismFrame::default()
        };
        for _ in 0..500 {
            tick(&mut hardware, &held, RecordMode::Live);
        }
        // 30 degrees on 48T output means 120 additional P8 degrees.
        assert!((hardware.pivot_sensor.position - 480.0).abs() < 1.0);
        let released = MechanismFrame::default();
        tick(&mut hardware, &released, RecordMode::Live);
        assert!(hardware.pivot.voltage < 0.0);
        for _ in 0..500 {
            tick(&mut hardware, &released, RecordMode::Live);
        }
        assert!((hardware.pivot_sensor.position - 360.0).abs() < 1.0);
        assert_eq!(hardware.pivot_selection, PivotSelection::Follow);
        // A stored pose above 90 cannot bypass the live-X-only limit.
        let archived = MechanismFrame {
            pivot_override_degrees: Some(480.0),
            pivot_selection: PivotSelection::Pickup,
            pivot_raise: true,
            lift_position_degrees: -500.0,
            ..MechanismFrame::default()
        };
        for _ in 0..500 {
            tick(&mut hardware, &archived, RecordMode::Playback);
        }
        assert!((hardware.pivot_sensor.position - 360.0).abs() < 1.0);
    }

    #[test]
    fn recording_does_not_latch_x_after_release() {
        use std::task::{Context, Poll, Waker};
        let mut hardware = hardware(None);
        tick(
            &mut hardware,
            &MechanismFrame {
                pivot_raise: true,
                ..MechanismFrame::default()
            },
            RecordMode::Live,
        );
        let released = MechanismFrame::default();
        tick(&mut hardware, &released, RecordMode::Live);
        let mut future = hardware.finalize_frame_value(&released);
        let Poll::Ready(recorded) = future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
        else {
            panic!("frame finalization must complete immediately");
        };
        assert_eq!(recorded.pivot_selection, PivotSelection::Follow);
        assert!(!recorded.pivot_raise);
    }

    #[test]
    fn playback_presets_cannot_grant_extra_pivot_travel() {
        let mut hardware = hardware(None);
        for (selection, target) in [
            (PivotSelection::Pickup, 0.0),
            (PivotSelection::Place, 360.0),
        ] {
            let command = MechanismFrame {
                pivot_selection: selection,
                pivot_raise: true,
                lift_position_degrees: 0.0,
                ..MechanismFrame::default()
            };
            for _ in 0..500 {
                tick(&mut hardware, &command, RecordMode::Playback);
            }
            assert!((hardware.pivot_sensor.position - target).abs() < 2.0);
        }
    }

    #[test]
    fn conflicting_preset_buttons_keep_current_selection_and_interpolation_is_discrete() {
        let mut selection = PivotSelection::Pickup;
        selection.select(true, true, false);
        selection.select(false, false, false);
        assert_eq!(selection, PivotSelection::Pickup);
        let from = MechanismFrame {
            pivot_selection: PivotSelection::Pickup,
            ..MechanismFrame::default()
        };
        let to = MechanismFrame {
            pivot_selection: PivotSelection::Place,
            ..MechanismFrame::default()
        };
        assert_eq!(
            MechanismFrame::interpolate(&from, &to, 0.49).pivot_selection,
            PivotSelection::Pickup
        );
        assert_eq!(
            MechanismFrame::interpolate(&from, &to, 0.5).pivot_selection,
            PivotSelection::Place
        );
    }

    #[test]
    fn p8_deflection_is_corrected_after_every_up_down_cycle_in_both_modes() {
        for mode in [RecordMode::Live, RecordMode::Playback] {
            let mut hardware = hardware(None);
            for _ in 0..3 {
                for (lift, target) in [(-500.0, 360.0), (0.0, 0.0)] {
                    hardware.lift.position = lift;
                    let command = MechanismFrame {
                        lift_position_degrees: lift,
                        ..MechanismFrame::default()
                    };
                    for _ in 0..600 {
                        tick(&mut hardware, &command, mode);
                    }
                    assert!((hardware.pivot_sensor.position - target).abs() < 1.0);
                    hardware.pivot_sensor.position =
                        if target > 0.0 { target - 15.0 } else { 15.0 };
                    tick(&mut hardware, &command, mode);
                    assert!(
                        hardware.pivot.voltage * (target - hardware.pivot_sensor.position) > 0.0
                    );
                    for _ in 0..600 {
                        tick(&mut hardware, &command, mode);
                    }
                    assert!((hardware.pivot_sensor.position - target).abs() < 1.0);
                }
            }
            assert_eq!(hardware.pivot_sensor.zero_count, 1);
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
            assert!(hardware.pivot.voltage.abs() < 0.1);
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
        assert!(hardware.pivot.voltage > 0.0);
        hardware.pivot_sensor.position = 360.0;
        hardware.pivot_controller.reset();
        hardware.last_update = Instant::now() - Duration::from_millis(10);
        hardware.apply(&frame(), RecordMode::Live);
        assert!(hardware.pivot.voltage.abs() < 0.1);
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
        assert!(hardware.pivot.voltage.abs() < 0.1);
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
        for position in [-950.0, -951.0, -1800.0] {
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
            hardware.lift.position = -925.0;
            tick(&mut hardware, &command, mode);
            // Playback also slows for its position target (25 degrees away).
            let expected = if mode == RecordMode::Playback {
                3.0
            } else {
                6.72
            };
            assert!((hardware.lift.voltage + expected).abs() < 1e-9);
            hardware.lift.position = -950.0;
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
    fn default_following_stays_within_90_degree_endpoint_until_pickup_is_pressed() {
        let calibration = crate::MECHANISM_CALIBRATION;
        assert_eq!(calibration.pivot_target(0.0), 0.0);
        for travel in 0..=950 {
            let target = calibration.pivot_target(f64::from(travel));
            assert!((0.0..=360.0).contains(&target));
            assert_eq!(PivotSelection::Follow.target(calibration, target), target);
        }
        assert_eq!(
            PivotSelection::Follow.target(calibration, calibration.pivot_target(950.0)),
            360.0
        );
        let mut selection = PivotSelection::Follow;
        selection.select(false, false, false);
        assert_eq!(selection, PivotSelection::Follow);
        selection.select(true, false, false);
        assert_eq!(selection.target(calibration, 0.0), 0.0);
        selection.select(false, true, false);
        assert_eq!(selection.target(calibration, 0.0), 360.0);
    }

    #[test]
    fn pivot_reaches_90_degree_endpoint_at_configured_lift_height() {
        let c = crate::MECHANISM_CALIBRATION;
        assert!((c.pivot_target(67.5) - 145.0 * 1.0).abs() < 1e-9);
        assert!((c.pivot_target(135.0) - 290.0 * 1.0).abs() < 1e-9);
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
        assert!(hardware.run_intake(1));
        hardware.stop();
        assert_eq!(hardware.intake.voltage, 0.0);
    }

    #[test]
    fn recording_replays_the_applied_match_load_roller_not_a_recomputed_gate() {
        let mut hardware = hardware(None);
        let input = MechanismFrame {
            intake_direction: 1,
            ..Default::default()
        };
        tick(&mut hardware, &input, RecordMode::Live);
        assert_eq!(hardware.roller.voltage, -12.0);
        let snapshot = hardware.recording_snapshot(&input).unwrap();
        assert_eq!(snapshot.roller_override_direction, Some(-1));
        hardware.stop();
        tick(&mut hardware, &snapshot, RecordMode::Playback);
        assert_eq!(hardware.roller.voltage, -12.0);
        assert_eq!(hardware.intake.voltage, 12.0);
        hardware.stop();
        assert_eq!(hardware.roller.voltage, 0.0);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn debug_targets_voltages_and_fault_counts_match_the_controller() {
        let mut hardware = hardware(None);
        hardware.lift.position = -100.0;
        hardware.pivot_sensor.position = 30.0;
        tick(
            &mut hardware,
            &MechanismFrame {
                lift_direction: 1,
                ..Default::default()
            },
            RecordMode::Live,
        );
        assert_eq!(hardware.debug.lift_target, None);
        assert!(
            (hardware.debug.pivot_target.unwrap() - hardware.calibration.pivot_target(100.0)).abs()
                < 1e-9
        );
        assert_eq!(hardware.debug.pivot_command_v, hardware.pivot.voltage);
        assert_eq!(hardware.debug.lift_command_v, hardware.lift.voltage);
        assert_eq!(hardware.debug.fault_count, 0);
        hardware.pivot_sensor.connected = false;
        tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        assert_eq!(hardware.debug.fault, "M02");
        assert_eq!(hardware.debug.fault_count, 1);
        tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        assert_eq!(hardware.debug.fault_count, 2);
        hardware.pivot_sensor.connected = true;
        tick(&mut hardware, &MechanismFrame::default(), RecordMode::Live);
        assert!(hardware.debug.fault.is_empty());
        assert_eq!(hardware.debug.fault_count, 2);
        hardware.stop();
        assert_eq!(hardware.debug.pivot_target, None);
        assert_eq!(hardware.debug.lift_command_v, 0.0);
    }

    #[test]
    fn repeated_stops_keep_hold_reference_without_reissuing_brakes() {
        let mut hardware = hardware(None);
        hardware.lift.position = -300.0;
        tick(&mut hardware, &frame(), RecordMode::Live);
        hardware.stop();
        let brakes = (
            hardware.lift.brakes,
            hardware.pivot.brakes,
            hardware.intake.brakes,
        );
        for _ in 0..100 {
            hardware.stop();
        }
        assert_eq!(
            (
                hardware.lift.brakes,
                hardware.pivot.brakes,
                hardware.intake.brakes
            ),
            brakes
        );
        // A failed stop stays pending and retries rather than becoming a held state.
        hardware.lift_holding = false;
        hardware.lift.fail_writes = true;
        hardware.stop();
        assert!(!hardware.lift_holding);
        hardware.lift.fail_writes = false;
        hardware.stop();
        assert!(hardware.lift_holding);
    }

    #[test]
    fn recorded_pose_replays_after_manual_capture_and_driver_can_return_home() {
        let mut hardware = hardware(None);
        let mut capture = crate::recording::Capture::new(crate::recording::RecordTarget::New);
        for step in 0..60 {
            hardware.lift.position = -f64::from(step) * 3.0;
            let input = MechanismFrame {
                intake_direction: if step > 30 { -1 } else { 0 },
                ..Default::default()
            };
            tick(&mut hardware, &input, RecordMode::Live);
            let snapshot = hardware.recording_snapshot(&input).unwrap();
            assert_eq!(snapshot.intake_direction, input.intake_direction);
            capture
                .push(
                    crate::recording::RobotFrame {
                        version: crate::config::RECORDING_FORMAT_VERSION,
                        drivetrain: ozton::record::DifferentialVoltageFrame::tank(0.0, 0.0),
                        mechanisms: snapshot.clone(),
                        boost: false,
                    },
                    Duration::from_millis(10),
                )
                .unwrap();
            assert!(hardware.apply(&snapshot, RecordMode::Playback));
        }
        assert!(capture.finish().is_some());
        hardware.stop();
        for _ in 0..1600 {
            tick(
                &mut hardware,
                &MechanismFrame {
                    lift_direction: -1,
                    ..Default::default()
                },
                RecordMode::Live,
            );
            hardware.lift.position =
                (hardware.lift.position + hardware.lift.voltage * 0.5).min(0.0);
        }
        assert!(hardware.lift.position.abs() < 0.02);
        assert!(hardware.pivot_sensor.position.abs() < 1.0);
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
    fn pivot_tracks_moving_lift_and_continues_p8_position_control_after_release() {
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
                assert!(hardware.pivot.voltage > 0.0);
            }
            hardware.pivot_sensor.advance(hardware.pivot.voltage);
        }
        assert_eq!(hardware.pivot.brakes, before);
        let neutral = MechanismFrame::default();
        for _ in 0..150 {
            hardware.last_update = Instant::now() - Duration::from_millis(10);
            apply_test_frame(&mut hardware, &neutral, RecordMode::Live);
            hardware.pivot_sensor.advance(hardware.pivot.voltage);
        }
        assert!(hardware.pivot.voltage.abs() < 0.1);
        assert_eq!(hardware.pivot.brakes, before);
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
