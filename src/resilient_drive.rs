//! Competition policy: keep every available drive motor running and retry missing motors.

use ozton::drivetrain::model::{CoupledDifferential, DrivetrainModel, Tank};
use std::{cell::Cell, rc::Rc};
use vexide::smart::PortError;

pub trait AdjustableDrive: Tank<Error = PortError> {
    fn set_output_scale(&mut self, scale: f64);
    fn brake_now(&mut self) -> Result<(), PortError> {
        self.stop_now()
    }
}

impl AdjustableDrive for CoupledDifferential {
    fn set_output_scale(&mut self, scale: f64) {
        CoupledDifferential::set_output_scale(self, scale);
    }
    fn brake_now(&mut self) -> Result<(), PortError> {
        CoupledDifferential::brake_now(self, vexide::smart::motor::BrakeMode::Brake)
    }
}

/// The coupled model attempts all six motors before returning a device error.
/// This robot tolerates those errors in both live control and recorded playback.
pub struct ResilientDrive<M: AdjustableDrive = CoupledDifferential> {
    model: M,
    boost: Rc<Cell<bool>>,
    normal_scale: f64,
    #[cfg(debug_assertions)]
    debug_command: (f64, f64, f64),
    #[cfg(debug_assertions)]
    debug_faults: u64,
    #[cfg(debug_assertions)]
    debug_failed: bool,
}

impl<M: AdjustableDrive> ResilientDrive<M> {
    pub fn new(model: M, boost: Rc<Cell<bool>>, normal_scale: f64) -> Self {
        Self {
            model,
            boost,
            normal_scale,
            #[cfg(debug_assertions)]
            debug_command: (0.0, 0.0, normal_scale),
            #[cfg(debug_assertions)]
            debug_faults: 0,
            #[cfg(debug_assertions)]
            debug_failed: false,
        }
    }

    #[cfg(debug_assertions)]
    fn debug_result(&mut self, result: &Result<(), PortError>) {
        if result.is_err() {
            self.debug_faults += 1;
            if !self.debug_failed {
                println!(
                    "DBG,FAULT,t_ms={},source=drive,count={},error={result:?}",
                    crate::diagnostics::time_ms(),
                    self.debug_faults
                );
            }
        } else if self.debug_failed {
            println!(
                "DBG,RECOVERED,t_ms={},source=drive,count={}",
                crate::diagnostics::time_ms(),
                self.debug_faults
            );
        }
        self.debug_failed = result.is_err();
    }
    fn send_tank(&mut self, left: f64, right: f64, scale: f64) -> Result<(), PortError> {
        self.model.set_output_scale(scale);
        #[cfg(debug_assertions)]
        {
            self.debug_command = (left, right, scale);
        }
        let result = self.model.drive_tank(left, right);
        #[cfg(debug_assertions)]
        self.debug_result(&result);
        result
    }
    fn send_brake(&mut self) -> Result<(), PortError> {
        #[cfg(debug_assertions)]
        {
            self.debug_command = (0.0, 0.0, self.normal_scale);
        }
        let result = AdjustableDrive::brake_now(&mut self.model);
        #[cfg(debug_assertions)]
        self.debug_result(&result);
        result
    }

    /// Attempt every brake; an absent drive motor does not cancel autonomous.
    pub fn brake_now(&mut self) -> Result<(), PortError> {
        self.boost.set(false);
        self.model.set_output_scale(self.normal_scale);
        let _ = self.send_brake();
        Ok(())
    }

    pub fn coast_now(&mut self) -> Result<(), PortError> {
        self.boost.set(false);
        self.model.set_output_scale(self.normal_scale);
        let _ = self.model.stop_now();
        Ok(())
    }
}

impl<M: AdjustableDrive> DrivetrainModel for ResilientDrive<M> {
    type Error = PortError;
}

impl<M: AdjustableDrive> ResilientDrive<M> {
    /// Same driver response, with errors observable by recording validation.
    pub fn live_drive(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let scale = if self.boost.get() {
            1.0
        } else {
            self.normal_scale
        };
        let left = if left.is_finite() { left } else { 0.0 };
        let right = if right.is_finite() { right } else { 0.0 };
        self.send_tank(left, right, scale)
    }

    /// Replay the captured boost setting without consulting today's controller.
    pub fn playback_drive(&mut self, left: f64, right: f64, boost: bool) -> Result<(), PortError> {
        self.boost.set(false);
        let scale = if boost { 1.0 } else { self.normal_scale };
        if left == 0.0 && right == 0.0 {
            self.send_brake()
        } else {
            self.send_tank(left, right, scale)
        }
    }

    /// Autonomous treats any output failure as a terminal route fault.
    pub fn autonomous_drive(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.boost.set(false);
        self.model.set_output_scale(self.normal_scale);
        if left == 0.0 && right == 0.0 {
            self.send_brake()
        } else {
            self.send_tank(left, right, self.normal_scale)
        }
    }

    pub fn autonomous_brake(&mut self) -> Result<(), PortError> {
        self.boost.set(false);
        self.model.set_output_scale(self.normal_scale);
        self.send_brake()
    }
}

impl ResilientDrive {
    #[cfg(debug_assertions)]
    pub fn debug_sample(&self, seq: u64, ms: u128) {
        let wheels = self.wheel_positions().unwrap_or((f64::NAN, f64::NAN));
        let circumference = core::f64::consts::PI * crate::config::DRIVE_WHEEL_DIAMETER_METERS;
        println!(
            "DBG,DRIVE,seq={seq},t_ms={ms},left_cmd={:.4},right_cmd={:.4},scale={:.3},left_rev={:.6},right_rev={:.6},left_m={:.5},right_m={:.5},write_failed={},fault_count={}",
            self.debug_command.0,
            self.debug_command.1,
            self.debug_command.2,
            wheels.0,
            wheels.1,
            wheels.0 * circumference,
            wheels.1 * circumference,
            u8::from(self.debug_failed),
            self.debug_faults
        );
        let (left, right) = self.model.motor_sides();
        for (port, drive) in [
            (5, &left[0]),
            (4, &left[1]),
            (3, &left[2]),
            (20, &right[0]),
            (19, &right[1]),
            (18, &right[2]),
        ] {
            let m = &drive.motor;
            println!(
                "DBG,DRIVE_MOTOR,seq={seq},t_ms={ms},port={port},rpm={:.2},measured_v={:.3},current_a={:.3},temp_c={:.2}",
                m.velocity().unwrap_or(f64::NAN),
                m.voltage().unwrap_or(f64::NAN),
                m.current().unwrap_or(f64::NAN),
                m.temperature().unwrap_or(f64::NAN)
            );
        }
    }

    /// First motor on each side (P5/P20), converted to wheel revolutions.
    /// Distance autonomous requires both encoders; driver control remains tolerant.
    pub fn wheel_positions(&self) -> Option<(f64, f64)> {
        let (left, right) = self.model.motor_sides();
        let revolutions = |motor: &ozton::drivetrain::model::DriveMotor| {
            motor.motor.position().ok().map(|position| {
                position.as_degrees() / 360.0 * motor.wheel_revolutions_per_motor_revolution
            })
        };
        Some((revolutions(left.first()?)?, revolutions(right.first()?)?))
    }
}

impl<M: AdjustableDrive> Tank for ResilientDrive<M> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let _ = self.live_drive(left, right);
        Ok(())
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.coast_now()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct FakeDrive {
        commands: Vec<(f64, f64)>,
        stops: usize,
        scale: f64,
    }
    impl AdjustableDrive for FakeDrive {
        fn set_output_scale(&mut self, scale: f64) {
            self.scale = scale;
        }
    }
    impl DrivetrainModel for FakeDrive {
        type Error = PortError;
    }
    impl Tank for FakeDrive {
        fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
            self.commands.push((left, right));
            Err(PortError::Disconnected { port: 16 })
        }
        fn stop_now(&mut self) -> Result<(), PortError> {
            self.stops += 1;
            Err(PortError::Disconnected { port: 16 })
        }
    }

    #[test]
    fn missing_drive_motor_does_not_abort_playback_or_prevent_retry() {
        let mut drive = ResilientDrive::new(FakeDrive::default(), Rc::new(Cell::new(false)), 0.70);
        assert!(drive.drive_tank(0.4, 0.5).is_ok());
        assert!(drive.drive_tank(0.2, 0.3).is_ok());
        assert_eq!(drive.model.commands, [(0.4, 0.5), (0.2, 0.3)]);
        assert!(drive.stop_now().is_ok());
        assert!(drive.stop_now().is_ok());
        assert_eq!(drive.model.stops, 2);
        assert!(drive.brake_now().is_ok());
        assert_eq!(drive.model.stops, 3);
    }

    #[test]
    fn autonomous_propagates_output_faults_and_stops_clear_boost() {
        let boost = Rc::new(Cell::new(true));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost.clone(), 0.70);
        assert!(drive.autonomous_drive(0.35, 0.35).is_err());
        assert!(!boost.get());
        assert_eq!(drive.model.commands, [(0.35, 0.35)]);
        assert!(drive.autonomous_drive(0.0, 0.0).is_err());
        assert!(drive.autonomous_brake().is_err());
        assert_eq!(drive.model.stops, 2);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn debug_drive_keeps_request_scale_and_failed_write_count() {
        let boost = Rc::new(Cell::new(true));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost, 0.7);
        assert!(drive.live_drive(0.2, -0.3).is_err());
        assert_eq!(drive.debug_command, (0.2, -0.3, 1.0));
        assert_eq!(drive.debug_faults, 1);
        assert!(drive.autonomous_drive(0.35, 0.35).is_err());
        assert_eq!(drive.debug_command, (0.35, 0.35, 0.7));
        assert_eq!(drive.debug_faults, 2);
        drive.brake_now().unwrap();
        assert_eq!(drive.debug_command, (0.0, 0.0, 0.7));
        assert_eq!(drive.debug_faults, 3);
    }

    #[test]
    fn invalid_archived_commands_become_neutral() {
        let mut drive = ResilientDrive::new(FakeDrive::default(), Rc::new(Cell::new(false)), 0.70);
        assert!(drive.drive_tank(f64::NAN, f64::INFINITY).is_ok());
        assert_eq!(drive.model.commands, [(0.0, 0.0)]);
    }

    #[test]
    fn boost_only_lasts_while_held_and_stop_clears_it() {
        let boost = Rc::new(Cell::new(false));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost.clone(), 0.70);
        for held in [false, true, true, false, true] {
            boost.set(held);
            assert!(drive.drive_tank(1.0, -1.0).is_ok());
            assert_eq!(drive.model.scale, if held { 1.0 } else { 0.70 });
        }
        assert!(drive.stop_now().is_ok());
        assert!(!boost.get());
        assert_eq!(drive.model.scale, 0.70);
        assert!(drive.drive_tank(1.0, 1.0).is_ok());
        assert_eq!(drive.model.scale, 0.70);
    }
}
