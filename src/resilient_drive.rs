//! Competition policy: keep every available drive motor running and retry missing motors.

use ozton::drivetrain::model::{CoupledDifferential, DrivetrainModel, Tank};
use std::{cell::Cell, rc::Rc};
use vexide::smart::PortError;

pub trait AdjustableDrive: Tank<Error = PortError> {
    fn set_output_scale(&mut self, scale: f64);
}

impl AdjustableDrive for CoupledDifferential {
    fn set_output_scale(&mut self, scale: f64) {
        CoupledDifferential::set_output_scale(self, scale);
    }
}

/// The coupled model attempts all six motors before returning a device error.
/// This robot tolerates those errors in both live control and recorded playback.
pub struct ResilientDrive<M: AdjustableDrive = CoupledDifferential> {
    model: M,
    boost: Rc<Cell<bool>>,
    normal_scale: f64,
}

impl<M: AdjustableDrive> ResilientDrive<M> {
    pub fn new(model: M, boost: Rc<Cell<bool>>, normal_scale: f64) -> Self {
        Self {
            model,
            boost,
            normal_scale,
        }
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

impl<M: AdjustableDrive> Tank for ResilientDrive<M> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.model.set_output_scale(if self.boost.get() {
            1.0
        } else {
            self.normal_scale
        });
        // Invalid archived commands become neutral rather than reaching hardware.
        let left = if left.is_finite() { left } else { 0.0 };
        let right = if right.is_finite() { right } else { 0.0 };
        let _ = self.model.drive_tank(left, right);
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
