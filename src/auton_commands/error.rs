//! Allocation-free errors for premade autonomous commands.
use core::{error::Error, fmt};
use vexide::smart::PortError;

/// Identifies a command failure without requiring callers to inspect log text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AutonError {
    Cancelled,
    RouteTimeout,
    CommandTimeout,
    ControlDelay,
    InvalidLiftTarget,
    InvalidPivotTarget,
    InvalidDistance,
    NegativeForwardDistance,
    NegativeBackwardDistance,
    EncoderUnavailable,
    InvalidWheelFeedback,
    MechanismFault,
    MechanismFeedbackUnavailable,
    DriveOutput(PortError),
    DriveBrake(PortError),
}

impl AutonError {
    /// Stable, allocation-free text for the Brain's existing status screen.
    pub(crate) const fn status(self) -> &'static str {
        match self {
            Self::Cancelled => "Autonomous cancelled",
            Self::RouteTimeout => "STOP: autonomous total timeout",
            Self::CommandTimeout => "STOP: autonomous command timeout",
            Self::ControlDelay => "STOP: autonomous control delay",
            Self::InvalidLiftTarget => "STOP: invalid lift target",
            Self::InvalidPivotTarget => "STOP: invalid pivot target",
            Self::InvalidDistance => "STOP: invalid drive distance",
            Self::NegativeForwardDistance => "STOP: forward distance must be positive",
            Self::NegativeBackwardDistance => "STOP: backward distance must be positive",
            Self::EncoderUnavailable => "STOP: P5/P20 encoder missing",
            Self::InvalidWheelFeedback => "STOP: invalid wheel motion feedback",
            Self::MechanismFault => "STOP: autonomous mechanism fault",
            Self::MechanismFeedbackUnavailable => "STOP: mechanism feedback missing",
            Self::DriveOutput(_) => "STOP: autonomous drive output fault",
            Self::DriveBrake(_) => "STOP: drive brake fault",
        }
    }
}

impl fmt::Display for AutonError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.status())?;
        if let Self::DriveOutput(source) | Self::DriveBrake(source) = self {
            write!(formatter, ": {source}")?;
        }
        Ok(())
    }
}

impl Error for AutonError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::DriveOutput(source) | Self::DriveBrake(source) => Some(source),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hardware_error_retains_source_while_ui_status_stays_stable() {
        let error = AutonError::DriveOutput(PortError::Disconnected { port: 5 });
        assert_eq!(error.status(), "STOP: autonomous drive output fault");
        assert!(error.source().is_some());
        assert!(error.to_string().contains("port 5"));
        assert!(AutonError::Cancelled.source().is_none());
    }
}
