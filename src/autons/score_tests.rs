use super::*;
use crate::auton_commands::AutonError;
use std::{
    future::Future,
    task::{Context, Poll, Waker},
    time::Duration,
};

#[derive(Default)]
struct TestRobot {
    lift: f64,
    pivot: Option<f64>,
    eject: bool,
    waits: usize,
    drove_forward: bool,
    lowered: bool,
    released: bool,
    raised_while_ejecting: bool,
    retreated: bool,
    fail_alignment: bool,
}
impl Commands for TestRobot {
    fn set_lift_height(&mut self, height: f64) -> AutonResult {
        if self.drove_forward && height < self.lift {
            self.lowered = true;
        }
        if self.eject && height > self.lift {
            self.raised_while_ejecting = true;
        }
        self.lift = height;
        Ok(())
    }
    fn follow_pivot(&mut self) {
        self.pivot = None;
    }
    fn set_pivot_angle(&mut self, degrees: f64) -> AutonResult {
        self.pivot = Some(degrees);
        Ok(())
    }
    fn start_eject(&mut self) {
        self.eject = true;
    }
    fn stop_eject(&mut self) {
        self.eject = false;
    }
    async fn drive_forward(&mut self, meters: f64) -> AutonResult {
        assert_eq!(meters, AUTON_FORWARD_METERS);
        assert!(self.lift > 0.0);
        assert_eq!(self.pivot, None);
        assert!(!self.eject);
        self.drove_forward = true;
        Ok(())
    }
    async fn drive_backward(&mut self, meters: f64) -> AutonResult {
        assert_eq!(meters, AUTON_RETREAT_METERS);
        assert!(self.released && self.raised_while_ejecting);
        assert!(!self.eject);
        self.retreated = true;
        Ok(())
    }
    async fn wait_for_mechanisms(&mut self) -> AutonResult {
        self.waits += 1;
        if self.fail_alignment {
            return Err(AutonError::MechanismFault);
        }
        assert_eq!(self.pivot, Some(PIVOT_NORMAL_OUTPUT_DEGREES));
        Ok(())
    }
    async fn wait(&mut self, duration: Duration) -> AutonResult {
        assert!(self.lowered && self.eject);
        if duration == AUTON_RELEASE_TIME {
            assert!(!self.raised_while_ejecting);
            self.released = true;
        } else {
            assert_eq!(duration, EJECT_TIME);
            assert!(self.raised_while_ejecting);
        }
        Ok(())
    }
}
fn complete(future: impl Future<Output = AutonResult>) -> AutonResult {
    match std::pin::pin!(future)
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(result) => result,
        Poll::Pending => panic!("test commands must be immediately ready"),
    }
}
#[test]
fn route_raises_during_forward_travel_lowers_before_release_and_lifts_while_ejecting() {
    let mut robot = TestRobot::default();
    complete(run(&mut robot)).unwrap();
    assert!(robot.retreated);
    assert_eq!(robot.waits, 3);
    assert!(!robot.eject);
}
#[test]
fn failed_command_prevents_release_and_retreat() {
    let mut robot = TestRobot {
        fail_alignment: true,
        ..Default::default()
    };
    assert_eq!(complete(run(&mut robot)), Err(AutonError::MechanismFault));
    assert!(robot.drove_forward);
    assert!(!robot.lowered && !robot.eject && !robot.retreated);
}
