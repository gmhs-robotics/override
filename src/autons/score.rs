//! Raise while approaching, lower to score, eject while lifting, then retreat.
use crate::{
    auton_commands::{AutonResult, Commands},
    config::{
        AUTON_FORWARD_METERS, AUTON_LOWER_DEGREES, AUTON_RAISE_FRACTION, AUTON_RELEASE_TIME,
        AUTON_RETREAT_METERS, EJECT_TIME, MECHANISM_CALIBRATION, PIVOT_NORMAL_OUTPUT_DEGREES,
    },
};

pub(super) async fn run(robot: &mut impl Commands) -> AutonResult {
    let lift = MECHANISM_CALIBRATION;
    let raised = lift.lift_bottom_degrees
        + (lift.lift_upper_limit() - lift.lift_bottom_degrees) * AUTON_RAISE_FRACTION;
    let lowered = (raised - AUTON_LOWER_DEGREES).max(lift.lift_bottom_degrees);

    robot.set_lift_height(raised)?;
    robot.follow_pivot();
    robot.drive_forward(AUTON_FORWARD_METERS).await?;

    robot.set_pivot_angle(PIVOT_NORMAL_OUTPUT_DEGREES)?;
    robot.wait_for_mechanisms().await?;
    robot.set_lift_height(lowered)?;
    robot.wait_for_mechanisms().await?;

    robot.start_eject();
    robot.wait(AUTON_RELEASE_TIME).await?;
    robot.set_lift_height(raised)?;
    robot.wait(EJECT_TIME).await?;
    robot.wait_for_mechanisms().await?;

    robot.stop_eject();
    robot.drive_backward(AUTON_RETREAT_METERS).await
}

#[cfg(test)]
#[path = "score_tests.rs"]
mod tests;
