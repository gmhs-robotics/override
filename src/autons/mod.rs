//! Premade routes are ordinary async functions. Recorded routes use recording.rs.
mod score;

pub(crate) async fn run(robot: &mut crate::Robot) -> crate::auton_commands::AutonResult {
    robot.begin_autonomous();
    let result = score::run(robot).await;
    robot.stop();
    result
}
