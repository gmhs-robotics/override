#![deny(dead_code)]
#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

mod auton_commands;
mod auton_selection;
mod autons;
mod config;
mod diagnostics;
mod forward_auton;
mod low_score;
mod mechanism_control;
mod mechanisms;
mod pivot_control;
mod recording;
mod recovery;
mod resilient_drive;

use std::{cell::Cell, rc::Rc, time::Instant};

#[cfg(debug_assertions)]
use std::time::Duration;

use mechanisms::{MechanismFrame, Mechanisms, PivotSelection};
use ozton::{
    control::{drive::AntiTip, loops::Pid},
    drivetrain::model::{CoupledDifferential, TipAngleAxis},
    record::{DifferentialVoltageFrame, frame::RecordMode},
};
use resilient_drive::ResilientDrive;
use vexide::{
    color::Color,
    competition::{self, CompetitionMode},
    display::{Display, Font, FontFamily, FontSize, Rect, Text},
    prelude::*,
    time::sleep,
};

use config::*;

fn anti_tip_controller() -> AntiTip {
    let (kp, ki, kd) = TIP_PID;
    AntiTip::new(
        Pid::new(kp, ki, kd, None),
        TIP_THRESHOLD_RADIANS,
        TIP_POLARITY,
        TIP_MAXIMUM_CORRECTION,
    )
}
struct Robot {
    controller: Controller,
    drive_boost: Rc<Cell<bool>>,
    drivetrain: ResilientDrive,
    mechanisms: Mechanisms,
    menu: auton_selection::RouteMenu,
    auton: auton_commands::AutonControl,
    #[cfg(debug_assertions)]
    debug_log: diagnostics::DebugLog,
    #[cfg(debug_assertions)]
    debug_imu: Rc<InertialSensor>,
    #[cfg(debug_assertions)]
    debug_level: Option<vexide::math::Angle>,
}

struct DriverCommand {
    boost: bool,
    drivetrain: DifferentialVoltageFrame,
    mechanisms: MechanismFrame,
}

impl Robot {
    fn stop(&mut self) {
        let _ = self.drivetrain.brake_now();
        self.mechanisms.stop();
        self.auton.clear_targets();
    }

    #[cfg(debug_assertions)]
    fn debug_tick(
        &mut self,
        elapsed: Duration,
        route_active: bool,
        recording: bool,
        pending_save: bool,
        playback: Option<&recording::Playback>,
        capture: Option<&recording::Capture>,
    ) {
        let mode = competition::mode();
        if let Some((seq, t_ms)) = self.debug_log.sample(elapsed) {
            let imu = self.debug_imu.euler().ok();
            let tilt = imu
                .zip(self.debug_level)
                .map(|(a, level)| (a.c - level).wrapped_half().as_degrees())
                .unwrap_or(f64::NAN);
            println!(
                "DBG,STATE,seq={seq},t_ms={t_ms},mode={mode:?},selection={:?},route_active={},recording={},pending_save={},controller_ok={},imu_ok={},roll_relative_deg={:.3}",
                self.menu.action(),
                u8::from(route_active),
                u8::from(recording),
                u8::from(pending_save),
                u8::from(self.controller.state().is_ok()),
                u8::from(imu.is_some() && self.debug_level.is_some()),
                tilt
            );
            if route_active {
                if let Some(route) = playback {
                    route.debug_sample(seq, t_ms);
                } else {
                    self.auton
                        .debug_sample(seq, t_ms, self.drivetrain.wheel_positions());
                }
            }
            if let Ok(input) = self.controller.state() {
                println!(
                    "DBG,INPUT,seq={seq},t_ms={t_ms},throttle={:.4},turn={:.4},r1={},r2={},l1={},l2={},x={},a={},b={},up={},down={},boost={}",
                    input.left_stick.y(),
                    input.right_stick.x(),
                    u8::from(input.button_r1.is_pressed()),
                    u8::from(input.button_r2.is_pressed()),
                    u8::from(input.button_l1.is_pressed()),
                    u8::from(input.button_l2.is_pressed()),
                    u8::from(input.button_x.is_pressed()),
                    u8::from(input.button_a.is_pressed()),
                    u8::from(input.button_b.is_pressed()),
                    u8::from(input.button_up.is_pressed()),
                    u8::from(input.button_down.is_pressed()),
                    u8::from(input.button_y.is_pressed())
                );
            }
            self.drivetrain.debug_sample(seq, t_ms);
            self.mechanisms.debug_sample(seq, t_ms);
            if let Some(active) = capture {
                active.debug_sample(seq, t_ms);
            }
        }
    }

    fn driver_command(&self) -> Option<DriverCommand> {
        // Missing controller cancels automatic mechanism cycles and retries next tick.
        let state = self.controller.state().ok()?;
        let boost = state.button_y.is_pressed();
        self.drive_boost.set(boost);
        let direction = match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
            (true, false) => 1,
            (false, true) => -1,
            _ => 0,
        };
        Some(DriverCommand {
            boost,
            drivetrain: DifferentialVoltageFrame::arcade(
                drive_axis(state.left_stick.y()),
                drive_axis(state.right_stick.x()),
            ),
            mechanisms: MechanismFrame {
                roller_override_direction: None,
                lift_direction: direction,
                pivot_raise: state.button_x.is_pressed(),
                pivot_lower: state.button_a.is_pressed() || state.button_up.is_pressed(),
                pivot_follow: state.button_down.is_pressed(),
                pivot_selection: PivotSelection::Follow,
                low_score: state.button_b.is_pressed(),
                low_score_cancel: state.button_r1.is_pressed()
                    || state.button_r2.is_pressed()
                    || state.button_l1.is_pressed()
                    || state.button_l2.is_pressed(),
                pivot_override_degrees: None,
                intake_direction: match (state.button_l1.is_pressed(), state.button_l2.is_pressed())
                {
                    (true, false) => 1,
                    (false, true) => -1,
                    _ => 0,
                },
                lift_position_degrees: 0.0,
            },
        })
    }
}

fn drive_axis(value: f64) -> f64 {
    if value.is_finite() && value.abs() > DRIVE_INPUT_DEADBAND {
        value
    } else {
        0.0
    }
}

fn show_status(display: &mut Display, status: &str) {
    display.fill(
        &Rect::new(
            [0, 0],
            [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
        ),
        Color::BLACK,
    );
    display.draw_text(
        &Text::from_string(
            status,
            Font::new(FontSize::MEDIUM, FontFamily::Proportional),
            [16, 70],
        ),
        Color::WHITE,
        None,
    );
    diagnostics::event!("status={}", status);
}

enum ActiveRoute {
    Builtin,
    Recorded(recording::Playback),
}

fn save_pending(
    store: &mut recording::RouteStore,
    pending: &mut Option<recording::PendingRecording>,
    menu: &mut auton_selection::RouteMenu,
) {
    let Some(recording) = pending.as_mut() else {
        return;
    };
    match store.save(recording) {
        Ok(id) => {
            *pending = None;
            menu.refresh(&store.index, false, auton_selection::Action::Playback(id));
            menu.status("Route saved");
        }
        Err(error) => {
            menu.refresh(&store.index, true, auton_selection::Action::RetrySave);
            menu.status(error);
        }
    }
}

async fn control_loop(mut robot: Robot, mut store: recording::RouteStore) -> ! {
    use auton_selection::Action;
    use recording::{Capture, RecordTarget, RobotFrame};
    let mut previous_mode = None;
    let mut autonomous: Option<ActiveRoute> = None;
    let mut capture: Option<Capture> = None;
    let mut pending = None;
    let mut last_tick = Instant::now();
    loop {
        let now = Instant::now();
        let mut elapsed = now.saturating_duration_since(last_tick);
        last_tick = now;
        let mode = competition::mode();
        if robot.menu.poll(mode == CompetitionMode::Disabled) == Some(Action::RetrySave) {
            robot.stop();
            save_pending(&mut store, &mut pending, &mut robot.menu);
        }
        if previous_mode != Some(mode) {
            diagnostics::event!(
                "mode={:?},previous={:?},selection={:?}",
                mode,
                previous_mode,
                robot.menu.action()
            );
            robot.stop();
            if let Some(finished) = capture.take() {
                pending = finished.finish();
            }
            previous_mode = Some(mode);
            autonomous = None;
            match mode {
                CompetitionMode::Disabled => {
                    if pending.is_some() {
                        save_pending(&mut store, &mut pending, &mut robot.menu);
                    } else {
                        robot.menu.status("Ready - select a route");
                    }
                }
                CompetitionMode::Driver => {
                    let target = match robot.menu.action() {
                        Action::RecordNew => Some(RecordTarget::New),
                        Action::Replace(id) => Some(RecordTarget::Replace(id)),
                        _ => None,
                    };
                    if let Some(target) = target {
                        if pending.is_none() {
                            capture = Some(Capture::new(target));
                            robot.menu.status("Recording - disable to save");
                        } else {
                            robot.menu.status("Save pending recording first");
                        }
                    } else {
                        robot.menu.status("Driver control");
                    }
                }
                CompetitionMode::Autonomous => match robot.menu.action() {
                    Action::Score => {
                        autonomous = Some(ActiveRoute::Builtin);
                        robot.menu.status("Autonomous running");
                    }
                    Action::Playback(id) => match store.load(id) {
                        Ok(playback)
                            if robot
                                .mechanisms
                                .pose()
                                .is_some_and(|(h, p)| playback.start_pose_matches(h, p)) =>
                        {
                            autonomous = Some(ActiveRoute::Recorded(playback));
                            robot.menu.status("Playing recorded route");
                        }
                        Ok(_) => robot.menu.status("Set lift/pivot to route start"),
                        Err(error) => robot.menu.status(error),
                    },
                    _ => robot.menu.status("Autonomous disabled"),
                },
            }
            // SD loading/saving occurs only after a stop; don't count it as a late motion tick.
            robot.stop();
            last_tick = Instant::now();
            elapsed = CONTROL_INTERVAL;
        }
        // Loading or saving may span a mode change. Recheck before any output.
        if competition::mode() != mode {
            robot.stop();
            continue;
        }
        match mode {
            CompetitionMode::Disabled => robot.stop(),
            CompetitionMode::Driver => {
                if let Some(command) = robot.driver_command() {
                    let drive_ok = robot
                        .drivetrain
                        .live_drive(command.drivetrain.left, command.drivetrain.right)
                        .is_ok();
                    let mechanisms_ok = robot
                        .mechanisms
                        .apply(&command.mechanisms, RecordMode::Live);
                    if capture.is_some() {
                        if let Some(mechanisms) =
                            robot.mechanisms.recording_snapshot(&command.mechanisms)
                            && drive_ok
                            && mechanisms_ok
                        {
                            let frame = RobotFrame {
                                version: RECORDING_FORMAT_VERSION,
                                drivetrain: command.drivetrain,
                                mechanisms,
                                boost: command.boost,
                            };
                            if let Some(active) = capture.as_mut() {
                                match active.push(frame, elapsed) {
                                    Ok(true) => {
                                        pending = capture.take().and_then(Capture::finish);
                                        robot.menu.status("Recording ready - disable to save");
                                    }
                                    Ok(false) => {}
                                    Err(reason) => {
                                        capture = None;
                                        robot.menu.status(reason);
                                    }
                                }
                            }
                        } else {
                            capture = None;
                            robot.menu.status("Recording discarded: device fault");
                        }
                    }
                } else {
                    robot.stop();
                    if capture.take().is_some() {
                        robot.menu.status("Recording discarded: controller");
                    }
                }
            }
            CompetitionMode::Autonomous => {
                let result = match autonomous.as_mut() {
                    Some(ActiveRoute::Builtin) => {
                        let result = autons::run(&mut robot).await;
                        // The command function owns its control ticks while awaited.
                        last_tick = Instant::now();
                        #[cfg(debug_assertions)]
                        if let Err(error) = result {
                            diagnostics::event!("auton_error={error:?}");
                        }
                        result
                            .map(|()| true)
                            .map_err(auton_commands::AutonError::status)
                    }
                    Some(ActiveRoute::Recorded(playback)) => match playback.update(elapsed) {
                        Ok(Some(frame)) => {
                            let drive_ok = robot
                                .drivetrain
                                .playback_drive(
                                    frame.drivetrain.left,
                                    frame.drivetrain.right,
                                    frame.boost,
                                )
                                .is_ok();
                            let mechanisms_ok = robot
                                .mechanisms
                                .apply(&frame.mechanisms, RecordMode::Playback);
                            if drive_ok && mechanisms_ok {
                                Ok(false)
                            } else {
                                Err("Playback stopped: device fault")
                            }
                        }
                        Ok(None) => Ok(true),
                        Err(error) => Err(error),
                    },
                    None => {
                        robot.stop();
                        Ok(false)
                    }
                };
                match result {
                    Ok(true) => {
                        robot.stop();
                        autonomous = None;
                        robot.menu.status("Autonomous complete");
                    }
                    Err(reason) => {
                        robot.stop();
                        autonomous = None;
                        robot.menu.status(reason);
                    }
                    Ok(false) => {}
                }
            }
        }
        #[cfg(debug_assertions)]
        robot.debug_tick(
            elapsed,
            autonomous.is_some(),
            capture.is_some(),
            pending.is_some(),
            autonomous.as_ref().and_then(|route| match route {
                ActiveRoute::Recorded(playback) => Some(playback),
                _ => None,
            }),
            capture.as_ref(),
        );
        sleep(CONTROL_INTERVAL).await;
    }
}

#[cfg_attr(debug_assertions, vexide::main)]
#[cfg_attr(not(debug_assertions), vexide::main(banner(enabled = false)))]
async fn main(peripherals: Peripherals) {
    diagnostics::event!(
        "boot,log_schema=1,build=debug,period_ms={},wheel_dia_m={:.6},drive_scale={:.3},wheel_per_motor_11w={:.6},wheel_per_motor_5w={:.6},pivot_sensor_per_output={:.3},lift_max_deg={:.3},pivot_normal_deg={:.3},forward_m={:.4},retreat_m={:.4}",
        DEBUG_SAMPLE_INTERVAL.as_millis(),
        DRIVE_WHEEL_DIAMETER_METERS,
        DRIVE_OUTPUT_SCALE,
        ELEVEN_W_WHEEL_PER_MOTOR_REV,
        FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
        PIVOT_SENSOR_PER_OUTPUT_REV,
        MECHANISM_CALIBRATION.lift_upper_limit(),
        PIVOT_NORMAL_OUTPUT_DEGREES,
        AUTON_FORWARD_METERS,
        AUTON_RETREAT_METERS
    );
    if !config::valid() {
        let mut display = peripherals.display;
        show_status(&mut display, "STOP: invalid robot configuration");
        loop {
            sleep(CONTROL_INTERVAL).await;
        }
    }
    let (controller, mut display, mut imu, left, right, mechanisms) = config::hardware(peripherals);
    show_status(&mut display, "Starting: keep robot level and intake stowed");

    diagnostics::event!(
        "config,pivot_kp={:.5},pivot_ki={:.5},pivot_kd={:.5},pivot_max_v={:.3},pivot_ratio={:.5},drive_kp={:.4},drive_ki={:.4},drive_kd={:.4},tip_kp={:.4},tip_kd={:.4},tip_threshold_rad={:.4},lift_position_gain={:.5},auton_lower_deg={:.3}",
        POSITION_GAIN,
        INTEGRAL_GAIN,
        VELOCITY_DAMPING,
        MAXIMUM_VOLTS,
        PIVOT_SENSOR_DEGREES_PER_LIFT_DEGREE,
        AUTON_DRIVE_PID.0,
        AUTON_DRIVE_PID.1,
        AUTON_DRIVE_PID.2,
        TIP_PID.0,
        TIP_PID.2,
        TIP_THRESHOLD_RADIANS,
        LIFT_POSITION_GAIN,
        AUTON_LOWER_DEGREES
    );
    diagnostics::event!(
        "scoring_config,b_low_height_deg={:.3},b_extra_output_deg={:.3},auton_lower_deg={:.3},auton_lift_arrival_deg={:.3},auton_pivot_arrival_output_deg={:.3}",
        LOW_HEIGHT_ABOVE_BOTTOM_DEGREES,
        B_PIVOT_EXTRA_OUTPUT_DEGREES,
        AUTON_LOWER_DEGREES,
        AUTON_LIFT_ARRIVAL_TOLERANCE,
        AUTON_PIVOT_ARRIVAL_OUTPUT_DEGREES
    );
    // Keep the robot level and stationary while calibrating and capturing level.
    let imu_ready = imu.calibrate().await.is_ok();
    diagnostics::event!("imu_calibration_ok={}", u8::from(imu_ready));
    if !imu_ready {
        diagnostics::event!("P7 IMU calibration failed; anti-tip unavailable");
    }
    let imu = Rc::new(imu);

    #[cfg(debug_assertions)]
    let debug_level = if imu_ready {
        imu.euler().ok().map(|a| a.c)
    } else {
        None
    };
    let mut model =
        CoupledDifferential::new_direct_voltage(left, right).with_output_scale(DRIVE_OUTPUT_SCALE);
    if imu_ready {
        match imu.euler() {
            Ok(angles) => {
                // Install below the shared drive wrapper so driver control,
                // driver control and autonomous both use live tilt.
                #[cfg(debug_assertions)]
                let drive_imu = imu.clone();
                #[cfg(not(debug_assertions))]
                let drive_imu = imu;
                model = model.with_anti_tip_axis(
                    drive_imu,
                    anti_tip_controller(),
                    TipAngleAxis::Roll,
                    angles.c,
                );
            }
            Err(_error) => {
                diagnostics::event!("P7 IMU roll unavailable; anti-tip disabled: {_error:?}")
            }
        }
    }
    let drive_boost = Rc::new(Cell::new(false));
    let (store, load_error) = match recording::RouteStore::open("") {
        Ok(store) => (store, None),
        Err(error) => (recording::RouteStore::empty(""), Some(error)),
    };
    let mut menu = auton_selection::RouteMenu::new(display, &store.index, AUTON_ENABLED_DEFAULT);
    if let Some(error) = load_error {
        menu.status(error);
    }
    let robot = Robot {
        controller,
        drive_boost: drive_boost.clone(),
        drivetrain: ResilientDrive::new(model, drive_boost, DRIVE_OUTPUT_SCALE),
        mechanisms,
        menu,
        auton: auton_commands::AutonControl::default(),
        #[cfg(debug_assertions)]
        debug_log: diagnostics::DebugLog::new(),
        #[cfg(debug_assertions)]
        debug_imu: imu,
        #[cfg(debug_assertions)]
        debug_level,
    };

    control_loop(robot, store).await;
}

#[cfg(test)]
mod anti_tip_tests {
    use super::*;

    #[test]
    fn upside_down_x_mount_preserves_roll_correction_across_angle_wrap() {
        use vexide::math::Angle;
        // A 180-degree X rotation adds a half-turn to level roll without
        // reversing the roll axis. Exercise both sides of the +/-180 boundary.
        for level_degrees in [179.0, -179.0] {
            let level = Angle::from_degrees(level_degrees);
            for tilt_degrees in [-20.0, 20.0] {
                let reading = (level + Angle::from_degrees(tilt_degrees)).wrapped_half();
                let tilt = (reading - level).wrapped_half().as_radians();
                assert!((tilt - Angle::from_degrees(tilt_degrees).as_radians()).abs() < 1e-9);
                let mut controller = anti_tip_controller();
                assert!(controller.correction(tilt, CONTROL_INTERVAL) * tilt_degrees > 0.0);
            }
        }
    }

    #[test]
    fn level_and_small_tilt_do_not_drive_but_both_tilt_directions_recover() {
        let mut controller = anti_tip_controller();
        for tilt in [0.0, 0.10, -0.10] {
            assert_eq!(controller.correction(tilt, CONTROL_INTERVAL), 0.0);
        }
        for direction in [-1.0, 1.0] {
            let mut controller = anti_tip_controller();
            let command = controller.correction(direction * 0.3, CONTROL_INTERVAL);
            assert!(command * direction > 0.0);
            assert!(command.abs() <= TIP_MAXIMUM_CORRECTION);
            assert_eq!(controller.correction(0.0, CONTROL_INTERVAL), 0.0);
        }
    }
}
