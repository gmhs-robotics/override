#![deny(dead_code)]
#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

mod mechanism_control;
mod mechanisms;
mod pivot_control;
mod recovery;
mod resilient_drive;

use std::{
    cell::Cell,
    rc::Rc,
    time::{Duration, Instant},
};

use mechanism_control::Calibration;
use mechanisms::{MechanismFrame, Mechanisms};
use ozton::{
    derive::RecordedRobot,
    drivetrain::{
        Drivetrain,
        model::{CoupledDifferential, DriveMotor, Tank},
    },
    record::{
        self, DifferentialVoltageFrame, RecordableDrivetrain,
        frame::PlaybackOutcome,
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
    tracking::NoTracking,
};
use resilient_drive::ResilientDrive;
use vexide::{
    color::Color,
    display::{Display, Font, FontFamily, FontSize, Rect, Text, TouchState},
    prelude::*,
    time::sleep,
};

// Competition robot configuration.
const CONTROL_INTERVAL: Duration = Duration::from_millis(10);
const ELEVEN_W_WHEEL_PER_MOTOR_REV: f64 = 36.0 / 48.0;
const FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV: f64 = 1.0;
const RECORD_DRIVE_LIMIT: f64 = 0.45;
const DRIVE_OUTPUT_SCALE: f64 = 0.70;
const FIRMWARE_LABEL: &str = "OVERRIDE P9 PIVOT";
// Pivot follows twice the original 145/135 shaft-to-lift ratio.
// It reaches 90 output degrees at about -167.59 lift encoder degrees.
// Start with the lift at zero and the intake stowed so startup references match the mechanism.
// Pivot feedback uses the rotation sensor on the motor shaft, before external gearing.
const MECHANISM_CALIBRATION: Calibration = Calibration {
    lift_bottom_degrees: 0.0,
    lift_top_degrees: 1800.0,
    lift_rotated_degrees: 135.0 * 360.0 / (145.0 * 2.0),
    pivot_home_degrees: 0.0,
    // One tooth on the 12T shaft: 30 shaft degrees / 4 = 7.5 intake degrees.
    pivot_alignment_output_degrees: (360.0 / 12.0) / (48.0 / 12.0),
    pivot_travel_output_degrees: 90.0,
    pivot_motor_per_output_rev: 48.0 / 12.0, // 12T motor gear drives the 48T intake gear
};
const BOOT_SELECT_TIMEOUT: Duration = Duration::from_secs(4);
const OFF_WALL_DRIVE_POWER: f64 = 0.35;
const OFF_WALL_DRIVE_TIME: Duration = Duration::from_millis(750);

type RobotDrive = RecordableDrivetrain<ResilientDrive, NoTracking>;

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    controller: Controller,
    #[record(skip)]
    record_mode: bool,
    #[record(skip)]
    drive_boost: Rc<Cell<bool>>,
    drivetrain: RobotDrive,
    mechanisms: Mechanisms,
}

#[record::async_trait(?Send)]
impl record::Recordable for Robot {
    const UPDATE_INTERVAL: Duration = CONTROL_INTERVAL;
    const PREDETERMINED_ROUTE_NAME: Option<&'static str> = Some("Move Off Wall");

    async fn run_predetermined_route(&mut self) -> PlaybackOutcome {
        // Reissue the current command so motors that reconnect during the route join in.
        let start = Instant::now();
        while start.elapsed() < OFF_WALL_DRIVE_TIME {
            let _ = self
                .drivetrain
                .model
                .drive_tank(OFF_WALL_DRIVE_POWER, OFF_WALL_DRIVE_POWER);
            sleep(CONTROL_INTERVAL).await;
        }
        let _ = self.drivetrain.model.coast_now();
        PlaybackOutcome::Completed
    }

    async fn get_new_frame(&self) -> Self::Frame {
        // A controller read failure produces neutral commands; retry on the next control tick.
        let state = self.controller.state().unwrap_or_default();
        let boost = state.button_y.is_pressed();
        self.drive_boost.set(boost);
        let input_limit = if self.record_mode && !boost {
            RECORD_DRIVE_LIMIT
        } else {
            1.0
        };
        let direction = match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
            (true, false) => 1,
            (false, true) => -1,
            _ => 0,
        };
        Self::Frame {
            drivetrain: DifferentialVoltageFrame::arcade(
                state.left_stick.y() * input_limit,
                state.right_stick.x() * input_limit,
            ),
            mechanisms: MechanismFrame {
                lift_direction: direction,
                pivot_raise: state.button_up.is_pressed() || state.button_x.is_pressed(),
                pivot_lower: state.button_down.is_pressed() || state.button_a.is_pressed(),
                pivot_trim_output_degrees: 0.0,
                intake_direction: match (state.button_l1.is_pressed(), state.button_l2.is_pressed())
                {
                    (true, false) => 1,
                    (false, true) => -1,
                    _ => 0,
                },
                lift_position_degrees: 0.0,
            },
        }
    }

    async fn on_playback_abort(&mut self) {
        let _ = self.drivetrain.model.coast_now();
        self.mechanisms.hold();
    }

    async fn on_save(&mut self) {
        // Saving already succeeded. Missing rumble feedback does not invalidate the saved route.
        let _ = self.controller.rumble(".").await;
    }
}

async fn choose_record_mode(display: &mut Display) -> bool {
    // Capture presses made while the menu is being drawn, including quick taps.
    let mut last_press_count = display.touch_status().press_count;
    display.fill(
        &Rect::new(
            [0, 0],
            [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
        ),
        Color::BLACK,
    );
    let font = Font::new(FontSize::MEDIUM, FontFamily::Proportional);
    display.draw_text(
        &Text::from_string(FIRMWARE_LABEL, font, [16, 20]),
        Color::WHITE,
        None,
    );
    display.draw_text(
        &Text::from_string("Touch upper half: RECORD", font, [16, 52]),
        Color::WHITE,
        None,
    );
    display.draw_text(
        &Text::from_string("Touch lower half: PLAYBACK", font, [16, 164]),
        Color::WHITE,
        None,
    );
    let start = Instant::now();
    let record_mode = loop {
        let touch = display.touch_status();
        let new_press = touch.press_count != last_press_count;
        last_press_count = touch.press_count;
        if (new_press || touch.state == TouchState::Held)
            && (0..Display::HORIZONTAL_RESOLUTION).contains(&touch.point.x)
            && (0..Display::VERTICAL_RESOLUTION).contains(&touch.point.y)
        {
            break touch.point.y < Display::VERTICAL_RESOLUTION / 2;
        }
        if start.elapsed() >= BOOT_SELECT_TIMEOUT {
            break false;
        }
        sleep(CONTROL_INTERVAL).await;
    };
    display.fill(
        &Rect::new(
            [0, 0],
            [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
        ),
        Color::BLACK,
    );
    let selected = if record_mode {
        "RECORD selected"
    } else {
        "PLAYBACK selected"
    };
    display.draw_text(
        &Text::from_string(selected, font, [16, 108]),
        Color::WHITE,
        None,
    );
    record_mode
}

// Match vexide's default code signature and startup, without the entrypoint
// macro's generated `Peripherals::take().unwrap()`.
#[cfg_attr(target_os = "vexos", unsafe(link_section = ".code_signature"))]
#[used]
#[unsafe(no_mangle)]
static __VEXIDE_CODE_SIGNATURE: vexide::program::CodeSignature =
    vexide::program::CodeSignature::new(
        vexide::program::ProgramType::User,
        vexide::program::ProgramOwner::Partner,
        vexide::program::ProgramOptions::empty(),
    );

fn main() {
    // SAFETY: Called once at the program entrypoint, before using VEX devices.
    unsafe { vexide::startup::startup() };
    vexide::startup::banner::print(vexide::startup::banner::themes::THEME_DEFAULT);
    vexide::runtime::block_on(async {
        loop {
            if let Some(peripherals) = Peripherals::take() {
                run(peripherals).await;
            }
            sleep(CONTROL_INTERVAL).await;
        }
    });
}

async fn run(peripherals: Peripherals) {
    println!(
        "{FIRMWARE_LABEL}: drive L5/4/3 R20/19/18; lift P1; roller P2; intake P10; pivot P9; rotation P8"
    );
    let mut display = peripherals.display;
    let record_mode = choose_record_mode(&mut display).await;

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
    let model =
        CoupledDifferential::new_direct_voltage(left, right).with_output_scale(DRIVE_OUTPUT_SCALE);
    let drive_boost = Rc::new(Cell::new(false));
    let robot = Robot {
        controller: peripherals.primary_controller,
        record_mode,
        drive_boost: drive_boost.clone(),
        drivetrain: RecordableDrivetrain::new(Drivetrain::new(
            ResilientDrive::new(model, drive_boost, DRIVE_OUTPUT_SCALE),
            NoTracking,
        )),
        mechanisms: Mechanisms::new(
            Motor::new(peripherals.port_1, Gearset::Red, Direction::Forward),
            Motor::new_exp(peripherals.port_9, Direction::Reverse),
            // P10 main intake: forward draws pins inward.
            Motor::new_exp(peripherals.port_10, Direction::Forward),
            // P2 match-load roller: reversed wiring; loading direction is logical reverse.
            Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
            // Raw P8 increases downward; expose positive upward travel to control.
            RotationSensor::new(peripherals.port_8, Direction::Reverse),
            MECHANISM_CALIBRATION,
        ),
    };

    if record_mode {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
