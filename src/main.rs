use std::{
    cell::Cell,
    rc::Rc,
    time::{Duration, Instant},
};

use ozton::{
    control::{drive::AntiTip, lift::LiftProfile, loops::Pid},
    derive::RecordedRobot,
    drivetrain::{
        Drivetrain,
        model::{CoupledDifferential, DriveMotor, Tank, TipAngleAxis},
    },
    record::{
        self, DifferentialVoltageFrame, RecordField, RecordableDrivetrain,
        frame::{PlaybackOutcome, RecordMode},
        lift::{LiftFrame, MirroredLift},
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use vexide::adi::digital::LogicLevel;
use vexide::{
    color::Color,
    display::{Display, Font, FontFamily, FontSize, Rect, Text, TouchState},
    math::Angle,
    prelude::*,
    smart::PortError,
    time::sleep,
};

// ROBOT CONFIGURATION: replace provisional measurements and signs after commissioning.
const CONTROL_INTERVAL: Duration = Duration::from_millis(10);
const TRACKING_WHEEL_DIAMETER_M: f64 = 0.0508; // provisional 2 inches; confirm tread diameter
// X increases forward; Y increases left. Only perpendicular coordinate affects each wheel.
const FORWARD_SENSOR_Y_M: f64 = 0.00635; // 1/4 inch left
const SIDEWAYS_SENSOR_X_M: f64 = -0.0762; // 3 inches behind
const FORWARD_WHEEL_DIRECTION: Direction = Direction::Forward;
const SIDEWAYS_WHEEL_DIRECTION: Direction = Direction::Forward;
const ELEVEN_W_WHEEL_PER_MOTOR_REV: f64 = 36.0 / 48.0;
const FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV: f64 = 1.0; // verify compound stage
const TIP_THRESHOLD_RADIANS: f64 = 0.16;
const TIP_PID: (f64, f64, f64) = (1.35, 0.0, 0.03);
const TIP_POLARITY: f64 = -1.0; // sensor roll positive when nose tips down
const TIP_MAXIMUM_CORRECTION: f64 = 0.35;
const CLAW_HIGH_IS_OPEN: bool = false; // observed low output opens the claw
const CLAW_OPEN_ON_BOOT: bool = false;
const RECORD_DRIVE_LIMIT: f64 = 0.45;
const LIFT_BOTTOM_DEGREES: f64 = 30.0;
const LIFT_TOP_DEGREES: f64 = 600.0;
const LIFT_UP_MAXIMUM_COMMAND: f64 = 0.75;
const LIFT_DOWN_MAXIMUM_COMMAND: f64 = 0.33;
const LIFT_UP_RISE_PER_SECOND: f64 = 1.5;
const LIFT_DOWN_RISE_PER_SECOND: f64 = 0.9;
const LIFT_POSITION_GAIN: f64 = 0.01;
const BOOT_SELECT_TIMEOUT: Duration = Duration::from_secs(4);
const OFF_WALL_DRIVE_POWER: f64 = 0.35;
const OFF_WALL_DRIVE_TIME: Duration = Duration::from_millis(750);
const TOGGLE_MOTOR_RPM: i32 = 100;
const TOGGLE_MOVE_TIMEOUT: Duration = Duration::from_secs(3);
const TOGGLE_POSITION_TOLERANCE_DEGREES: f64 = 8.0;
const TOGGLE_PAUSE: Duration = Duration::from_millis(250);
const TOGGLE_STALL_WINDOW: Duration = Duration::from_millis(400);
const TOGGLE_MIN_PROGRESS_DEGREES: f64 = 5.0;

type RobotDrive = RecordableDrivetrain<CoupledDifferential, WheeledTracking>;

struct PneumaticClaw {
    output: AdiDigitalOut,
}

#[record::async_trait(?Send)]
impl RecordField for PneumaticClaw {
    type Output = bool;

    async fn apply_frame_value(&mut self, open: &bool, _: RecordMode) -> Result<(), PortError> {
        self.output.set_level(if *open == CLAW_HIGH_IS_OPEN {
            LogicLevel::High
        } else {
            LogicLevel::Low
        })
    }
}

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    controller: Controller,
    #[record(skip)]
    claw_open: Cell<bool>,
    #[record(skip)]
    imu_calibrated: bool,
    #[record(skip)]
    record_mode: bool,
    drivetrain: RobotDrive,
    lift: MirroredLift,
    pneumatic_claw: PneumaticClaw,
    auxiliary_motor: Motor,
}

#[record::async_trait(?Send)]
impl record::Recordable for Robot {
    const UPDATE_INTERVAL: Duration = CONTROL_INTERVAL;
    const PREDETERMINED_ROUTE_NAME: Option<&'static str> = Some("Move Off Wall");
    // Hidden until the mechanism's safe motor travel is measured. Both routes
    // currently request 180 degrees, which exceeds the observed hard stop.
    const ADDITIONAL_PREDETERMINED_ROUTE_NAMES: &'static [&'static str] = &[];

    async fn run_predetermined_route(&mut self) -> PlaybackOutcome {
        if self
            .drivetrain
            .model
            .drive_tank(OFF_WALL_DRIVE_POWER, OFF_WALL_DRIVE_POWER)
            .is_err()
        {
            self.on_playback_abort().await;
            return PlaybackOutcome::OutputFailed;
        }

        sleep(OFF_WALL_DRIVE_TIME).await;
        if self.drivetrain.model.coast_now().is_err() {
            self.on_playback_abort().await;
            return PlaybackOutcome::OutputFailed;
        }
        PlaybackOutcome::Completed
    }

    async fn run_additional_predetermined_route(&mut self, index: usize) -> PlaybackOutcome {
        let half_turns = match index {
            0 => 2,
            1 => 1,
            _ => return PlaybackOutcome::OutputFailed,
        };

        for half_turn in 0..half_turns {
            match self.turn_toggle_motor(-180.0).await {
                Ok(true) => {}
                Ok(false) | Err(_) => {
                    self.on_playback_abort().await;
                    return PlaybackOutcome::OutputFailed;
                }
            }
            if half_turn + 1 < half_turns {
                sleep(TOGGLE_PAUSE).await;
            }
        }

        if self.auxiliary_motor.set_voltage(0.0).is_err() {
            self.on_playback_abort().await;
            return PlaybackOutcome::OutputFailed;
        }
        PlaybackOutcome::Completed
    }

    async fn get_new_frame(&self) -> Self::Frame {
        let state = self.controller.state().unwrap_or_default();
        if state.button_l1.is_now_pressed() {
            self.claw_open.set(!self.claw_open.get());
        }
        let direction = match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
            (true, false) => 1,
            (false, true) => -1,
            _ => 0,
        };
        Self::Frame {
            drivetrain: DifferentialVoltageFrame::arcade(
                state.left_stick.y()
                    * if self.record_mode {
                        RECORD_DRIVE_LIMIT
                    } else {
                        1.0
                    },
                state.right_stick.x()
                    * if self.record_mode {
                        RECORD_DRIVE_LIMIT
                    } else {
                        1.0
                    },
            ),
            lift: LiftFrame {
                direction,
                position_degrees: 0.0,
            },
            pneumatic_claw: self.claw_open.get(),
            auxiliary_motor: if state.button_x.is_pressed() {
                -1.0
            } else {
                0.0
            },
        }
    }

    fn playback_ready(&self) -> bool {
        self.imu_calibrated && self.drivetrain.tracking.is_ready()
    }

    async fn on_playback_abort(&mut self) {
        let _ = self.drivetrain.model.coast_now();
        let _ = self.lift.hold();
        let _ = self.auxiliary_motor.set_voltage(0.0);
    }

    async fn on_save(&mut self) {
        let _ = self.controller.rumble(".").await;
    }
}

impl Robot {
    async fn turn_toggle_motor(&mut self, degrees: f64) -> Result<bool, PortError> {
        let target = self.auxiliary_motor.position()? + Angle::from_degrees(degrees);
        self.auxiliary_motor
            .set_position_target(target, TOGGLE_MOTOR_RPM)?;
        let start = Instant::now();
        let mut progress_start = start;
        let mut previous_error = degrees.abs();
        loop {
            let error = (target - self.auxiliary_motor.position()?)
                .as_degrees()
                .abs();
            if error <= TOGGLE_POSITION_TOLERANCE_DEGREES {
                self.auxiliary_motor.set_voltage(0.0)?;
                return Ok(true);
            }
            if start.elapsed() >= TOGGLE_MOVE_TIMEOUT {
                self.auxiliary_motor.set_voltage(0.0)?;
                return Ok(false);
            }
            if progress_start.elapsed() >= TOGGLE_STALL_WINDOW {
                if previous_error - error < TOGGLE_MIN_PROGRESS_DEGREES {
                    self.auxiliary_motor.set_voltage(0.0)?;
                    return Ok(false);
                }
                previous_error = error;
                progress_start = Instant::now();
            }
            sleep(CONTROL_INTERVAL).await;
        }
    }
}

fn lift_profile() -> LiftProfile {
    LiftProfile {
        bottom_degrees: LIFT_BOTTOM_DEGREES,
        top_degrees: LIFT_TOP_DEGREES,
        up_maximum_command: LIFT_UP_MAXIMUM_COMMAND,
        down_maximum_command: LIFT_DOWN_MAXIMUM_COMMAND,
        up_rise_per_second: LIFT_UP_RISE_PER_SECOND,
        down_rise_per_second: LIFT_DOWN_RISE_PER_SECOND,
        position_gain: LIFT_POSITION_GAIN,
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

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut display = peripherals.display;
    let record_mode = choose_record_mode(&mut display).await;

    // IMU: sensor Y points forward and sensor Z points down. Smart port 15.
    // Raw heading decreases CCW; ozton's Gyro adapter converts it to CCW-positive.
    let mut imu = InertialSensor::new(peripherals.port_15);
    let imu_ready = imu.calibrate().await.is_ok();
    if !imu_ready {
        println!("IMU calibration failed; recording and playback unavailable");
    }
    let imu = Rc::new(imu);

    // Rotation sensors: forward wheel on P1, sideways wheel on P10.
    let tracking = WheeledTracking::new(
        [0.0, 0.0],
        Angle::ZERO,
        [TrackingWheel::new(
            RotationSensor::new(peripherals.port_1, FORWARD_WHEEL_DIRECTION),
            TRACKING_WHEEL_DIAMETER_M,
            FORWARD_SENSOR_Y_M,
            None,
        )],
        [TrackingWheel::new(
            RotationSensor::new(peripherals.port_10, SIDEWAYS_WHEEL_DIRECTION),
            TRACKING_WHEEL_DIAMETER_M,
            -SIDEWAYS_SENSOR_X_M,
            None,
        )],
        Some(imu.clone()),
    );

    let left = [
        DriveMotor::new(
            Motor::new_exp(peripherals.port_12, Direction::Reverse),
            FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
    ];
    let right = [
        DriveMotor::new(
            Motor::new_exp(peripherals.port_19, Direction::Forward),
            FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
        DriveMotor::new(
            Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
            ELEVEN_W_WHEEL_PER_MOTOR_REV,
        ),
    ];
    let mut model = CoupledDifferential::new_direct_voltage(left, right);
    if imu_ready {
        match imu.euler() {
            Ok(angles) => {
                let (kp, ki, kd) = TIP_PID;
                model = model.with_anti_tip_axis(
                    imu.clone(),
                    AntiTip::new(
                        Pid::new(kp, ki, kd, None),
                        TIP_THRESHOLD_RADIANS,
                        TIP_POLARITY,
                        TIP_MAXIMUM_CORRECTION,
                    ),
                    TipAngleAxis::Roll,
                    angles.c,
                );
            }
            Err(error) => println!("Anti-tip disabled: IMU roll unavailable: {error:?}"),
        }
    }

    let claw_level = if CLAW_OPEN_ON_BOOT == CLAW_HIGH_IS_OPEN {
        LogicLevel::High
    } else {
        LogicLevel::Low
    };
    let robot = Robot {
        controller: peripherals.primary_controller,
        claw_open: Cell::new(CLAW_OPEN_ON_BOOT),
        imu_calibrated: imu_ready,
        record_mode,
        drivetrain: RecordableDrivetrain::new(Drivetrain::new(model, tracking)),
        lift: MirroredLift::new(
            Motor::new(peripherals.port_17, Gearset::Green, Direction::Reverse),
            Motor::new(peripherals.port_14, Gearset::Green, Direction::Forward),
            lift_profile(),
        ),
        pneumatic_claw: PneumaticClaw {
            output: AdiDigitalOut::with_initial_level(peripherals.adi_a, claw_level),
        },
        auxiliary_motor: Motor::new_exp(peripherals.port_16, Direction::Forward),
    };

    if record_mode {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
