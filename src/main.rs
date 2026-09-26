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
        model::{CoupledDifferential, DriveMotor},
    },
    record::{
        self, DifferentialVoltageFrame, RecordableDrivetrain,
        lift::{LiftFrame, MirroredLift},
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use vexide::{
    adi::digital::LogicLevel,
    color::Color,
    display::{Display, Font, FontFamily, FontSize, Rect, Text, TouchState},
    math::Angle,
    prelude::*,
    time::sleep,
};

// ROBOT CONFIGURATION: replace provisional measurements and signs after commissioning.
const CONTROL_INTERVAL: Duration = Duration::from_millis(20);
const TRACKING_WHEEL_DIAMETER_M: f64 = 0.0508; // 2 inches
// X increases forward; Y increases left. Only perpendicular coordinate affects each wheel.
#[allow(dead_code)] // Along-axis position does not change this wheel's measured travel.
const FORWARD_SENSOR_X_M: f64 = 0.0;
const FORWARD_SENSOR_Y_M: f64 = 0.0;
const SIDEWAYS_SENSOR_X_M: f64 = 0.0;
#[allow(dead_code)] // Along-axis position does not change this wheel's measured travel.
const SIDEWAYS_SENSOR_Y_M: f64 = 0.0;
const FORWARD_WHEEL_DIRECTION: Direction = Direction::Forward;
const SIDEWAYS_WHEEL_DIRECTION: Direction = Direction::Forward;
const ELEVEN_W_WHEEL_PER_MOTOR_REV: f64 = 36.0 / 48.0;
const FIVE_POINT_FIVE_W_WHEEL_PER_MOTOR_REV: f64 = 1.0; // verify compound stage
const MAXIMUM_WHEEL_RPM: f64 = 180.0;
const DRIVE_ACCEL_PER_SECOND: f64 = 1.4;
const DRIVE_DECEL_PER_SECOND: f64 = 1.0;
const TIP_THRESHOLD_RADIANS: f64 = 0.14;
const TIP_PID: (f64, f64, f64) = (1.1, 0.0, 0.06);
const TIP_POLARITY: f64 = -1.0; // vexide pitch is positive nose-down
const TIP_MAXIMUM_CORRECTION: f64 = 0.35;
const CLAW_HIGH_IS_OPEN: bool = true; // confirm valve tubing and polarity
const CLAW_OPEN_ON_BOOT: bool = false;
const LIFT_BOTTOM_DEGREES: f64 = 30.0;
const LIFT_TOP_DEGREES: f64 = 600.0;
const LIFT_MAXIMUM_COMMAND: f64 = 0.75;
const LIFT_UP_RISE_PER_SECOND: f64 = 0.8;
const LIFT_DOWN_RISE_PER_SECOND: f64 = 1.8;
const LIFT_NEAR_TOP_DEGREES: f64 = 80.0;
const LIFT_NEAR_BOTTOM_DEGREES: f64 = 180.0;
const LIFT_SYNC_GAIN: f64 = 0.003;
const LIFT_POSITION_GAIN: f64 = 0.01;
const BOOT_SELECT_TIMEOUT: Duration = Duration::from_secs(4);

type RobotDrive = RecordableDrivetrain<CoupledDifferential, WheeledTracking>;

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    controller: Controller,
    #[record(skip)]
    claw_open: Cell<bool>,
    #[record(skip)]
    imu_calibrated: bool,
    drivetrain: RobotDrive,
    lift: MirroredLift,
    claw: AdiDigitalOut,
}

#[record::async_trait(?Send)]
impl record::Recordable for Robot {
    const UPDATE_INTERVAL: Duration = CONTROL_INTERVAL;

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
                state.left_stick.y(),
                state.right_stick.x(),
            ),
            lift: LiftFrame {
                direction,
                position_degrees: 0.0,
            },
            claw: self.claw_open.get() == CLAW_HIGH_IS_OPEN,
        }
    }

    fn playback_ready(&self) -> bool {
        self.imu_calibrated && self.drivetrain.tracking.is_ready()
    }

    async fn on_playback_abort(&mut self) {
        let _ = self.drivetrain.model.coast_now();
        let _ = self.lift.hold();
    }

    async fn on_save(&mut self) {
        let _ = self.controller.rumble(".").await;
    }
}

fn lift_profile() -> LiftProfile {
    LiftProfile {
        bottom_degrees: LIFT_BOTTOM_DEGREES,
        top_degrees: LIFT_TOP_DEGREES,
        maximum_command: LIFT_MAXIMUM_COMMAND,
        up_rise_per_second: LIFT_UP_RISE_PER_SECOND,
        down_rise_per_second: LIFT_DOWN_RISE_PER_SECOND,
        near_top_degrees: LIFT_NEAR_TOP_DEGREES,
        near_bottom_degrees: LIFT_NEAR_BOTTOM_DEGREES,
        sync_gain: LIFT_SYNC_GAIN,
        position_gain: LIFT_POSITION_GAIN,
    }
}

async fn choose_record_mode(display: &mut Display) -> bool {
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
    while start.elapsed() < BOOT_SELECT_TIMEOUT {
        let touch = display.touch_status();
        if touch.state == TouchState::Pressed {
            return touch.point.y < Display::VERTICAL_RESOLUTION / 2;
        }
        sleep(CONTROL_INTERVAL).await;
    }
    false
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut display = peripherals.display;
    let record_mode = choose_record_mode(&mut display).await;

    // IMU: robot X+ forward, Z+ up. Smart port 15.
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
    let (kp, ki, kd) = TIP_PID;
    let mut model = CoupledDifferential::new(
        left,
        right,
        MAXIMUM_WHEEL_RPM,
        DRIVE_ACCEL_PER_SECOND,
        DRIVE_DECEL_PER_SECOND,
    );
    if imu_ready {
        model = model.with_anti_tip(
            imu,
            AntiTip::new(
                Pid::new(kp, ki, kd, None),
                TIP_THRESHOLD_RADIANS,
                TIP_POLARITY,
                TIP_MAXIMUM_CORRECTION,
            ),
        );
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
        drivetrain: RecordableDrivetrain::new(Drivetrain::new(model, tracking)),
        lift: MirroredLift::new(
            Motor::new(peripherals.port_17, Gearset::Green, Direction::Forward),
            Motor::new(peripherals.port_14, Gearset::Green, Direction::Forward),
            lift_profile(),
        ),
        claw: AdiDigitalOut::with_initial_level(peripherals.adi_a, claw_level),
    };

    if record_mode {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
