//! Bounded route capture, validated storage and smooth clock-driven replay.
//! Uses ozton's recording format and robot mechanism frames; no I/O while moving.
use crate::{config::*, mechanisms::MechanismFrame};
use ozton::record::{
    self, DifferentialVoltageFrame, Interpolate, frame::Recording, routes::RouteIndex,
};
use std::{
    collections::BTreeMap,
    io::{Read, Write},
    path::PathBuf,
    time::Duration,
};

record::record_frame! {
    pub struct RobotFrame {
        pub version: u32,
        pub drivetrain: DifferentialVoltageFrame,
        pub mechanisms: MechanismFrame,
        pub boost: bool,
    }
}

impl RobotFrame {
    pub fn valid(&self) -> bool {
        let m = &self.mechanisms;
        self.version == RECORDING_FORMAT_VERSION
            && [self.drivetrain.left, self.drivetrain.right]
                .iter()
                .all(|v| v.is_finite() && v.abs() <= 1.0)
            && m.lift_position_degrees.is_finite()
            && m.lift_position_degrees * LIFT_RAISE_DIRECTION
                >= MECHANISM_CALIBRATION.lift_bottom_degrees - LIFT_POSITION_TOLERANCE
            && m.lift_position_degrees * LIFT_RAISE_DIRECTION
                <= MECHANISM_CALIBRATION.lift_upper_limit() + LIFT_POSITION_TOLERANCE
            && m.pivot_override_degrees.is_some_and(|p| {
                p.is_finite()
                    && p >= MECHANISM_CALIBRATION.pivot_home_degrees
                        - RECORDING_START_PIVOT_TOLERANCE
                    && p <= MECHANISM_CALIBRATION.pivot_limit(PIVOT_PICKUP_EXTRA_OUTPUT_DEGREES)
                        + RECORDING_START_PIVOT_TOLERANCE
            })
            && m.roller_override_direction
                .is_none_or(|v| (-1..=1).contains(&v))
            && (-1..=1).contains(&m.intake_direction)
            && (-1..=1).contains(&m.lift_direction)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordTarget {
    New,
    Replace(u32),
}

pub struct Capture {
    target: RecordTarget,
    recording: Recording<RobotFrame>,
    elapsed: Duration,
}

impl Capture {
    pub fn new(target: RecordTarget) -> Self {
        Self {
            target,
            recording: Recording::with_frame_capacity(MAX_RECORDING_FRAMES),
            elapsed: Duration::ZERO,
        }
    }
    /// Returns true when capture is full. Caller finishes capture but keeps driving.
    pub fn push(&mut self, frame: RobotFrame, elapsed: Duration) -> Result<bool, &'static str> {
        if !frame.valid() || elapsed > MAX_CONTROL_GAP {
            return Err("Recording discarded: feedback");
        }
        let dt = if self.recording.frames.is_empty() {
            Duration::ZERO
        } else {
            elapsed
        };
        if self.recording.frames.len() >= MAX_RECORDING_FRAMES
            || self.elapsed.saturating_add(dt) > MAX_RECORDING_DURATION
        {
            return Ok(true);
        }
        self.elapsed = self.elapsed.saturating_add(dt);
        self.recording.push_timed(dt, frame);
        Ok(self.recording.frames.len() >= MAX_RECORDING_FRAMES
            || self.elapsed >= MAX_RECORDING_DURATION)
    }
    #[cfg(debug_assertions)]
    pub fn debug_sample(&self, seq: u64, ms: u128) {
        println!(
            "DBG,RECORD,seq={seq},t_ms={ms},target={:?},frames={},duration_ms={}",
            self.target,
            self.recording.frames.len(),
            self.elapsed.as_millis()
        );
    }
    pub fn finish(self) -> Option<PendingRecording> {
        (!self.recording.frames.is_empty()).then_some(PendingRecording {
            target: self.target,
            recording: self.recording,
            allocated_file: None,
        })
    }
}

pub struct PendingRecording {
    target: RecordTarget,
    recording: Recording<RobotFrame>,
    allocated_file: Option<u32>,
}

/// Immutable route data and an append-only commit index work with VEXos's limited
/// filesystem. Replacing a route never truncates its previous recording.
pub struct RouteStore {
    root: PathBuf,
    pub index: RouteIndex,
    files: BTreeMap<u32, u32>,
}

impl RouteStore {
    pub fn open(root: impl Into<PathBuf>) -> Result<Self, &'static str> {
        let root = root.into();
        let path = root.join(RouteIndex::INDEX_PATH);
        let text = match std::fs::File::open(&path) {
            Ok(file) => {
                let mut text = String::new();
                file.take(MAX_ROUTE_INDEX_BYTES + 1)
                    .read_to_string(&mut text)
                    .map_err(|_| "Cannot read route index")?;
                if text.len() as u64 > MAX_ROUTE_INDEX_BYTES {
                    return Err("Route index full");
                }
                text
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
            Err(_) => return Err("Cannot read route index"),
        };
        let mut store = Self::empty(root);
        for line in text
            .split_inclusive('\n')
            .filter(|line| line.ends_with('\n'))
        {
            let fields: Vec<_> = line.trim_end().split('\t').collect();
            let (id, name, file) = match fields.as_slice() {
                [id, name] => (*id, *name, *id), // Existing ozton index entry.
                [id, name, file, "COMMIT"] => (*id, *name, *file),
                _ => continue,
            };
            if let (Ok(id), Ok(file)) = (id.parse::<u32>(), file.parse::<u32>())
                && !name.trim().is_empty()
                && id > 0
                && file > 0
                && (store.index.map.contains_key(&id) || store.index.len() < MAX_SAVED_ROUTES)
            {
                store.index.update(id, name.trim());
                store.files.insert(id, file);
            }
        }
        Ok(store)
    }
    pub fn empty(root: impl Into<PathBuf>) -> Self {
        Self {
            root: root.into(),
            index: RouteIndex::default(),
            files: BTreeMap::new(),
        }
    }
    /// Call only while stopped. Keep pending capture on error so the operator can retry.
    pub fn save(&mut self, pending: &mut PendingRecording) -> Result<u32, &'static str> {
        let id = match pending.target {
            RecordTarget::Replace(id) if self.index.map.contains_key(&id) => id,
            RecordTarget::Replace(_) => return Err("Recording target missing"),
            RecordTarget::New => {
                if self.index.len() >= MAX_SAVED_ROUTES {
                    return Err("Route storage full");
                }
                (1..=MAX_ROUTE_ID)
                    .find(|id| !self.index.map.contains_key(id))
                    .ok_or("No free route slots")?
            }
        };
        let file_id = match pending.allocated_file {
            Some(id) => id,
            None => {
                let id = (1..=MAX_ROUTE_ID)
                    .find(|id| !self.root.join(RouteIndex::path_for(*id)).exists())
                    .ok_or("SD route files full")?;
                pending.allocated_file = Some(id);
                id
            }
        };
        let path = self.root.join(RouteIndex::path_for(file_id));
        pending
            .recording
            .save(&path)
            .map_err(|_| "Save failed: check SD card")?;
        // Sync and validate before making the new data visible to route selection.
        std::fs::File::open(&path)
            .and_then(|f| f.sync_all())
            .map_err(|_| "Save failed: sync")?;
        let check =
            Recording::<RobotFrame>::load(&path).map_err(|_| "Save failed: verification")?;
        let _ = Playback::new(check).map_err(|_| "Save failed: invalid capture")?;
        let name = self
            .index
            .display_name(id)
            .map(str::to_owned)
            .unwrap_or_else(|| format!("Route {id}"));
        let entry = format!("\n{id}\t{name}\t{file_id}\tCOMMIT\n");
        let index_path = self.root.join(RouteIndex::INDEX_PATH);
        if std::fs::metadata(&index_path)
            .is_ok_and(|m| m.len() + entry.len() as u64 > MAX_ROUTE_INDEX_BYTES)
        {
            return Err("Route index full");
        }
        let mut index_file = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(&index_path)
            .map_err(|_| "Route saved; index failed")?;
        index_file
            .write_all(entry.as_bytes())
            .and_then(|_| index_file.flush())
            .map_err(|_| "Route saved; index failed")?;
        crate::diagnostics::event!(
            "saved_route_id={},file_id={},frames={},duration_ms={}",
            id,
            file_id,
            pending.recording.frames.len(),
            pending.recording.duration().as_millis()
        );
        self.index.update(id, &name);
        self.files.insert(id, file_id);
        Ok(id)
    }
    pub fn load(&self, id: u32) -> Result<Playback, &'static str> {
        let file_id = self
            .files
            .get(&id)
            .copied()
            .ok_or("Route missing from index")?;
        let path = self.root.join(RouteIndex::path_for(file_id));
        if std::fs::metadata(&path)
            .map_err(|_| "Route missing: check SD card")?
            .len()
            > MAX_RECORDING_BYTES
        {
            return Err("Route file too large");
        }
        let recording =
            Recording::<RobotFrame>::load(path).map_err(|_| "Route incompatible or damaged")?;
        crate::diagnostics::event!(
            "loaded_route_id={},file_id={},frames={},duration_ms={}",
            id,
            file_id,
            recording.frames.len(),
            recording.duration().as_millis()
        );
        Playback::new(recording)
    }
}

pub struct Playback {
    recording: Recording<RobotFrame>,
    elapsed: Duration,
    index: usize,
    frame_time: Duration,
    duration: Duration,
    started: bool,
}

impl Playback {
    pub fn new(recording: Recording<RobotFrame>) -> Result<Self, &'static str> {
        if recording.frames.is_empty()
            || recording.frames.len() > MAX_RECORDING_FRAMES
            || recording.duration() > MAX_RECORDING_DURATION
            || recording
                .frames
                .first()
                .is_none_or(|f| f.delta_time_micros != 0)
            || recording.frames.iter().any(|f| {
                !f.frame.valid() || Duration::from_micros(f.delta_time_micros) > MAX_CONTROL_GAP
            })
        {
            return Err("Route invalid or incompatible");
        }
        let duration = recording.duration();
        Ok(Self {
            recording,
            elapsed: Duration::ZERO,
            index: 0,
            frame_time: Duration::ZERO,
            duration,
            started: false,
        })
    }
    #[cfg(debug_assertions)]
    pub fn debug_sample(&self, seq: u64, ms: u128) {
        println!(
            "DBG,PLAYBACK,seq={seq},t_ms={ms},elapsed_ms={},duration_ms={},frame_index={},frame_count={}",
            self.elapsed.as_millis(),
            self.duration.as_millis(),
            self.index,
            self.recording.frames.len()
        );
    }
    pub fn start_pose_matches(&self, height: f64, pivot: f64) -> bool {
        let Some(first) = self.recording.frames.first() else {
            return false;
        };
        let frame = &first.frame.mechanisms;
        height.is_finite()
            && pivot.is_finite()
            && (height - frame.lift_position_degrees * LIFT_RAISE_DIRECTION).abs()
                <= RECORDING_START_LIFT_TOLERANCE
            && frame
                .pivot_override_degrees
                .is_some_and(|p| (pivot - p).abs() <= RECORDING_START_PIVOT_TOLERANCE)
    }
    /// Interpolate continuous signals while keeping buttons discrete. Cursor only moves forward.
    pub fn update(&mut self, dt: Duration) -> Result<Option<RobotFrame>, &'static str> {
        if dt.is_zero() || dt > MAX_CONTROL_GAP {
            return Err("Playback stopped: control delay");
        }
        if self.started {
            self.elapsed = self.elapsed.saturating_add(dt);
        }
        self.started = true;
        if self.elapsed > self.duration {
            return Ok(None);
        }
        while let Some(next) = self.recording.frames.get(self.index + 1) {
            let next_time = self
                .frame_time
                .saturating_add(Duration::from_micros(next.delta_time_micros));
            if next_time > self.elapsed {
                break;
            }
            self.index += 1;
            self.frame_time = next_time;
        }
        let Some(current) = self.recording.frames.get(self.index) else {
            return Ok(None);
        };
        if let Some(next) = self.recording.frames.get(self.index + 1) {
            let span = Duration::from_micros(next.delta_time_micros).as_secs_f64();
            if span > 0.0 {
                let amount = self.elapsed.saturating_sub(self.frame_time).as_secs_f64() / span;
                return Ok(Some(RobotFrame {
                    drivetrain: DifferentialVoltageFrame::interpolate(
                        &current.frame.drivetrain,
                        &next.frame.drivetrain,
                        amount,
                    ),
                    mechanisms: MechanismFrame {
                        lift_position_degrees: f64::interpolate(
                            &current.frame.mechanisms.lift_position_degrees,
                            &next.frame.mechanisms.lift_position_degrees,
                            amount,
                        ),
                        pivot_override_degrees: current
                            .frame
                            .mechanisms
                            .pivot_override_degrees
                            .zip(next.frame.mechanisms.pivot_override_degrees)
                            .map(|(a, b)| f64::interpolate(&a, &b, amount)),
                        ..current.frame.mechanisms.clone()
                    },
                    ..current.frame.clone()
                }));
            }
        }
        Ok(Some(current.frame.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(height: f64, pivot: f64, drive: f64) -> RobotFrame {
        RobotFrame {
            version: RECORDING_FORMAT_VERSION,
            drivetrain: DifferentialVoltageFrame::tank(drive, drive),
            mechanisms: MechanismFrame {
                lift_position_degrees: -height,
                pivot_override_degrees: Some(pivot),
                ..Default::default()
            },
            boost: false,
        }
    }
    fn pending(target: RecordTarget, drive: f64) -> PendingRecording {
        let mut capture = Capture::new(target);
        capture
            .push(frame(0.0, 0.0, drive), CONTROL_INTERVAL)
            .unwrap();
        capture
            .push(frame(10.0, 20.0, drive), CONTROL_INTERVAL)
            .unwrap();
        capture.finish().unwrap()
    }
    fn folder() -> PathBuf {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let root = std::env::temp_dir().join(format!(
            "override-record-test-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&root).unwrap();
        root
    }
    #[test]
    fn record_save_reopen_select_and_playback_round_trip_and_replace() {
        let root = folder();
        let mut store = RouteStore::open(&root).unwrap();
        let mut first = pending(RecordTarget::New, 0.25);
        let id = store.save(&mut first).unwrap();
        assert_eq!(id, 1);
        let mut store = RouteStore::open(&root).unwrap();
        let mut playback = store.load(id).unwrap();
        assert!(playback.start_pose_matches(0.0, 0.0));
        assert!(!playback.start_pose_matches(100.0, 0.0));
        assert!(!playback.start_pose_matches(0.0, 40.0));
        assert_eq!(
            playback
                .update(CONTROL_INTERVAL)
                .unwrap()
                .unwrap()
                .drivetrain
                .left,
            0.25
        );
        assert_eq!(
            playback
                .update(CONTROL_INTERVAL)
                .unwrap()
                .unwrap()
                .mechanisms
                .lift_position_degrees,
            -10.0
        );
        assert!(playback.update(CONTROL_INTERVAL).unwrap().is_none());
        let mut replacement = pending(RecordTarget::Replace(id), -0.2);
        assert_eq!(store.save(&mut replacement).unwrap(), id);
        assert_eq!(store.index.len(), 1);
        let store = RouteStore::open(&root).unwrap();
        assert_eq!(
            store
                .load(id)
                .unwrap()
                .update(CONTROL_INTERVAL)
                .unwrap()
                .unwrap()
                .drivetrain
                .left,
            -0.2
        );
        assert!(root.join("1.route").exists()); // Previous data survives replacement.
        assert!(root.join("2.route").exists());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn interpolation_is_smooth_and_discrete_commands_are_not_early() {
        let mut recording = Recording::default();
        let a = frame(0.0, 0.0, 0.0);
        let mut b = frame(100.0, 200.0, 0.6);
        b.boost = true;
        b.mechanisms.intake_direction = -1;
        recording.push_timed(Duration::ZERO, a);
        recording.push_timed(Duration::from_millis(100), b);
        let mut playback = Playback::new(recording).unwrap();
        playback.update(CONTROL_INTERVAL).unwrap();
        for i in 1..10 {
            let frame = playback.update(CONTROL_INTERVAL).unwrap().unwrap();
            assert!((frame.drivetrain.left - 0.06 * f64::from(i)).abs() < 1e-9);
            assert!((frame.mechanisms.lift_position_degrees + 10.0 * f64::from(i)).abs() < 1e-9);
            assert_eq!(frame.mechanisms.intake_direction, 0);
            assert!(!frame.boost);
        }
        let last = playback.update(CONTROL_INTERVAL).unwrap().unwrap();
        assert_eq!(last.mechanisms.intake_direction, -1);
        assert!(last.boost);
        assert!(playback.update(CONTROL_INTERVAL).unwrap().is_none());
    }
    #[test]
    fn capture_is_bounded_and_invalid_values_fail_before_replay() {
        assert!(Capture::new(RecordTarget::New).finish().is_none());
        let mut capture = Capture::new(RecordTarget::New);
        let mut full = false;
        for _ in 0..2000 {
            if capture
                .push(frame(0.0, 0.0, 0.2), CONTROL_INTERVAL)
                .unwrap()
            {
                full = true;
                break;
            }
        }
        assert!(full);
        let recording = capture.finish().unwrap().recording;
        assert!(recording.duration() <= MAX_RECORDING_DURATION);
        assert!(recording.frames.len() <= MAX_RECORDING_FRAMES);
        assert!(Playback::new(recording).is_ok());
        for invalid in [
            frame(f64::NAN, 0.0, 0.0),
            frame(0.0, f64::NAN, 0.0),
            frame(0.0, 0.0, 2.0),
        ] {
            assert!(
                Capture::new(RecordTarget::New)
                    .push(invalid, CONTROL_INTERVAL)
                    .is_err()
            );
        }
        let mut invalid = pending(RecordTarget::New, 0.1).recording;
        invalid.frames[0].frame.version = 0;
        assert!(Playback::new(invalid).is_err());
    }
    #[test]
    fn torn_index_tail_or_corrupt_route_never_becomes_executable() {
        let root = folder();
        let mut store = RouteStore::open(&root).unwrap();
        let id = store.save(&mut pending(RecordTarget::New, 0.2)).unwrap();
        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .open(root.join(".routes"))
            .unwrap();
        file.write_all(b"\n1\tBroken replacement\t99\tCOM").unwrap();
        drop(file);
        let store = RouteStore::open(&root).unwrap();
        assert!(store.load(id).is_ok());
        std::fs::write(root.join("1.route"), b"invalid route").unwrap();
        assert!(store.load(id).is_err());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn failed_save_retains_capture_for_retry_without_changing_index() {
        let root = folder();
        let absent = root.join("card");
        let mut store = RouteStore::empty(&absent);
        let mut pending = pending(RecordTarget::New, 0.1);
        assert!(store.save(&mut pending).is_err());
        assert_eq!(store.index.len(), 0);
        std::fs::create_dir(&absent).unwrap();
        assert_eq!(store.save(&mut pending).unwrap(), 1);
        assert!(RouteStore::open(&absent).unwrap().load(1).is_ok());
        std::fs::remove_dir_all(root).unwrap();
    }
}
