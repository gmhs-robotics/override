//! Debug-build terminal diagnostics. Release builds contain no periodic logging.
//! All samples share a monotonic timestamp and sequence number; missing readings
//! are NaN, never fabricated zeros. Events preserve short faults between samples.

#[cfg(debug_assertions)]
use std::time::{Duration, Instant};

#[cfg(debug_assertions)]
pub fn time_ms() -> u128 {
    vexide::time::user_uptime().as_millis()
}

/// Events compile out entirely when debug assertions are disabled.
macro_rules! event {
    ($($arg:tt)*) => {{
        #[cfg(debug_assertions)]
        println!("DBG,EVENT,t_ms={},{}",$crate::diagnostics::time_ms(),format_args!($($arg)*));
    }};
}
pub(crate) use event;

#[cfg(debug_assertions)]
pub struct DebugLog {
    last: Instant,
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
}

#[cfg(debug_assertions)]
impl DebugLog {
    pub fn new() -> Self {
        Self {
            last: Instant::now(),
            sequence: 0,
            ticks: 0,
            max_tick: Duration::ZERO,
        }
    }
    pub fn sample(&mut self, dt: Duration) -> Option<(u64, u128)> {
        self.ticks += 1;
        self.max_tick = self.max_tick.max(dt);
        if self.last.elapsed() < crate::config::DEBUG_SAMPLE_INTERVAL {
            return None;
        }
        self.last = Instant::now();
        self.sequence += 1;
        let ms = time_ms();
        println!(
            "DBG,LOOP,seq={},t_ms={},ticks={},dt_ms={:.3},max_dt_ms={:.3}",
            self.sequence,
            ms,
            self.ticks,
            dt.as_secs_f64() * 1000.0,
            self.max_tick.as_secs_f64() * 1000.0
        );
        self.max_tick = Duration::ZERO;
        Some((self.sequence, ms))
    }
}

#[cfg(debug_assertions)]
#[derive(Default)]
pub struct MechanismDebug {
    pub lift_target: Option<f64>,
    pub pivot_target: Option<f64>,
    pub pivot_command_v: f64,
    pub lift_command_v: f64,
    pub fault: &'static str,
    pub fault_count: u64,
}

#[cfg(debug_assertions)]
impl MechanismDebug {
    pub fn fault(&mut self, reason: &'static str) {
        self.fault_count += 1;
        if self.fault != reason {
            println!(
                "DBG,FAULT,t_ms={},source=mechanisms,code={},count={}",
                time_ms(),
                reason,
                self.fault_count
            );
        }
        self.fault = reason;
    }
    pub fn recovered(&mut self) {
        if !self.fault.is_empty() {
            println!(
                "DBG,RECOVERED,t_ms={},source=mechanisms,previous={},count={}",
                time_ms(),
                self.fault,
                self.fault_count
            );
        }
        self.fault = "";
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn disabled_event_does_not_evaluate_arguments() {
        let count = core::cell::Cell::new(0);
        super::event!("evaluation_test={}", {
            count.set(count.get() + 1);
            count.get()
        });
        assert_eq!(count.get(), u8::from(cfg!(debug_assertions)));
    }
}
