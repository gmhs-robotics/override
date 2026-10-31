//! Device recovery state, independent of hardware.

#[derive(Default)]
pub struct StartupReference {
    initialized: bool,
}

impl StartupReference {
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Retry the startup reference until it succeeds. Never re-zero a device
    /// that already had a valid reference just because it temporarily disconnects.
    pub fn initialize(&mut self, mut initialize: impl FnMut() -> bool) -> bool {
        if !self.initialized {
            self.initialized = initialize();
        }
        self.initialized
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absent_at_boot_retries_until_connected() {
        let mut reference = StartupReference::default();
        let mut attempts = 0;
        for connected in [false, false, true, true] {
            assert_eq!(
                reference.initialize(|| {
                    attempts += 1;
                    connected
                }),
                connected
            );
        }
        assert_eq!(attempts, 3);
    }

    #[test]
    fn reconnect_does_not_assign_a_new_zero() {
        let mut reference = StartupReference::default();
        assert!(reference.initialize(|| true));
        // Reinitializing after a mid-travel disconnection would invent a bottom position.
        assert!(reference.initialize(|| panic!("must preserve existing startup reference")));
    }
}
