use crate::event::Event;

/// Event decimation filter — passes every Nth event.
///
/// Useful for reducing event rate in bandwidth-limited applications
/// or for downsampling event streams for visualization/logging.
pub struct DecimationFilter {
    factor: u32,
    counter: u32,
}

impl DecimationFilter {
    /// Create a new decimation filter that passes every `factor`-th event.
    ///
    /// A factor of 1 passes all events; a factor of 10 passes ~10% of events.
    ///
    /// # Panics
    /// Panics if `factor` is 0.
    pub fn new(factor: u32) -> Self {
        assert!(factor > 0, "decimation factor must be positive");
        Self { factor, counter: 0 }
    }

    /// Returns true if this event should be kept (every Nth event).
    pub fn filter(&mut self, _event: &Event) -> bool {
        self.counter += 1;
        if self.counter >= self.factor {
            self.counter = 0;
            true
        } else {
            false
        }
    }

    pub fn factor(&self) -> u32 {
        self.factor
    }

    pub fn set_factor(&mut self, factor: u32) {
        assert!(factor > 0, "decimation factor must be positive");
        self.factor = factor;
        self.counter = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_factor_1_passes_all() {
        let mut filter = DecimationFilter::new(1);
        let ev = Event::new(10, 10, 100_000, 1);
        for _ in 0..10 {
            assert!(filter.filter(&ev));
        }
    }

    #[test]
    fn test_factor_2_passes_half() {
        let mut filter = DecimationFilter::new(2);
        let ev = Event::new(10, 10, 100_000, 1);
        let mut passed = 0;
        for _ in 0..10 {
            if filter.filter(&ev) {
                passed += 1;
            }
        }
        assert_eq!(passed, 5);
    }

    #[test]
    fn test_factor_5_passes_fifth() {
        let mut filter = DecimationFilter::new(5);
        let ev = Event::new(10, 10, 100_000, 1);
        let mut passed = 0;
        for _ in 0..20 {
            if filter.filter(&ev) {
                passed += 1;
            }
        }
        assert_eq!(passed, 4);
    }

    #[test]
    fn test_pattern_correct() {
        let mut filter = DecimationFilter::new(3);
        let ev = Event::new(10, 10, 100_000, 1);
        // Pattern: reject, reject, pass, reject, reject, pass, ...
        assert!(!filter.filter(&ev));
        assert!(!filter.filter(&ev));
        assert!(filter.filter(&ev));
        assert!(!filter.filter(&ev));
        assert!(!filter.filter(&ev));
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_set_factor_resets_counter() {
        let mut filter = DecimationFilter::new(3);
        let ev = Event::new(10, 10, 100_000, 1);
        filter.filter(&ev); // counter = 1
        filter.set_factor(2);
        // Counter reset to 0, so next two: reject, pass
        assert!(!filter.filter(&ev));
        assert!(filter.filter(&ev));
    }

    #[test]
    #[should_panic(expected = "decimation factor must be positive")]
    fn test_zero_factor_panics() {
        DecimationFilter::new(0);
    }
}
