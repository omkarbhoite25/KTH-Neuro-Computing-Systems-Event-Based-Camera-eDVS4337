use crate::event::Event;

/// Slicing mode for the event stream slicer.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SliceMode {
    /// Slice by event count — triggers after every N events.
    ByCount = 0,
    /// Slice by time window — triggers every T microseconds.
    ByTime = 1,
}

impl SliceMode {
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(SliceMode::ByCount),
            1 => Some(SliceMode::ByTime),
            _ => None,
        }
    }
}

/// Event stream slicer — triggers at regular intervals (count or time based).
///
/// Returns true when a slice boundary is reached. Useful for batch processing
/// event streams into fixed-size chunks for downstream algorithms.
pub struct EventSlicer {
    mode: SliceMode,
    threshold: i64,
    counter: i64,
    window_start: i64,
}

impl EventSlicer {
    /// Create a new event slicer.
    ///
    /// - `mode`: `ByCount` or `ByTime`.
    /// - `threshold`: Number of events (ByCount) or microseconds (ByTime) per slice.
    pub fn new(mode: SliceMode, threshold: i64) -> Self {
        assert!(threshold > 0, "threshold must be positive");
        Self {
            mode,
            threshold,
            counter: 0,
            window_start: 0,
        }
    }

    /// Process an event. Returns true if a slice boundary was just reached.
    pub fn process(&mut self, event: &Event) -> bool {
        match self.mode {
            SliceMode::ByCount => {
                self.counter += 1;
                if self.counter >= self.threshold {
                    self.counter = 0;
                    true
                } else {
                    false
                }
            }
            SliceMode::ByTime => {
                let ts = event.timestamp;
                if self.window_start == 0 {
                    self.window_start = ts;
                }
                if ts.saturating_sub(self.window_start) >= self.threshold {
                    self.window_start = ts;
                    true
                } else {
                    false
                }
            }
        }
    }

    /// Reset the slicer state.
    pub fn reset(&mut self) {
        self.counter = 0;
        self.window_start = 0;
    }

    pub fn mode(&self) -> SliceMode {
        self.mode
    }

    pub fn threshold(&self) -> i64 {
        self.threshold
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(ts: i64) -> Event {
        Event::new(0, 0, ts, 1)
    }

    #[test]
    fn test_by_count_triggers_at_threshold() {
        let mut slicer = EventSlicer::new(SliceMode::ByCount, 5);
        for i in 0..4 {
            assert!(!slicer.process(&ev(i * 1000)), "should not trigger before threshold");
        }
        assert!(slicer.process(&ev(4000)), "should trigger at 5th event");
    }

    #[test]
    fn test_by_count_repeats() {
        let mut slicer = EventSlicer::new(SliceMode::ByCount, 3);
        let mut triggers = 0;
        for i in 0..12 {
            if slicer.process(&ev(i * 1000)) {
                triggers += 1;
            }
        }
        assert_eq!(triggers, 4); // 12 / 3 = 4
    }

    #[test]
    fn test_by_time_triggers_at_window() {
        let mut slicer = EventSlicer::new(SliceMode::ByTime, 10_000);
        assert!(!slicer.process(&ev(1000)));
        assert!(!slicer.process(&ev(5000)));
        assert!(!slicer.process(&ev(9000)));
        assert!(slicer.process(&ev(11_000)), "should trigger at 10ms");
    }

    #[test]
    fn test_by_time_repeats() {
        let mut slicer = EventSlicer::new(SliceMode::ByTime, 10_000);
        let mut triggers = 0;
        for i in 0..50 {
            if slicer.process(&ev(i * 1000)) {
                triggers += 1;
            }
        }
        // First trigger at ~10ms, then every 10ms
        assert!(triggers >= 3);
    }

    #[test]
    fn test_reset() {
        let mut slicer = EventSlicer::new(SliceMode::ByCount, 3);
        slicer.process(&ev(1000));
        slicer.process(&ev(2000));
        slicer.reset();
        // Counter should be reset, need 3 more events
        assert!(!slicer.process(&ev(3000)));
        assert!(!slicer.process(&ev(4000)));
        assert!(slicer.process(&ev(5000)));
    }

    #[test]
    fn test_from_i32() {
        assert_eq!(SliceMode::from_i32(0), Some(SliceMode::ByCount));
        assert_eq!(SliceMode::from_i32(1), Some(SliceMode::ByTime));
        assert_eq!(SliceMode::from_i32(99), None);
    }
}
