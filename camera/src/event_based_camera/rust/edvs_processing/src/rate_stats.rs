use crate::event::Event;

/// Event rate statistics tracker.
///
/// Tracks global event count and rate over a configurable sliding time window.
/// Provides current rate, peak rate, and total event count. Useful as a
/// diagnostic tool to monitor sensor activity.
pub struct EventRateStats {
    window_us: i64,
    window_start: i64,
    window_count: u64,
    total_count: u64,
    peak_rate: f64,
    last_rate: f64,
    initialized: bool,
}

impl EventRateStats {
    /// Create a new rate statistics tracker.
    ///
    /// - `window_us`: Time window for rate computation in microseconds.
    pub fn new(window_us: i64) -> Self {
        assert!(window_us > 0, "window_us must be positive");
        Self {
            window_us,
            window_start: 0,
            window_count: 0,
            total_count: 0,
            peak_rate: 0.0,
            last_rate: 0.0,
            initialized: false,
        }
    }

    /// Record an event and update statistics.
    /// Returns the current rate (events per second) if a window boundary was crossed.
    pub fn record(&mut self, event: &Event) -> Option<f64> {
        let ts = event.timestamp;
        self.total_count += 1;
        self.window_count += 1;

        if !self.initialized {
            self.initialized = true;
            self.window_start = ts;
            return None;
        }

        if ts.saturating_sub(self.window_start) >= self.window_us {
            // Compute rate in events per second
            let elapsed_us = ts.saturating_sub(self.window_start) as f64;
            let rate = (self.window_count as f64 / elapsed_us) * 1_000_000.0;

            if rate > self.peak_rate {
                self.peak_rate = rate;
            }
            self.last_rate = rate;

            // Reset window
            self.window_start = ts;
            self.window_count = 0;

            Some(rate)
        } else {
            None
        }
    }

    /// Get the most recently computed rate (events per second).
    pub fn last_rate(&self) -> f64 {
        self.last_rate
    }

    /// Get the peak rate observed since creation/reset.
    pub fn peak_rate(&self) -> f64 {
        self.peak_rate
    }

    /// Get the total event count since creation/reset.
    pub fn total_count(&self) -> u64 {
        self.total_count
    }

    /// Reset all statistics.
    pub fn reset(&mut self) {
        self.initialized = false;
        self.window_start = 0;
        self.window_count = 0;
        self.total_count = 0;
        self.peak_rate = 0.0;
        self.last_rate = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(ts: i64) -> Event {
        Event::new(0, 0, ts, 1)
    }

    #[test]
    fn test_initial_state() {
        let stats = EventRateStats::new(1_000_000);
        assert_eq!(stats.total_count(), 0);
        assert_eq!(stats.last_rate(), 0.0);
        assert_eq!(stats.peak_rate(), 0.0);
    }

    #[test]
    fn test_count_increments() {
        let mut stats = EventRateStats::new(1_000_000);
        stats.record(&ev(100_000));
        stats.record(&ev(200_000));
        stats.record(&ev(300_000));
        assert_eq!(stats.total_count(), 3);
    }

    #[test]
    fn test_rate_computed_at_window() {
        let mut stats = EventRateStats::new(1_000_000); // 1 second window
        // 100 events in 1 second
        for i in 0..100 {
            stats.record(&ev(i * 10_000));
        }
        // Trigger window boundary
        let rate = stats.record(&ev(1_000_000));
        assert!(rate.is_some());
        let r = rate.unwrap();
        // ~101 events in 1 second = ~101 events/sec
        assert!(r > 90.0 && r < 120.0, "rate should be ~101 ev/s, got {}", r);
    }

    #[test]
    fn test_peak_rate_tracks_max() {
        let mut stats = EventRateStats::new(100_000); // 100ms window

        // Fast burst: 50 events in 100ms = 500 ev/s
        for i in 0..50 {
            stats.record(&ev(i * 2000));
        }
        stats.record(&ev(100_000));

        let peak_after_burst = stats.peak_rate();
        assert!(peak_after_burst > 400.0);

        // Slow period: 5 events in 100ms = 50 ev/s
        for i in 0..5 {
            stats.record(&ev(100_000 + i * 20_000));
        }
        stats.record(&ev(200_000));

        // Peak should still reflect the burst
        assert!(stats.peak_rate() >= peak_after_burst);
    }

    #[test]
    fn test_reset() {
        let mut stats = EventRateStats::new(1_000_000);
        stats.record(&ev(100_000));
        stats.record(&ev(200_000));
        stats.reset();
        assert_eq!(stats.total_count(), 0);
        assert_eq!(stats.peak_rate(), 0.0);
    }

    #[test]
    fn test_no_rate_before_window() {
        let mut stats = EventRateStats::new(1_000_000);
        for i in 0..10 {
            let rate = stats.record(&ev(i * 10_000));
            if i == 0 {
                assert!(rate.is_none());
            }
        }
    }
}
