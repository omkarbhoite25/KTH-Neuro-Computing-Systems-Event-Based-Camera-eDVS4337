use crate::event::Event;

/// Anti-flicker filter for fluorescent lighting artifacts.
///
/// Fluorescent lights operate at mains frequency (50Hz or 60Hz), producing
/// periodic brightness variations that generate spurious events. This filter
/// tracks per-pixel event intervals and rejects events whose timing matches
/// the expected flicker period within a configurable tolerance.
///
/// Detection: For each pixel, the filter records the last two event timestamps.
/// If the interval between consecutive events falls within
/// `[period - tolerance, period + tolerance]`, the event is classified as
/// flicker and rejected.
pub struct AntiFlickerFilter {
    prev_timestamp: Vec<i64>,
    prev_prev_timestamp: Vec<i64>,
    width: usize,
    height: usize,
    period_us: i64,
    tolerance_us: i64,
}

impl AntiFlickerFilter {
    /// Create a new anti-flicker filter.
    ///
    /// - `period_us`: Expected flicker period in microseconds.
    ///   For 50Hz: 10000us (half-period) or 20000us (full period).
    ///   For 60Hz: 8333us (half-period) or 16667us (full period).
    /// - `tolerance_us`: Allowed deviation from the expected period.
    pub fn new(width: u32, height: u32, period_us: i64, tolerance_us: i64) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(period_us > 0, "period_us must be positive");
        assert!(tolerance_us >= 0, "tolerance_us must be non-negative");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            prev_timestamp: vec![0; n],
            prev_prev_timestamp: vec![0; n],
            width: w,
            height: h,
            period_us,
            tolerance_us,
        }
    }

    /// Returns true if the event passes (is NOT flicker).
    pub fn filter(&mut self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;

        if x >= self.width || y >= self.height {
            return false;
        }

        let idx = y * self.width + x;
        let prev = self.prev_timestamp[idx];

        // Shift timestamps
        self.prev_prev_timestamp[idx] = prev;
        self.prev_timestamp[idx] = ts;

        if prev <= 0 {
            return true; // Not enough history
        }

        let interval = ts.saturating_sub(prev);
        let deviation = (interval - self.period_us).abs();

        if deviation <= self.tolerance_us {
            // Matches flicker period — reject
            return false;
        }

        // Also check against double period (full cycle)
        let double_period = self.period_us.saturating_mul(2);
        let deviation_double = (interval - double_period).abs();
        if deviation_double <= self.tolerance_us {
            return false;
        }

        true
    }

    pub fn period_us(&self) -> i64 {
        self.period_us
    }

    pub fn tolerance_us(&self) -> i64 {
        self.tolerance_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_event_passes() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        let ev = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_non_periodic_passes() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // 5000us later — not near 10000us period
        let ev = Event::new(10, 10, 105_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_periodic_at_flicker_freq_rejected() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // Exactly 10000us later — matches 50Hz half-period
        let ev = Event::new(10, 10, 110_000, 1);
        assert!(!filter.filter(&ev), "event matching flicker period should be rejected");
    }

    #[test]
    fn test_periodic_within_tolerance_rejected() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // 10200us later — within 500us tolerance of 10000us
        let ev = Event::new(10, 10, 110_200, 1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_periodic_outside_tolerance_passes() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // 11000us later — outside tolerance
        let ev = Event::new(10, 10, 111_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_double_period_rejected() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // 20000us (double period) — also matches
        let ev = Event::new(10, 10, 120_000, 1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_different_pixels_independent() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        filter.filter(&Event::new(10, 10, 100_000, 1));
        filter.filter(&Event::new(10, 10, 110_000, 1)); // would be flicker

        // Different pixel, first event — should pass
        let ev = Event::new(20, 20, 110_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_out_of_bounds_rejected() {
        let mut filter = AntiFlickerFilter::new(128, 128, 10_000, 500);
        assert!(!filter.filter(&Event::new(200, 200, 100_000, 1)));
    }
}
