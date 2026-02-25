use crate::event::Event;

/// Per-pixel frequency estimator.
///
/// Estimates the dominant frequency at each pixel by tracking inter-event
/// intervals. Useful for vibration analysis, flickering detection, and
/// periodic motion characterization.
///
/// For each pixel, maintains a circular buffer of the last N event timestamps.
/// The estimated frequency is computed from the median inter-event interval.
pub struct FrequencyEstimator {
    /// Circular buffers of timestamps per pixel. Flat: pixel_idx * history_len + offset
    timestamps: Vec<i64>,
    /// Write index per pixel
    write_idx: Vec<usize>,
    /// Count of events per pixel (saturates at history_len)
    counts: Vec<usize>,
    width: usize,
    height: usize,
    history_len: usize,
}

impl FrequencyEstimator {
    /// Create a new frequency estimator.
    ///
    /// - `history_len`: Number of recent timestamps to track per pixel (e.g. 8).
    ///   Must be at least 3 to compute meaningful intervals.
    pub fn new(width: u32, height: u32, history_len: u32) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(history_len >= 3, "history_len must be at least 3");
        let w = width as usize;
        let h = height as usize;
        let pixels = w.checked_mul(h).expect("dimension overflow");
        let hl = history_len as usize;
        let total = pixels.checked_mul(hl).expect("pixels*history overflow");
        Self {
            timestamps: vec![0; total],
            write_idx: vec![0; pixels],
            counts: vec![0; pixels],
            width: w,
            height: h,
            history_len: hl,
        }
    }

    /// Record an event and update the timestamp history for its pixel.
    pub fn record(&mut self, event: &Event) {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return;
        }
        let pixel = y * self.width + x;
        let base = pixel * self.history_len;
        let wi = self.write_idx[pixel];
        self.timestamps[base + wi] = event.timestamp;
        self.write_idx[pixel] = (wi + 1) % self.history_len;
        if self.counts[pixel] < self.history_len {
            self.counts[pixel] += 1;
        }
    }

    /// Estimate the frequency at a pixel in Hz.
    ///
    /// Returns `None` if there are fewer than 3 events recorded at this pixel
    /// (need at least 2 intervals for a meaningful estimate).
    pub fn estimate_frequency(&self, x: u16, y: u16) -> Option<f64> {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height {
            return None;
        }
        let pixel = y * self.width + x;
        let count = self.counts[pixel];
        if count < 3 {
            return None;
        }

        let base = pixel * self.history_len;

        // Collect the actual timestamps stored (up to count entries)
        let mut ts_buf: Vec<i64> = Vec::with_capacity(count);
        for i in 0..count {
            ts_buf.push(self.timestamps[base + i]);
        }
        ts_buf.sort_unstable();

        // Compute inter-event intervals (u64 to avoid truncation from abs_diff)
        let mut intervals: Vec<u64> = Vec::with_capacity(count - 1);
        for i in 1..ts_buf.len() {
            let dt = ts_buf[i].abs_diff(ts_buf[i - 1]);
            if dt > 0 {
                intervals.push(dt);
            }
        }

        if intervals.is_empty() {
            return None;
        }

        // Median interval (compute via f64 to avoid u64 addition overflow)
        intervals.sort_unstable();
        let median_us = if intervals.len().is_multiple_of(2) {
            let mid = intervals.len() / 2;
            intervals[mid - 1] as f64 / 2.0 + intervals[mid] as f64 / 2.0
        } else {
            intervals[intervals.len() / 2] as f64
        };

        if median_us <= 0.0 {
            return None;
        }

        // Frequency = 1 / period (convert from microseconds)
        Some(1_000_000.0 / median_us)
    }

    /// Get the number of recorded events at a pixel.
    pub fn event_count(&self, x: u16, y: u16) -> usize {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height {
            return 0;
        }
        self.counts[y * self.width + x]
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.timestamps.fill(0);
        self.write_idx.fill(0);
        self.counts.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let fe = FrequencyEstimator::new(4, 4, 8);
        assert_eq!(fe.event_count(0, 0), 0);
        assert!(fe.estimate_frequency(0, 0).is_none());
    }

    #[test]
    fn test_too_few_events() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        fe.record(&Event::new(0, 0, 100_000, 1));
        fe.record(&Event::new(0, 0, 200_000, 1));
        assert_eq!(fe.event_count(0, 0), 2);
        assert!(fe.estimate_frequency(0, 0).is_none());
    }

    #[test]
    fn test_periodic_100hz() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        // 100 Hz = 10000 us period
        for i in 0..8 {
            fe.record(&Event::new(0, 0, i * 10_000, 1));
        }
        let freq = fe.estimate_frequency(0, 0).unwrap();
        assert!(
            (freq - 100.0).abs() < 5.0,
            "expected ~100 Hz, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_periodic_1000hz() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        // 1000 Hz = 1000 us period
        for i in 0..8 {
            fe.record(&Event::new(1, 1, i * 1000, 1));
        }
        let freq = fe.estimate_frequency(1, 1).unwrap();
        assert!(
            (freq - 1000.0).abs() < 50.0,
            "expected ~1000 Hz, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_different_pixels_independent() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        // Pixel (0,0) at 100 Hz
        for i in 0..5 {
            fe.record(&Event::new(0, 0, i * 10_000, 1));
        }
        // Pixel (1,1) at 500 Hz
        for i in 0..5 {
            fe.record(&Event::new(1, 1, i * 2_000, 1));
        }
        let f0 = fe.estimate_frequency(0, 0).unwrap();
        let f1 = fe.estimate_frequency(1, 1).unwrap();
        assert!((f0 - 100.0).abs() < 10.0);
        assert!((f1 - 500.0).abs() < 50.0);
    }

    #[test]
    fn test_out_of_bounds_ignored() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        fe.record(&Event::new(10, 10, 100_000, 1));
        assert_eq!(fe.event_count(10, 10), 0);
    }

    #[test]
    fn test_reset() {
        let mut fe = FrequencyEstimator::new(4, 4, 8);
        for i in 0..5 {
            fe.record(&Event::new(0, 0, i * 10_000, 1));
        }
        fe.reset();
        assert_eq!(fe.event_count(0, 0), 0);
        assert!(fe.estimate_frequency(0, 0).is_none());
    }

    #[test]
    fn test_circular_buffer_overwrites() {
        let mut fe = FrequencyEstimator::new(4, 4, 4);
        // Write 6 events into a buffer of size 4 -> oldest 2 overwritten
        for i in 0..6 {
            fe.record(&Event::new(0, 0, (i + 1) * 10_000, 1));
        }
        assert_eq!(fe.event_count(0, 0), 4);
    }
}
