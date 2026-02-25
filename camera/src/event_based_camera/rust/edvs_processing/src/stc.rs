use crate::event::Event;

/// Spatio-Temporal Contrast (STC) filter.
///
/// More selective than basic temporal denoising. An event passes only if a
/// spatial neighbor had a recent event with the SAME polarity. This preserves
/// edges and motion patterns while rejecting uncorrelated noise.
///
/// The filter maintains both per-pixel timestamps and per-pixel polarities
/// to enable polarity-aware neighbor checking.
pub struct StcFilter {
    last_timestamp: Vec<i64>,
    last_polarity: Vec<i8>,
    width: usize,
    height: usize,
    threshold_us: i64,
}

impl StcFilter {
    pub fn new(width: u32, height: u32, threshold_us: i64) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(threshold_us > 0, "threshold_us must be positive");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            last_timestamp: vec![0; n],
            last_polarity: vec![0; n],
            width: w,
            height: h,
            threshold_us,
        }
    }

    /// Returns true if the event passes the STC filter.
    ///
    /// An event passes if at least one 8-connected neighbor had a recent event
    /// (within `threshold_us`) with the same polarity.
    pub fn filter(&mut self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;
        let pol = event.polarity;

        if x >= self.width || y >= self.height {
            return false;
        }

        let idx = y * self.width + x;

        // Update this pixel
        self.last_timestamp[idx] = ts;
        self.last_polarity[idx] = pol;

        // Check 8-connected neighbors for same-polarity recent event
        let x_min = x.saturating_sub(1);
        let x_max = (x + 1).min(self.width - 1);
        let y_min = y.saturating_sub(1);
        let y_max = (y + 1).min(self.height - 1);

        for ny in y_min..=y_max {
            for nx in x_min..=x_max {
                if nx == x && ny == y {
                    continue;
                }
                let n_idx = ny * self.width + nx;
                let n_ts = self.last_timestamp[n_idx];
                let n_pol = self.last_polarity[n_idx];

                if n_ts > 0
                    && n_pol == pol
                    && ts.abs_diff(n_ts) <= self.threshold_us as u64
                {
                    return true;
                }
            }
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isolated_event_rejected() {
        let mut filter = StcFilter::new(128, 128, 5000);
        let ev = Event::new(64, 64, 100_000, 1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_same_polarity_neighbor_passes() {
        let mut filter = StcFilter::new(128, 128, 5000);
        // First event at (10, 10) with ON polarity
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // Same polarity neighbor at (11, 10), within threshold
        let ev = Event::new(11, 10, 102_000, 1);
        assert!(filter.filter(&ev), "same-polarity recent neighbor should pass");
    }

    #[test]
    fn test_opposite_polarity_neighbor_rejected() {
        let mut filter = StcFilter::new(128, 128, 5000);
        // ON event at (10, 10)
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // OFF event at neighbor (11, 10) — opposite polarity
        let ev = Event::new(11, 10, 102_000, -1);
        assert!(
            !filter.filter(&ev),
            "opposite polarity neighbor should not help"
        );
    }

    #[test]
    fn test_temporally_distant_neighbor_rejected() {
        let mut filter = StcFilter::new(128, 128, 5000);
        filter.filter(&Event::new(10, 10, 100_000, 1));

        // Same polarity but too far in time
        let ev = Event::new(11, 10, 200_000, 1);
        assert!(!filter.filter(&ev), "temporally distant neighbor should fail");
    }

    #[test]
    fn test_out_of_bounds_rejected() {
        let mut filter = StcFilter::new(128, 128, 5000);
        assert!(!filter.filter(&Event::new(200, 200, 100_000, 1)));
    }

    #[test]
    fn test_diagonal_neighbor_passes() {
        let mut filter = StcFilter::new(128, 128, 5000);
        filter.filter(&Event::new(10, 10, 100_000, -1));

        // Diagonal neighbor same polarity
        let ev = Event::new(11, 11, 102_000, -1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_corner_pixel() {
        let mut filter = StcFilter::new(128, 128, 5000);
        filter.filter(&Event::new(1, 0, 100_000, 1));

        let ev = Event::new(0, 0, 101_000, 1);
        assert!(filter.filter(&ev));
    }
}
