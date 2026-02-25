use crate::event::Event;

/// Nearest-neighbor temporal denoising filter.
///
/// An event is considered valid only if at least one of its 8 spatial neighbors
/// (in a 3x3 window) had an event within `threshold_us` microseconds.
/// This removes isolated noise events that have no temporal correlation.
pub struct TemporalFilter {
    last_timestamp: Vec<i64>,
    width: usize,
    height: usize,
    threshold_us: i64,
}

impl TemporalFilter {
    pub fn new(width: u32, height: u32, threshold_us: i64) -> Self {
        let w = width as usize;
        let h = height as usize;
        Self {
            last_timestamp: vec![0; w * h],
            width: w,
            height: h,
            threshold_us,
        }
    }

    /// Returns true if the event passes the filter (has a recent neighbor).
    /// Always updates the per-pixel timestamp regardless of filter result.
    pub fn filter(&mut self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;

        if x >= self.width || y >= self.height {
            return false;
        }

        // Update this pixel's timestamp
        let idx = y * self.width + x;
        self.last_timestamp[idx] = ts;

        // Check 8-connected neighbors
        let x_min = x.saturating_sub(1);
        let x_max = (x + 1).min(self.width - 1);
        let y_min = y.saturating_sub(1);
        let y_max = (y + 1).min(self.height - 1);

        for ny in y_min..=y_max {
            for nx in x_min..=x_max {
                if nx == x && ny == y {
                    continue;
                }
                let neighbor_idx = ny * self.width + nx;
                let neighbor_ts = self.last_timestamp[neighbor_idx];
                if neighbor_ts > 0 && (ts - neighbor_ts).abs() <= self.threshold_us {
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
        let mut filter = TemporalFilter::new(128, 128, 5000);
        let ev = Event::new(64, 64, 100000, 1);
        assert!(!filter.filter(&ev), "isolated event should be rejected");
    }

    #[test]
    fn test_neighbor_event_accepted() {
        let mut filter = TemporalFilter::new(128, 128, 5000);

        // First event at (10, 10)
        let ev1 = Event::new(10, 10, 100000, 1);
        filter.filter(&ev1); // rejected (no neighbor), but timestamp recorded

        // Second event at neighbor (11, 10), within threshold
        let ev2 = Event::new(11, 10, 102000, -1);
        assert!(filter.filter(&ev2), "event near recent neighbor should pass");
    }

    #[test]
    fn test_distant_neighbor_rejected() {
        let mut filter = TemporalFilter::new(128, 128, 5000);

        let ev1 = Event::new(10, 10, 100000, 1);
        filter.filter(&ev1);

        // Event at neighbor but too far in time
        let ev2 = Event::new(11, 10, 200000, -1);
        assert!(!filter.filter(&ev2), "temporally distant neighbor should fail");
    }

    #[test]
    fn test_out_of_bounds_rejected() {
        let mut filter = TemporalFilter::new(128, 128, 5000);
        let ev = Event::new(200, 200, 100000, 1);
        assert!(!filter.filter(&ev), "out-of-bounds event should be rejected");
    }

    #[test]
    fn test_corner_event() {
        let mut filter = TemporalFilter::new(128, 128, 5000);

        // Event at corner (0,0) neighbor (1,0)
        let ev1 = Event::new(1, 0, 100000, 1);
        filter.filter(&ev1);

        let ev2 = Event::new(0, 0, 101000, 1);
        assert!(filter.filter(&ev2), "corner event near neighbor should pass");
    }
}
