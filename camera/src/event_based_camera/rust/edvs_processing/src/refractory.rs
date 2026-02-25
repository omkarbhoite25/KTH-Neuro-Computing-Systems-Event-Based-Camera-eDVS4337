use crate::event::Event;

/// Per-pixel refractory period filter.
///
/// After a pixel fires an event, all subsequent events at that pixel are
/// suppressed for `refractory_period_us` microseconds. This mimics the
/// biological refractory period of neurons and reduces redundant events
/// from rapidly toggling pixels.
pub struct RefractoryFilter {
    last_timestamp: Vec<i64>,
    width: usize,
    height: usize,
    refractory_period_us: i64,
}

impl RefractoryFilter {
    pub fn new(width: u32, height: u32, refractory_period_us: i64) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(
            refractory_period_us > 0,
            "refractory_period_us must be positive"
        );
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            last_timestamp: vec![0; n],
            width: w,
            height: h,
            refractory_period_us,
        }
    }

    /// Returns true if the event passes (enough time has elapsed since the
    /// last event at this pixel). Updates the pixel's timestamp on pass.
    pub fn filter(&mut self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;

        if x >= self.width || y >= self.height {
            return false;
        }

        let idx = y * self.width + x;
        let last = self.last_timestamp[idx];

        if last > 0 && ts.saturating_sub(last) < self.refractory_period_us {
            return false;
        }

        self.last_timestamp[idx] = ts;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_event_passes() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev), "first event should always pass");
    }

    #[test]
    fn test_event_within_refractory_rejected() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev1 = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev1));

        // 500us later — within refractory period
        let ev2 = Event::new(10, 10, 100_500, -1);
        assert!(
            !filter.filter(&ev2),
            "event within refractory period should be rejected"
        );
    }

    #[test]
    fn test_event_after_refractory_passes() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev1 = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev1));

        // 1500us later — past refractory period
        let ev2 = Event::new(10, 10, 101_500, -1);
        assert!(
            filter.filter(&ev2),
            "event after refractory period should pass"
        );
    }

    #[test]
    fn test_different_pixels_independent() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev1 = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev1));

        // Different pixel at same time — should pass
        let ev2 = Event::new(20, 20, 100_000, 1);
        assert!(
            filter.filter(&ev2),
            "different pixel should have independent refractory"
        );
    }

    #[test]
    fn test_out_of_bounds_rejected() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev = Event::new(200, 200, 100_000, 1);
        assert!(!filter.filter(&ev), "out-of-bounds should be rejected");
    }

    #[test]
    fn test_exact_boundary_passes() {
        let mut filter = RefractoryFilter::new(128, 128, 1000);
        let ev1 = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev1));

        // Exactly at refractory boundary — should pass (< not <=)
        let ev2 = Event::new(10, 10, 101_000, -1);
        assert!(
            filter.filter(&ev2),
            "event exactly at refractory boundary should pass"
        );
    }
}
