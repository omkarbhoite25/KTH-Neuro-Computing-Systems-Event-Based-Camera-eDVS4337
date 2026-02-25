use crate::event::Event;

/// Hot pixel detector and filter.
///
/// Tracks per-pixel event counts within a sliding time window.
/// Pixels exceeding `max_rate` events per window are flagged as hot
/// and their events are rejected.
pub struct HotPixelFilter {
    event_counts: Vec<u32>,
    width: usize,
    height: usize,
    window_start: i64,
    window_us: i64,
    max_rate: u32,
    hot_pixels: Vec<bool>,
}

impl HotPixelFilter {
    pub fn new(width: u32, height: u32, window_us: i64, max_rate: u32) -> Self {
        let w = width as usize;
        let h = height as usize;
        let n = w * h;
        Self {
            event_counts: vec![0; n],
            width: w,
            height: h,
            window_start: 0,
            window_us,
            max_rate,
            hot_pixels: vec![false; n],
        }
    }

    /// Returns true if the event passes (pixel is not hot).
    pub fn filter(&mut self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;

        if x >= self.width || y >= self.height {
            return false;
        }

        // Check if we need to advance the window
        if self.window_start == 0 {
            self.window_start = ts;
        }

        if ts - self.window_start >= self.window_us {
            // Evaluate which pixels are hot, then reset counts
            for i in 0..self.event_counts.len() {
                self.hot_pixels[i] = self.event_counts[i] > self.max_rate;
                self.event_counts[i] = 0;
            }
            self.window_start = ts;
        }

        let idx = y * self.width + x;
        self.event_counts[idx] += 1;

        // Reject if this pixel was hot in the previous window
        !self.hot_pixels[idx]
    }

    /// Returns the number of currently flagged hot pixels.
    pub fn hot_pixel_count(&self) -> usize {
        self.hot_pixels.iter().filter(|&&h| h).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_pixel_passes() {
        let mut filter = HotPixelFilter::new(128, 128, 1_000_000, 100);
        let ev = Event::new(10, 10, 500_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_hot_pixel_detected() {
        let mut filter = HotPixelFilter::new(128, 128, 1_000_000, 5);

        // Flood pixel (10,10) with events in the first window
        for i in 0..10 {
            let ev = Event::new(10, 10, 100_000 + i * 1000, 1);
            filter.filter(&ev);
        }

        // Advance to next window — triggers evaluation
        let trigger = Event::new(50, 50, 1_200_000, 1);
        filter.filter(&trigger);

        // Now (10,10) should be flagged as hot
        let ev_hot = Event::new(10, 10, 1_300_000, 1);
        assert!(
            !filter.filter(&ev_hot),
            "hot pixel event should be rejected"
        );

        // Other pixels should still pass
        let ev_ok = Event::new(20, 20, 1_400_000, 1);
        assert!(filter.filter(&ev_ok));
    }

    #[test]
    fn test_hot_pixel_count() {
        let mut filter = HotPixelFilter::new(4, 4, 1_000_000, 2);

        // Flood two pixels
        for i in 0..5 {
            filter.filter(&Event::new(0, 0, 100_000 + i * 1000, 1));
            filter.filter(&Event::new(1, 1, 100_000 + i * 1000, 1));
        }

        // Advance window
        filter.filter(&Event::new(3, 3, 1_200_000, 1));

        assert_eq!(filter.hot_pixel_count(), 2);
    }
}
