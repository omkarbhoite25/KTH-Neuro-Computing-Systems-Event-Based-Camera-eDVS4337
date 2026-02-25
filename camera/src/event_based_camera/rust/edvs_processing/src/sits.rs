use crate::event::Event;

/// Speed-Invariant Time Surface (Sironi et al., 2018).
///
/// Normalizes the standard time surface to remove dependency on motion speed.
/// Each pixel's decay value is normalized by ranking the timestamp within
/// the local neighborhood, producing a representation invariant to the
/// speed of the stimulus.
///
/// The output value at each pixel is its normalized rank (0.0 to 1.0) within
/// the local context, where 1.0 is the most recent event.
pub struct SpeedInvariantTimeSurface {
    timestamps: Vec<i64>,
    width: usize,
    height: usize,
    radius: usize,
}

impl SpeedInvariantTimeSurface {
    /// Create a new speed-invariant time surface.
    ///
    /// - `radius`: Neighborhood radius for normalization (e.g. 2 for a 5x5 patch).
    pub fn new(width: u32, height: u32, radius: u32) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(radius > 0, "radius must be positive");
        assert!(radius <= 128, "radius must be at most 128");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            timestamps: vec![0; n],
            width: w,
            height: h,
            radius: radius as usize,
        }
    }

    /// Update the time surface with a new event.
    pub fn update(&mut self, event: &Event) {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return;
        }
        self.timestamps[y * self.width + x] = event.timestamp;
    }

    /// Get the speed-invariant context at the event's pixel.
    ///
    /// Returns a flattened (2*radius+1)^2 patch where each value is the
    /// normalized rank of that pixel's timestamp in [0.0, 1.0].
    /// Pixels with no events (ts=0) or out-of-bounds get rank 0.0.
    pub fn get_context(&self, x: u16, y: u16) -> Vec<f64> {
        let cx = x as i32;
        let cy = y as i32;
        let r = self.radius as i32;
        let side = (2 * r + 1) as usize;
        let total = side * side;

        // Collect timestamps for all positions in the patch
        let mut ts_values: Vec<i64> = Vec::with_capacity(total);
        for dy in -r..=r {
            for dx in -r..=r {
                let nx = cx + dx;
                let ny = cy + dy;
                if nx < 0 || ny < 0 || nx >= self.width as i32 || ny >= self.height as i32 {
                    ts_values.push(0);
                } else {
                    let idx = ny as usize * self.width + nx as usize;
                    ts_values.push(self.timestamps[idx]);
                }
            }
        }

        // Compute rank normalization: for each value, count how many
        // non-zero values are <= it, then divide by total non-zero count.
        let non_zero_count = ts_values.iter().filter(|&&t| t > 0).count();
        if non_zero_count == 0 {
            return vec![0.0; total];
        }

        ts_values
            .iter()
            .map(|&ts| {
                if ts <= 0 {
                    return 0.0;
                }
                // Count how many non-zero timestamps are <= this one
                let rank = ts_values.iter().filter(|&&t| t > 0 && t <= ts).count();
                rank as f64 / non_zero_count as f64
            })
            .collect()
    }

    /// Get raw timestamp at a pixel.
    pub fn get_timestamp(&self, x: u16, y: u16) -> i64 {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height {
            return 0;
        }
        self.timestamps[y * self.width + x]
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn radius(&self) -> usize {
        self.radius
    }

    /// Reset all timestamps to zero.
    pub fn reset(&mut self) {
        self.timestamps.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_timestamps_zero() {
        let sits = SpeedInvariantTimeSurface::new(4, 4, 1);
        for y in 0..4u16 {
            for x in 0..4u16 {
                assert_eq!(sits.get_timestamp(x, y), 0);
            }
        }
    }

    #[test]
    fn test_update_stores_timestamp() {
        let mut sits = SpeedInvariantTimeSurface::new(8, 8, 2);
        sits.update(&Event::new(3, 3, 500_000, 1));
        assert_eq!(sits.get_timestamp(3, 3), 500_000);
        assert_eq!(sits.get_timestamp(0, 0), 0);
    }

    #[test]
    fn test_empty_context_all_zeros() {
        let sits = SpeedInvariantTimeSurface::new(8, 8, 1);
        let ctx = sits.get_context(4, 4);
        assert_eq!(ctx.len(), 9);
        assert!(ctx.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_single_event_rank_one() {
        let mut sits = SpeedInvariantTimeSurface::new(8, 8, 1);
        sits.update(&Event::new(4, 4, 100_000, 1));
        let ctx = sits.get_context(4, 4);
        // Only one non-zero event -> its rank is 1/1 = 1.0
        assert!((ctx[4] - 1.0).abs() < 0.01); // center
        assert_eq!(ctx[0], 0.0); // no event
    }

    #[test]
    fn test_rank_ordering() {
        let mut sits = SpeedInvariantTimeSurface::new(8, 8, 1);
        // Three events with different timestamps in 3x3 neighborhood of (4,4)
        sits.update(&Event::new(3, 3, 100_000, 1)); // oldest
        sits.update(&Event::new(4, 4, 200_000, 1)); // middle
        sits.update(&Event::new(5, 5, 300_000, 1)); // newest
        let ctx = sits.get_context(4, 4);
        // 3 non-zero events. Ranks: 100k=1/3, 200k=2/3, 300k=3/3
        assert!((ctx[0] - 1.0 / 3.0).abs() < 0.01); // (3,3) oldest
        assert!((ctx[4] - 2.0 / 3.0).abs() < 0.01); // (4,4) middle
        assert!((ctx[8] - 1.0).abs() < 0.01); // (5,5) newest
    }

    #[test]
    fn test_out_of_bounds_update_ignored() {
        let mut sits = SpeedInvariantTimeSurface::new(4, 4, 1);
        sits.update(&Event::new(10, 10, 100_000, 1));
        assert_eq!(sits.get_timestamp(10, 10), 0);
    }

    #[test]
    fn test_reset() {
        let mut sits = SpeedInvariantTimeSurface::new(4, 4, 1);
        sits.update(&Event::new(1, 1, 100_000, 1));
        sits.reset();
        assert_eq!(sits.get_timestamp(1, 1), 0);
    }

    #[test]
    fn test_edge_context() {
        let mut sits = SpeedInvariantTimeSurface::new(4, 4, 1);
        sits.update(&Event::new(0, 0, 100_000, 1));
        let ctx = sits.get_context(0, 0);
        // At corner, most neighbors OOB -> 0.0
        assert_eq!(ctx.len(), 9);
        // Center (0,0) has the only event -> rank 1.0
        assert!((ctx[4] - 1.0).abs() < 0.01);
    }
}
