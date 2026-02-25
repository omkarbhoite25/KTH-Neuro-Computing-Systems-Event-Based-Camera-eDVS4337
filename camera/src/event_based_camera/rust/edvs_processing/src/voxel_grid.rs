use crate::event::Event;

/// Voxel Grid representation (Zhu et al., 2019).
///
/// Discretizes events into a 3D volume (x, y, time_bins). Each event's
/// polarity is distributed across two adjacent time bins using bilinear
/// interpolation. This produces a dense tensor representation suitable
/// as input for learning-based methods (e.g., optical flow networks).
pub struct VoxelGrid {
    grid: Vec<f32>,
    width: usize,
    height: usize,
    num_bins: usize,
    t_start: i64,
    t_end: i64,
    initialized: bool,
}

impl VoxelGrid {
    /// Create a new voxel grid.
    ///
    /// - `num_bins`: Number of temporal bins (e.g. 5).
    pub fn new(width: u32, height: u32, num_bins: u32) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(num_bins >= 2, "num_bins must be at least 2");
        let w = width as usize;
        let h = height as usize;
        let b = num_bins as usize;
        let n = w.checked_mul(h).expect("width*height overflow");
        let total = n.checked_mul(b).expect("dimension*bins overflow");
        Self {
            grid: vec![0.0; total],
            width: w,
            height: h,
            num_bins: b,
            t_start: 0,
            t_end: 0,
            initialized: false,
        }
    }

    /// Set the time window for the voxel grid.
    ///
    /// Events outside [t_start, t_end] are ignored. Must be called before
    /// adding events, and t_end must be > t_start.
    pub fn set_time_window(&mut self, t_start: i64, t_end: i64) {
        assert!(t_end > t_start, "t_end must be > t_start");
        self.t_start = t_start;
        self.t_end = t_end;
        self.initialized = true;
        self.grid.fill(0.0);
    }

    /// Add an event to the voxel grid.
    ///
    /// The event's polarity is distributed across two adjacent temporal bins
    /// using bilinear interpolation.
    pub fn add_event(&mut self, event: &Event) {
        if !self.initialized {
            return;
        }
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return;
        }

        let ts = event.timestamp;
        if ts < self.t_start || ts > self.t_end {
            return;
        }

        let duration = self.t_end.saturating_sub(self.t_start);
        if duration <= 0 {
            return;
        }

        // Normalize timestamp to [0, num_bins - 1]
        let t_norm = ts.saturating_sub(self.t_start) as f64 / duration as f64 * (self.num_bins - 1) as f64;
        let t_floor = t_norm.floor() as usize;
        let t_ceil = t_floor + 1;
        let weight_ceil = t_norm - t_floor as f64;
        let weight_floor = 1.0 - weight_ceil;

        let polarity = if event.polarity > 0 { 1.0f32 } else { -1.0f32 };
        let pixel_stride = self.width * self.height;

        // Distribute to floor bin
        if t_floor < self.num_bins {
            let idx = t_floor * pixel_stride + y * self.width + x;
            self.grid[idx] += polarity * weight_floor as f32;
        }

        // Distribute to ceil bin
        if t_ceil < self.num_bins {
            let idx = t_ceil * pixel_stride + y * self.width + x;
            self.grid[idx] += polarity * weight_ceil as f32;
        }
    }

    /// Get the voxel grid as a flat slice.
    ///
    /// Layout: `[bin0_pixels..., bin1_pixels..., ...]` where each bin's
    /// pixels are in row-major order (y * width + x).
    pub fn get_grid(&self) -> &[f32] {
        &self.grid
    }

    /// Get the value at a specific (x, y, bin).
    pub fn get_value(&self, x: u16, y: u16, bin: usize) -> f32 {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height || bin >= self.num_bins {
            return 0.0;
        }
        let pixel_stride = self.width * self.height;
        self.grid[bin * pixel_stride + y * self.width + x]
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn num_bins(&self) -> usize {
        self.num_bins
    }

    /// Reset the voxel grid (clears all values, keeps time window).
    pub fn reset(&mut self) {
        self.grid.fill(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_grid_zeros() {
        let vg = VoxelGrid::new(4, 4, 5);
        assert!(vg.get_grid().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_event_at_start() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        // Event at t=0 should go entirely into bin 0
        vg.add_event(&Event::new(1, 1, 0, 1));
        assert!((vg.get_value(1, 1, 0) - 1.0).abs() < 0.01);
        assert_eq!(vg.get_value(1, 1, 1), 0.0);
    }

    #[test]
    fn test_event_at_end() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        // Event at t_end should go entirely into last bin
        vg.add_event(&Event::new(1, 1, 1_000_000, 1));
        assert!((vg.get_value(1, 1, 4) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_event_midpoint_splits() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        // Event at midpoint between bin 1 and bin 2 (t_norm = 2.0 for t=500000)
        // t_norm = 500000/1000000 * 4 = 2.0 -> all weight in bin 2
        vg.add_event(&Event::new(0, 0, 500_000, 1));
        assert!((vg.get_value(0, 0, 2) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_interpolation_between_bins() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        // t_norm = 125000/1000000 * 4 = 0.5 -> split between bin 0 and bin 1
        vg.add_event(&Event::new(0, 0, 125_000, 1));
        let v0 = vg.get_value(0, 0, 0);
        let v1 = vg.get_value(0, 0, 1);
        assert!((v0 - 0.5).abs() < 0.01);
        assert!((v1 - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_negative_polarity() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        vg.add_event(&Event::new(0, 0, 0, -1));
        assert!((vg.get_value(0, 0, 0) - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_out_of_bounds_ignored() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        vg.add_event(&Event::new(10, 10, 500_000, 1));
        assert!(vg.get_grid().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_out_of_time_window_ignored() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(100_000, 200_000);
        vg.add_event(&Event::new(0, 0, 50_000, 1)); // before window
        vg.add_event(&Event::new(0, 0, 300_000, 1)); // after window
        assert!(vg.get_grid().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_accumulation() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        // Two ON events at same pixel and time
        vg.add_event(&Event::new(0, 0, 0, 1));
        vg.add_event(&Event::new(0, 0, 0, 1));
        assert!((vg.get_value(0, 0, 0) - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_reset() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        vg.set_time_window(0, 1_000_000);
        vg.add_event(&Event::new(0, 0, 0, 1));
        vg.reset();
        assert!(vg.get_grid().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_event_before_init_ignored() {
        let mut vg = VoxelGrid::new(4, 4, 5);
        // No set_time_window called
        vg.add_event(&Event::new(0, 0, 100_000, 1));
        assert!(vg.get_grid().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_grid_dimensions() {
        let vg = VoxelGrid::new(8, 6, 3);
        assert_eq!(vg.width(), 8);
        assert_eq!(vg.height(), 6);
        assert_eq!(vg.num_bins(), 3);
        assert_eq!(vg.get_grid().len(), 8 * 6 * 3);
    }
}
