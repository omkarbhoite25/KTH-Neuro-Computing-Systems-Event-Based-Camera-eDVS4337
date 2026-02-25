use crate::event::Event;

/// Time Surface representation (Lagorce et al., 2017).
///
/// Stores the most recent event timestamp at each pixel location, optionally
/// separated by polarity. The time surface can be queried as a decay-weighted
/// frame where recent events are bright and old events fade to zero.
///
/// This is a fundamental representation in neuromorphic vision used as input
/// for feature extraction, learning-based methods, and optical flow estimation.
pub struct TimeSurface {
    timestamps: Vec<i64>,
    width: usize,
    height: usize,
    decay_tau_us: f64,
}

impl TimeSurface {
    /// Create a new time surface.
    ///
    /// - `decay_tau_us`: Exponential decay time constant in microseconds.
    ///   Larger values produce longer-lasting traces.
    pub fn new(width: u32, height: u32, decay_tau_us: f64) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(decay_tau_us > 0.0, "decay_tau_us must be positive");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            timestamps: vec![0; n],
            width: w,
            height: h,
            decay_tau_us,
        }
    }

    /// Update the time surface with a new event.
    pub fn update(&mut self, event: &Event) {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return;
        }
        let idx = y * self.width + x;
        self.timestamps[idx] = event.timestamp;
    }

    /// Get the time surface as a decay-weighted u8 frame at the given reference time.
    ///
    /// Each pixel value is `255 * exp(-(t_ref - t_pixel) / tau)`, where `t_pixel`
    /// is the most recent event timestamp at that pixel. Pixels with no events are 0.
    pub fn get_frame_at(&self, t_ref: i64) -> Vec<u8> {
        self.timestamps
            .iter()
            .map(|&ts| {
                if ts <= 0 {
                    return 0;
                }
                let dt = t_ref.saturating_sub(ts) as f64;
                if dt < 0.0 {
                    return 255;
                }
                let val = 255.0 * (-dt / self.decay_tau_us).exp();
                val.clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    /// Get a local time surface context around a pixel at the given reference time.
    /// Returns a flattened (2*radius+1) x (2*radius+1) patch of decay values [0.0, 1.0].
    pub fn get_local_context(&self, x: u16, y: u16, radius: u16, t_ref: i64) -> Vec<f64> {
        let cx = x as i32;
        let cy = y as i32;
        let r = radius as i32;
        let side = (2 * r + 1) as usize;
        let mut patch = Vec::with_capacity(side * side);

        for dy in -r..=r {
            for dx in -r..=r {
                let nx = cx + dx;
                let ny = cy + dy;
                if nx < 0 || ny < 0 || nx >= self.width as i32 || ny >= self.height as i32 {
                    patch.push(0.0);
                    continue;
                }
                let idx = ny as usize * self.width + nx as usize;
                let ts = self.timestamps[idx];
                if ts <= 0 {
                    patch.push(0.0);
                } else {
                    let dt = t_ref.saturating_sub(ts) as f64;
                    let val = (-dt / self.decay_tau_us).exp();
                    patch.push(val.clamp(0.0, 1.0));
                }
            }
        }
        patch
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
        let ts = TimeSurface::new(4, 4, 100_000.0);
        for y in 0..4u16 {
            for x in 0..4u16 {
                assert_eq!(ts.get_timestamp(x, y), 0);
            }
        }
    }

    #[test]
    fn test_update_stores_timestamp() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(1, 2, 500_000, 1));
        assert_eq!(ts.get_timestamp(1, 2), 500_000);
        assert_eq!(ts.get_timestamp(0, 0), 0);
    }

    #[test]
    fn test_frame_recent_event_bright() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(0, 0, 100_000, 1));
        let frame = ts.get_frame_at(100_000);
        // dt = 0, exp(0) = 1.0 -> 255
        assert_eq!(frame[0], 255);
    }

    #[test]
    fn test_frame_old_event_fades() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(0, 0, 100_000, 1));
        let frame = ts.get_frame_at(1_000_000);
        // dt = 900000, tau = 100000, exp(-9) ~ 0.000123 -> ~0
        assert!(frame[0] < 2);
    }

    #[test]
    fn test_frame_no_event_zero() {
        let ts = TimeSurface::new(4, 4, 100_000.0);
        let frame = ts.get_frame_at(100_000);
        assert!(frame.iter().all(|&v| v == 0));
    }

    #[test]
    fn test_local_context_center() {
        let mut ts = TimeSurface::new(8, 8, 100_000.0);
        ts.update(&Event::new(4, 4, 100_000, 1));
        let ctx = ts.get_local_context(4, 4, 1, 100_000);
        // 3x3 patch, center should be ~1.0, others 0.0
        assert_eq!(ctx.len(), 9);
        assert!((ctx[4] - 1.0).abs() < 0.01); // center
        assert_eq!(ctx[0], 0.0); // no event at neighbor
    }

    #[test]
    fn test_local_context_at_edge() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(0, 0, 100_000, 1));
        let ctx = ts.get_local_context(0, 0, 1, 100_000);
        // At corner (0,0) with radius 1, patch goes from (-1,-1) to (1,1):
        // index 0=(-1,-1) OOB, index 1=(0,-1) OOB, index 2=(1,-1) OOB
        // index 3=(-1,0)  OOB, index 4=(0,0) center, index 5=(1,0)
        // index 6=(-1,1)  OOB, index 7=(0,1),        index 8=(1,1)
        assert_eq!(ctx.len(), 9);
        assert!((ctx[4] - 1.0).abs() < 0.01); // center pixel (0,0) -> 1.0
        assert_eq!(ctx[0], 0.0); // out-of-bounds neighbor
    }

    #[test]
    fn test_out_of_bounds_update_ignored() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(10, 10, 100_000, 1));
        assert_eq!(ts.get_timestamp(10, 10), 0);
    }

    #[test]
    fn test_reset() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(1, 1, 100_000, 1));
        ts.reset();
        assert_eq!(ts.get_timestamp(1, 1), 0);
    }

    #[test]
    fn test_overwrite_newer_timestamp() {
        let mut ts = TimeSurface::new(4, 4, 100_000.0);
        ts.update(&Event::new(1, 1, 100_000, 1));
        ts.update(&Event::new(1, 1, 200_000, -1));
        assert_eq!(ts.get_timestamp(1, 1), 200_000);
    }
}
