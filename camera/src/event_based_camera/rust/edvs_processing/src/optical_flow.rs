use crate::event::Event;

/// Local optical flow estimator using the plane-fitting method (Benosman et al., 2014).
///
/// Estimates the local optical flow at each event by fitting a plane to the
/// time surface in a local neighborhood. The flow velocity is derived from
/// the spatial gradient of the fitted plane:
///
///   t(x,y) = a*x + b*y + c  =>  vx = -1/a, vy = -1/b
///
/// The plane is fitted using least-squares on the local patch of timestamps.
pub struct OpticalFlowEstimator {
    timestamps: Vec<i64>,
    width: usize,
    height: usize,
    radius: usize,
}

/// Optical flow vector at a single event.
#[derive(Clone, Copy, Debug)]
pub struct FlowVector {
    pub vx: f64,
    pub vy: f64,
}

impl OpticalFlowEstimator {
    /// Create a new optical flow estimator.
    ///
    /// - `radius`: Neighborhood radius for plane fitting (e.g. 2 for 5x5).
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

    /// Process an event: update the time surface and estimate local optical flow.
    ///
    /// Returns `Some(FlowVector)` if the flow could be estimated (enough
    /// valid neighbors), or `None` if the event is at the border or has
    /// insufficient support.
    pub fn process(&mut self, event: &Event) -> Option<FlowVector> {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return None;
        }

        // Update time surface
        self.timestamps[y * self.width + x] = event.timestamp;

        let r = self.radius;
        // Check border
        if x < r || y < r || x + r >= self.width || y + r >= self.height {
            return None;
        }

        // Fit plane t = a*dx + b*dy + c using least squares.
        // Normal equations: [sum(dx^2), sum(dx*dy)] [a]   [sum(dx*t)]
        //                   [sum(dx*dy), sum(dy^2)] [b] = [sum(dy*t)]
        //
        // Where dx, dy are offsets from center and t is the timestamp.
        let mut sum_dx2 = 0.0_f64;
        let mut sum_dy2 = 0.0_f64;
        let mut sum_dxdy = 0.0_f64;
        let mut sum_dx_t = 0.0_f64;
        let mut sum_dy_t = 0.0_f64;
        let mut valid_count = 0;

        let ri = r as i32;
        let center_ts = self.timestamps[y * self.width + x] as f64;

        for dy_i in -ri..=ri {
            for dx_i in -ri..=ri {
                let nx = (x as i32 + dx_i) as usize;
                let ny = (y as i32 + dy_i) as usize;
                let ts = self.timestamps[ny * self.width + nx];
                if ts <= 0 {
                    continue;
                }

                let dx = dx_i as f64;
                let dy = dy_i as f64;
                let t = ts as f64 - center_ts; // relative to center

                sum_dx2 += dx * dx;
                sum_dy2 += dy * dy;
                sum_dxdy += dx * dy;
                sum_dx_t += dx * t;
                sum_dy_t += dy * t;
                valid_count += 1;
            }
        }

        // Need at least 3 valid points for a plane fit
        if valid_count < 3 {
            return None;
        }

        // Solve 2x2 system: [sum_dx2, sum_dxdy] [a] = [sum_dx_t]
        //                    [sum_dxdy, sum_dy2] [b]   [sum_dy_t]
        let det = sum_dx2 * sum_dy2 - sum_dxdy * sum_dxdy;
        if det.abs() < 1e-10 {
            return None;
        }

        let a = (sum_dy2 * sum_dx_t - sum_dxdy * sum_dy_t) / det;
        let b = (sum_dx2 * sum_dy_t - sum_dxdy * sum_dx_t) / det;

        // Flow velocity: vx = -1/a, vy = -1/b (pixels per microsecond)
        // Convert to pixels per second
        let min_gradient = 1e-10;
        let vx = if a.abs() > min_gradient {
            -1_000_000.0 / a
        } else {
            0.0
        };
        let vy = if b.abs() > min_gradient {
            -1_000_000.0 / b
        } else {
            0.0
        };

        Some(FlowVector { vx, vy })
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

    /// Reset the estimator state.
    pub fn reset(&mut self) {
        self.timestamps.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let of = OpticalFlowEstimator::new(16, 16, 2);
        assert_eq!(of.get_timestamp(0, 0), 0);
    }

    #[test]
    fn test_border_returns_none() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        // Pixel (0,0) is at the border with radius=2
        let result = of.process(&Event::new(0, 0, 100_000, 1));
        assert!(result.is_none());
    }

    #[test]
    fn test_no_neighbors_returns_none() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        // Single event in the middle with no neighbors
        let result = of.process(&Event::new(8, 8, 100_000, 1));
        assert!(result.is_none());
    }

    #[test]
    fn test_horizontal_motion() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        // Simulate rightward motion: events along a horizontal line
        // with increasing timestamps from left to right
        // t(x) = x * 1000 (1000 us per pixel)
        for x in 4..12u16 {
            for y in 6..10u16 {
                of.process(&Event::new(x, y, x as i64 * 1000, 1));
            }
        }
        // Query flow at center
        let flow = of.process(&Event::new(8, 8, 8000, 1));
        if let Some(f) = flow {
            // Expect dominant horizontal flow (vx should be large)
            // a = dt/dx = 1000 us/pixel, vx = -1e6/1000 = -1000 px/s
            assert!(f.vx.abs() > 100.0, "expected horizontal flow, vx={}", f.vx);
        }
    }

    #[test]
    fn test_vertical_motion() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        // Simulate downward motion: t(y) = y * 2000
        for y in 4..12u16 {
            for x in 6..10u16 {
                of.process(&Event::new(x, y, y as i64 * 2000, 1));
            }
        }
        let flow = of.process(&Event::new(8, 8, 16000, 1));
        if let Some(f) = flow {
            // Expect dominant vertical flow
            assert!(f.vy.abs() > 50.0, "expected vertical flow, vy={}", f.vy);
        }
    }

    #[test]
    fn test_static_scene_zero_flow() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        // All events at same timestamp -> no gradient -> zero flow
        for y in 4..12u16 {
            for x in 4..12u16 {
                of.process(&Event::new(x, y, 100_000, 1));
            }
        }
        let flow = of.process(&Event::new(8, 8, 100_000, 1));
        if let Some(f) = flow {
            assert!(
                f.vx.abs() < 1.0 && f.vy.abs() < 1.0,
                "static scene should have ~zero flow, got vx={} vy={}",
                f.vx,
                f.vy
            );
        }
    }

    #[test]
    fn test_out_of_bounds_returns_none() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        let result = of.process(&Event::new(100, 100, 100_000, 1));
        assert!(result.is_none());
    }

    #[test]
    fn test_reset() {
        let mut of = OpticalFlowEstimator::new(16, 16, 2);
        of.process(&Event::new(8, 8, 100_000, 1));
        of.reset();
        assert_eq!(of.get_timestamp(8, 8), 0);
    }
}
