use crate::event::Event;

/// FA-Harris Corner Detector for event cameras (Mueggler et al., 2017).
///
/// Computes the Harris corner response at each event's location using the
/// local time surface patch. Events at corners produce a high response,
/// while events along edges or flat regions produce low responses.
///
/// The algorithm:
/// 1. Maintain a per-pixel timestamp map (time surface).
/// 2. For each event, extract the local NxN patch of timestamps.
/// 3. Compute spatial gradients (Sobel) on the time surface patch.
/// 4. Build the structure tensor M = [[sum(Ix^2), sum(Ix*Iy)], [sum(Ix*Iy), sum(Iy^2)]].
/// 5. Compute Harris response R = det(M) - k * trace(M)^2.
/// 6. Classify event as corner if R > threshold.
pub struct HarrisCornerDetector {
    timestamps: Vec<i64>,
    width: usize,
    height: usize,
    harris_k: f64,
    threshold: f64,
}

impl HarrisCornerDetector {
    /// Create a new Harris corner detector.
    ///
    /// - `harris_k`: Harris detector sensitivity parameter (typical: 0.04-0.06).
    /// - `threshold`: Minimum Harris response to classify as corner.
    pub fn new(width: u32, height: u32, harris_k: f64, threshold: f64) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(harris_k > 0.0, "harris_k must be positive");
        assert!(threshold >= 0.0, "threshold must be non-negative");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            timestamps: vec![0; n],
            width: w,
            height: h,
            harris_k,
            threshold,
        }
    }

    /// Process an event: update the time surface and compute corner response.
    ///
    /// Returns `Some(response)` if the event is classified as a corner
    /// (response >= threshold), or `None` if it is not a corner.
    pub fn process(&mut self, event: &Event) -> Option<f64> {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return None;
        }

        // Update time surface
        self.timestamps[y * self.width + x] = event.timestamp;

        self.compute_response(x, y)
    }

    /// Compute Harris response at pixel (x, y) using a windowed structure tensor.
    ///
    /// Computes Sobel gradients at each pixel in a 3x3 window centered at (x, y),
    /// sums the outer products to form the structure tensor, then evaluates Harris.
    /// Requires a 5x5 neighborhood (2-pixel border).
    fn compute_response(&self, x: usize, y: usize) -> Option<f64> {
        if x < 2 || y < 2 || x + 2 >= self.width || y + 2 >= self.height {
            return None;
        }

        let ts_at = |px: usize, py: usize| -> f64 {
            self.timestamps[py * self.width + px] as f64
        };

        // Compute Sobel gradients at each pixel in a 3x3 window, then sum
        let mut sum_ixx = 0.0_f64;
        let mut sum_iyy = 0.0_f64;
        let mut sum_ixy = 0.0_f64;

        for wy in -1i32..=1 {
            for wx in -1i32..=1 {
                let cx = (x as i32 + wx) as usize;
                let cy = (y as i32 + wy) as usize;

                // Sobel at (cx, cy)
                let ix = -ts_at(cx - 1, cy - 1) + ts_at(cx + 1, cy - 1)
                    - 2.0 * ts_at(cx - 1, cy)
                    + 2.0 * ts_at(cx + 1, cy)
                    - ts_at(cx - 1, cy + 1)
                    + ts_at(cx + 1, cy + 1);

                let iy = -ts_at(cx - 1, cy - 1) - 2.0 * ts_at(cx, cy - 1) - ts_at(cx + 1, cy - 1)
                    + ts_at(cx - 1, cy + 1)
                    + 2.0 * ts_at(cx, cy + 1)
                    + ts_at(cx + 1, cy + 1);

                sum_ixx += ix * ix;
                sum_iyy += iy * iy;
                sum_ixy += ix * iy;
            }
        }

        // Harris response: det(M) - k * trace(M)^2
        let det = sum_ixx * sum_iyy - sum_ixy * sum_ixy;
        let trace = sum_ixx + sum_iyy;
        let response = det - self.harris_k * trace * trace;

        if response >= self.threshold {
            Some(response)
        } else {
            None
        }
    }

    /// Get the Harris response at an event's location without updating the time surface.
    pub fn query_response(&self, x: u16, y: u16) -> f64 {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height {
            return 0.0;
        }
        self.compute_response(x, y).unwrap_or(0.0)
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

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.timestamps.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flat_region_no_corner() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 1.0);
        // All events at same time -> flat time surface -> no corner
        for y in 0..16u16 {
            for x in 0..16u16 {
                det.process(&Event::new(x, y, 100_000, 1));
            }
        }
        // Interior point on flat surface should have zero response
        let r = det.query_response(8, 8);
        assert!(r.abs() < 0.01, "flat region response should be ~0, got {}", r);
    }

    #[test]
    fn test_edge_produces_negative_response() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 0.0);
        // Horizontal edge: top half at t=100k, bottom half at t=200k
        for y in 0..8u16 {
            for x in 0..16u16 {
                det.process(&Event::new(x, y, 100_000, 1));
            }
        }
        for y in 8..16u16 {
            for x in 0..16u16 {
                det.process(&Event::new(x, y, 200_000, 1));
            }
        }
        // Point on the edge: Harris response should be negative (edge, not corner)
        let r = det.query_response(8, 8);
        assert!(r <= 0.0, "edge should not be detected as corner, got {}", r);
    }

    #[test]
    fn test_corner_produces_high_response() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 0.0);
        // Create a corner-like pattern: different timestamps in quadrants
        for y in 0..8u16 {
            for x in 0..8u16 {
                det.process(&Event::new(x, y, 100_000, 1));
            }
            for x in 8..16u16 {
                det.process(&Event::new(x, y, 200_000, 1));
            }
        }
        for y in 8..16u16 {
            for x in 0..8u16 {
                det.process(&Event::new(x, y, 300_000, 1));
            }
            for x in 8..16u16 {
                det.process(&Event::new(x, y, 400_000, 1));
            }
        }
        // The corner junction area should have positive Harris response
        // due to gradients in both directions within the window
        let r = det.query_response(8, 8);
        assert!(r > 0.0, "corner should have positive response, got {}", r);
    }

    #[test]
    fn test_border_returns_none() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 0.0);
        // Border events can't compute windowed Sobel (need 2-pixel border) -> None
        let result = det.process(&Event::new(0, 0, 100_000, 1));
        assert!(result.is_none());
        let result = det.process(&Event::new(1, 1, 100_000, 1));
        assert!(result.is_none());
        let result = det.process(&Event::new(15, 15, 100_000, 1));
        assert!(result.is_none());
    }

    #[test]
    fn test_out_of_bounds_returns_none() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 0.0);
        let result = det.process(&Event::new(100, 100, 100_000, 1));
        assert!(result.is_none());
    }

    #[test]
    fn test_threshold_filtering() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 1e30);
        // Very high threshold -> nothing is a corner
        for y in 0..16u16 {
            for x in 0..16u16 {
                let result = det.process(&Event::new(x, y, (x as i64 + 1) * (y as i64 + 1) * 1000, 1));
                assert!(result.is_none());
            }
        }
    }

    #[test]
    fn test_reset() {
        let mut det = HarrisCornerDetector::new(16, 16, 0.04, 0.0);
        det.process(&Event::new(8, 8, 100_000, 1));
        det.reset();
        assert_eq!(det.get_timestamp(8, 8), 0);
    }
}
