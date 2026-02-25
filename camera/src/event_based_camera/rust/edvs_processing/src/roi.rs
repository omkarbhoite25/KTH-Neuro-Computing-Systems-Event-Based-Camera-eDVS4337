use crate::event::Event;

/// Region of interest (ROI) filter.
///
/// Only passes events whose (x, y) coordinates fall within the configured
/// rectangular region [x_min, x_max] x [y_min, y_max] (inclusive bounds).
pub struct RoiFilter {
    x_min: u16,
    y_min: u16,
    x_max: u16,
    y_max: u16,
}

impl RoiFilter {
    /// Create a new ROI filter. Bounds are inclusive.
    ///
    /// # Panics
    /// Panics if `x_min > x_max` or `y_min > y_max`.
    pub fn new(x_min: u16, y_min: u16, x_max: u16, y_max: u16) -> Self {
        assert!(x_min <= x_max, "x_min must be <= x_max");
        assert!(y_min <= y_max, "y_min must be <= y_max");
        Self {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    /// Returns true if the event falls within the ROI.
    pub fn filter(&self, event: &Event) -> bool {
        let x = event.x;
        let y = event.y;
        x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max
    }

    pub fn x_min(&self) -> u16 {
        self.x_min
    }
    pub fn y_min(&self) -> u16 {
        self.y_min
    }
    pub fn x_max(&self) -> u16 {
        self.x_max
    }
    pub fn y_max(&self) -> u16 {
        self.y_max
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inside_roi_passes() {
        let filter = RoiFilter::new(10, 10, 50, 50);
        let ev = Event::new(30, 30, 100_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_outside_roi_rejected() {
        let filter = RoiFilter::new(10, 10, 50, 50);
        let ev = Event::new(5, 5, 100_000, 1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_boundary_inclusive() {
        let filter = RoiFilter::new(10, 10, 50, 50);
        // All four corners should pass
        assert!(filter.filter(&Event::new(10, 10, 100_000, 1)));
        assert!(filter.filter(&Event::new(50, 50, 100_000, 1)));
        assert!(filter.filter(&Event::new(10, 50, 100_000, 1)));
        assert!(filter.filter(&Event::new(50, 10, 100_000, 1)));
    }

    #[test]
    fn test_just_outside_boundary() {
        let filter = RoiFilter::new(10, 10, 50, 50);
        assert!(!filter.filter(&Event::new(9, 30, 100_000, 1)));
        assert!(!filter.filter(&Event::new(51, 30, 100_000, 1)));
        assert!(!filter.filter(&Event::new(30, 9, 100_000, 1)));
        assert!(!filter.filter(&Event::new(30, 51, 100_000, 1)));
    }

    #[test]
    fn test_single_pixel_roi() {
        let filter = RoiFilter::new(20, 20, 20, 20);
        assert!(filter.filter(&Event::new(20, 20, 100_000, 1)));
        assert!(!filter.filter(&Event::new(21, 20, 100_000, 1)));
        assert!(!filter.filter(&Event::new(19, 20, 100_000, 1)));
    }

    #[test]
    fn test_full_sensor_roi() {
        let filter = RoiFilter::new(0, 0, 127, 127);
        assert!(filter.filter(&Event::new(0, 0, 100_000, 1)));
        assert!(filter.filter(&Event::new(127, 127, 100_000, 1)));
        assert!(filter.filter(&Event::new(64, 64, 100_000, 1)));
    }

    #[test]
    #[should_panic(expected = "x_min must be <= x_max")]
    fn test_invalid_x_bounds() {
        RoiFilter::new(50, 10, 10, 50);
    }

    #[test]
    #[should_panic(expected = "y_min must be <= y_max")]
    fn test_invalid_y_bounds() {
        RoiFilter::new(10, 50, 50, 10);
    }
}
