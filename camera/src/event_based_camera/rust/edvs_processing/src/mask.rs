use crate::event::Event;

/// Binary pixel mask filter.
///
/// Each pixel has an enable/disable flag. Events at disabled pixels are
/// rejected. Useful for masking out known dead pixels, noisy regions, or
/// restricting processing to specific sensor areas.
///
/// The mask can be updated dynamically via `set_pixel` and `set_rect`.
pub struct MaskFilter {
    mask: Vec<bool>,
    width: usize,
    height: usize,
}

impl MaskFilter {
    /// Create a new mask filter with all pixels enabled.
    pub fn new(width: u32, height: u32) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            mask: vec![true; n],
            width: w,
            height: h,
        }
    }

    /// Returns true if the event passes (pixel is enabled).
    pub fn filter(&self, event: &Event) -> bool {
        let x = event.x as usize;
        let y = event.y as usize;
        if x >= self.width || y >= self.height {
            return false;
        }
        self.mask[y * self.width + x]
    }

    /// Enable or disable a single pixel.
    pub fn set_pixel(&mut self, x: u16, y: u16, enabled: bool) {
        let x = x as usize;
        let y = y as usize;
        if x < self.width && y < self.height {
            self.mask[y * self.width + x] = enabled;
        }
    }

    /// Enable or disable a rectangular region (inclusive bounds).
    pub fn set_rect(&mut self, x_min: u16, y_min: u16, x_max: u16, y_max: u16, enabled: bool) {
        let x0 = (x_min as usize).min(self.width.saturating_sub(1));
        let x1 = (x_max as usize).min(self.width.saturating_sub(1));
        let y0 = (y_min as usize).min(self.height.saturating_sub(1));
        let y1 = (y_max as usize).min(self.height.saturating_sub(1));

        for y in y0..=y1 {
            for x in x0..=x1 {
                self.mask[y * self.width + x] = enabled;
            }
        }
    }

    /// Check if a specific pixel is enabled.
    pub fn is_enabled(&self, x: u16, y: u16) -> bool {
        let x = x as usize;
        let y = y as usize;
        if x >= self.width || y >= self.height {
            return false;
        }
        self.mask[y * self.width + x]
    }

    /// Disable all pixels.
    pub fn disable_all(&mut self) {
        self.mask.fill(false);
    }

    /// Enable all pixels.
    pub fn enable_all(&mut self) {
        self.mask.fill(true);
    }

    /// Count of enabled pixels.
    pub fn enabled_count(&self) -> usize {
        self.mask.iter().filter(|&&v| v).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_all_enabled_by_default() {
        let mask = MaskFilter::new(4, 4);
        assert_eq!(mask.enabled_count(), 16);
        assert!(mask.filter(&Event::new(2, 2, 100, 1)));
    }

    #[test]
    fn test_disable_pixel() {
        let mut mask = MaskFilter::new(4, 4);
        mask.set_pixel(1, 1, false);
        assert!(!mask.filter(&Event::new(1, 1, 100, 1)));
        assert!(mask.filter(&Event::new(0, 0, 100, 1)));
    }

    #[test]
    fn test_disable_rect() {
        let mut mask = MaskFilter::new(8, 8);
        mask.set_rect(2, 2, 5, 5, false);
        assert!(!mask.filter(&Event::new(3, 3, 100, 1)));
        assert!(!mask.filter(&Event::new(2, 2, 100, 1)));
        assert!(!mask.filter(&Event::new(5, 5, 100, 1)));
        assert!(mask.filter(&Event::new(1, 1, 100, 1)));
        assert!(mask.filter(&Event::new(6, 6, 100, 1)));
        assert_eq!(mask.enabled_count(), 64 - 16); // 8x8 - 4x4 = 48
    }

    #[test]
    fn test_disable_all() {
        let mut mask = MaskFilter::new(4, 4);
        mask.disable_all();
        assert_eq!(mask.enabled_count(), 0);
        assert!(!mask.filter(&Event::new(0, 0, 100, 1)));
    }

    #[test]
    fn test_enable_all() {
        let mut mask = MaskFilter::new(4, 4);
        mask.disable_all();
        mask.enable_all();
        assert_eq!(mask.enabled_count(), 16);
    }

    #[test]
    fn test_re_enable_pixel() {
        let mut mask = MaskFilter::new(4, 4);
        mask.set_pixel(1, 1, false);
        assert!(!mask.is_enabled(1, 1));
        mask.set_pixel(1, 1, true);
        assert!(mask.is_enabled(1, 1));
    }

    #[test]
    fn test_out_of_bounds() {
        let mask = MaskFilter::new(4, 4);
        assert!(!mask.filter(&Event::new(10, 10, 100, 1)));
        assert!(!mask.is_enabled(10, 10));
    }
}
