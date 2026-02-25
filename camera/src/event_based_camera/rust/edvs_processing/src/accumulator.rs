use crate::event::Event;

/// Event accumulation frame.
///
/// Accumulates polarity events into a grayscale frame.
/// Neutral value is 128. ON events (+1) increment, OFF events (-1) decrement,
/// clamped to [0, 255].
pub struct Accumulator {
    frame: Vec<u8>,
    width: usize,
    height: usize,
}

impl Accumulator {
    pub fn new(width: u32, height: u32) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
                "dimensions exceed maximum");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            frame: vec![128; n],
            width: w,
            height: h,
        }
    }

    /// Accumulate a single event into the frame.
    pub fn accumulate(&mut self, event: &Event) {
        let x = event.x as usize;
        let y = event.y as usize;

        if x >= self.width || y >= self.height {
            return;
        }

        let idx = y * self.width + x;
        let current = self.frame[idx] as i16;
        let delta = event.polarity as i16; // -1 or +1
        self.frame[idx] = (current + delta).clamp(0, 255) as u8;
    }

    /// Get a reference to the accumulated frame data.
    pub fn get_frame(&self) -> &[u8] {
        &self.frame
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Reset the frame to neutral (128).
    pub fn reset(&mut self) {
        self.frame.fill(128);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_frame_neutral() {
        let acc = Accumulator::new(4, 4);
        assert!(acc.get_frame().iter().all(|&v| v == 128));
    }

    #[test]
    fn test_on_event_increments() {
        let mut acc = Accumulator::new(4, 4);
        let ev = Event::new(1, 1, 100, 1); // ON
        acc.accumulate(&ev);
        // Pixel (1,1) in a 4-wide grid: index = 1*4 + 1 = 5
        assert_eq!(acc.get_frame()[5], 129);
    }

    #[test]
    fn test_off_event_decrements() {
        let mut acc = Accumulator::new(4, 4);
        let ev = Event::new(1, 1, 100, -1); // OFF
        acc.accumulate(&ev);
        // Pixel (1,1) in a 4-wide grid: index = 1*4 + 1 = 5
        assert_eq!(acc.get_frame()[5], 127);
    }

    #[test]
    fn test_clamp_upper() {
        let mut acc = Accumulator::new(4, 4);
        for i in 0..200 {
            acc.accumulate(&Event::new(0, 0, i, 1));
        }
        assert_eq!(acc.get_frame()[0], 255);
    }

    #[test]
    fn test_clamp_lower() {
        let mut acc = Accumulator::new(4, 4);
        for i in 0..200 {
            acc.accumulate(&Event::new(0, 0, i, -1));
        }
        assert_eq!(acc.get_frame()[0], 0);
    }

    #[test]
    fn test_reset() {
        let mut acc = Accumulator::new(4, 4);
        acc.accumulate(&Event::new(0, 0, 100, 1));
        acc.reset();
        assert!(acc.get_frame().iter().all(|&v| v == 128));
    }

    #[test]
    fn test_out_of_bounds_ignored() {
        let mut acc = Accumulator::new(4, 4);
        acc.accumulate(&Event::new(10, 10, 100, 1));
        assert!(acc.get_frame().iter().all(|&v| v == 128));
    }
}
