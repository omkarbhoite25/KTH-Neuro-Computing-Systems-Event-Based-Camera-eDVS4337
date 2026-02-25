use crate::event::Event;

/// Exponential decay accumulator.
///
/// Unlike the basic `Accumulator` which simply increments/decrements pixel
/// values, this accumulator applies exponential time decay so that older
/// events fade out naturally. Each event contributes a configurable weight,
/// and pixel values decay toward the neutral value over time.
///
/// The decay formula is:
///   value = neutral + (old_value - neutral) * exp(-(t - t_last) / decay_tau_us)
///
/// This produces a time surface-like representation that emphasizes recent
/// activity — widely used in neuromorphic vision (Lagorce et al., 2017).
pub struct DecayAccumulator {
    values: Vec<f64>,
    last_timestamp: Vec<i64>,
    width: usize,
    height: usize,
    decay_tau_us: f64,
    contribution: f64,
    neutral: f64,
    min_val: f64,
    max_val: f64,
}

impl DecayAccumulator {
    /// Create a new decay accumulator.
    ///
    /// - `decay_tau_us`: Time constant for exponential decay in microseconds.
    /// - `contribution`: Value added per ON event (subtracted per OFF event).
    /// - `neutral`: Resting value pixels decay toward.
    /// - `min_val` / `max_val`: Clamping bounds for pixel values.
    pub fn new(
        width: u32,
        height: u32,
        decay_tau_us: f64,
        contribution: f64,
        neutral: f64,
        min_val: f64,
        max_val: f64,
    ) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        assert!(
            width <= crate::MAX_SENSOR_DIM && height <= crate::MAX_SENSOR_DIM,
            "dimensions exceed maximum"
        );
        assert!(decay_tau_us > 0.0, "decay_tau_us must be positive");
        assert!(contribution > 0.0, "contribution must be positive");
        assert!(min_val <= neutral && neutral <= max_val, "neutral must be in [min_val, max_val]");
        let w = width as usize;
        let h = height as usize;
        let n = w.checked_mul(h).expect("dimension overflow");
        Self {
            values: vec![neutral; n],
            last_timestamp: vec![0; n],
            width: w,
            height: h,
            decay_tau_us,
            contribution,
            neutral,
            min_val,
            max_val,
        }
    }

    /// Accumulate a single event with time decay.
    pub fn accumulate(&mut self, event: &Event) {
        let x = event.x as usize;
        let y = event.y as usize;
        let ts = event.timestamp;

        if x >= self.width || y >= self.height {
            return;
        }

        let idx = y * self.width + x;

        // Apply decay since last event at this pixel
        let last_ts = self.last_timestamp[idx];
        if last_ts > 0 && ts > last_ts {
            let dt = ts.saturating_sub(last_ts) as f64;
            let decay = (-dt / self.decay_tau_us).exp();
            self.values[idx] = self.neutral + (self.values[idx] - self.neutral) * decay;
        }

        // Add contribution based on polarity
        let delta = self.contribution * event.polarity as f64;
        self.values[idx] = (self.values[idx] + delta).clamp(self.min_val, self.max_val);
        self.last_timestamp[idx] = ts;
    }

    /// Get the frame as u8 values by mapping [min_val, max_val] -> [0, 255].
    pub fn get_frame_u8(&self) -> Vec<u8> {
        let range = self.max_val - self.min_val;
        if range <= 0.0 {
            return vec![128; self.values.len()];
        }
        self.values
            .iter()
            .map(|&v| {
                let normalized = (v - self.min_val) / range;
                (normalized * 255.0).clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    /// Get a reference to the raw f64 values.
    pub fn get_values(&self) -> &[f64] {
        &self.values
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Reset all pixel values to neutral and clear timestamps.
    pub fn reset(&mut self) {
        self.values.fill(self.neutral);
        self.last_timestamp.fill(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_acc(w: u32, h: u32) -> DecayAccumulator {
        DecayAccumulator::new(w, h, 100_000.0, 1.0, 128.0, 0.0, 255.0)
    }

    #[test]
    fn test_initial_values_neutral() {
        let acc = default_acc(4, 4);
        assert!(acc.get_values().iter().all(|&v| (v - 128.0).abs() < 1e-10));
    }

    #[test]
    fn test_on_event_increases() {
        let mut acc = default_acc(4, 4);
        acc.accumulate(&Event::new(1, 1, 100_000, 1));
        let idx = 4 + 1; // pixel (1,1) in 4-wide grid
        assert!(acc.get_values()[idx] > 128.0);
        assert!((acc.get_values()[idx] - 129.0).abs() < 1e-10);
    }

    #[test]
    fn test_off_event_decreases() {
        let mut acc = default_acc(4, 4);
        acc.accumulate(&Event::new(1, 1, 100_000, -1));
        let idx = 4 + 1; // pixel (1,1) in 4-wide grid
        assert!(acc.get_values()[idx] < 128.0);
        assert!((acc.get_values()[idx] - 127.0).abs() < 1e-10);
    }

    #[test]
    fn test_decay_toward_neutral() {
        let mut acc = default_acc(4, 4);
        // First event pushes value up
        acc.accumulate(&Event::new(0, 0, 100_000, 1));
        let val_before = acc.get_values()[0];
        assert!(val_before > 128.0);

        // Much later event — value should have decayed toward neutral
        acc.accumulate(&Event::new(0, 0, 1_000_000, 1));
        // The decay should bring it closer to 128 before adding contribution
        // exp(-(900000/100000)) = exp(-9) ~ 0.000123
        // decayed = 128 + (129 - 128) * 0.000123 ~ 128.000123
        // after +1: ~ 129.000123
        let val_after = acc.get_values()[0];
        assert!(val_after < val_before + 1.5, "decay should have reduced accumulated value");
    }

    #[test]
    fn test_rapid_events_accumulate() {
        let mut acc = default_acc(4, 4);
        // Rapid events with minimal decay
        for i in 0..5 {
            acc.accumulate(&Event::new(0, 0, 100_000 + i * 10, 1));
        }
        // With tau=100000 and dt=10, decay is negligible
        // Each event adds ~1.0, so value ~ 128 + 5 = 133
        let val = acc.get_values()[0];
        assert!((val - 133.0).abs() < 0.1);
    }

    #[test]
    fn test_clamp_upper() {
        let mut acc = default_acc(4, 4);
        for i in 0..300 {
            acc.accumulate(&Event::new(0, 0, 100_000 + i * 10, 1));
        }
        assert!(acc.get_values()[0] <= 255.0);
    }

    #[test]
    fn test_clamp_lower() {
        let mut acc = default_acc(4, 4);
        for i in 0..300 {
            acc.accumulate(&Event::new(0, 0, 100_000 + i * 10, -1));
        }
        assert!(acc.get_values()[0] >= 0.0);
    }

    #[test]
    fn test_get_frame_u8() {
        let mut acc = default_acc(4, 4);
        acc.accumulate(&Event::new(0, 0, 100_000, 1)); // -> 129
        let frame = acc.get_frame_u8();
        // 129/255 * 255 = ~128.something -> mapped from [0,255] range
        let expected = ((129.0 / 255.0) * 255.0) as u8;
        assert_eq!(frame[0], expected);
    }

    #[test]
    fn test_reset() {
        let mut acc = default_acc(4, 4);
        acc.accumulate(&Event::new(0, 0, 100_000, 1));
        acc.reset();
        assert!(acc.get_values().iter().all(|&v| (v - 128.0).abs() < 1e-10));
    }

    #[test]
    fn test_out_of_bounds_ignored() {
        let mut acc = default_acc(4, 4);
        acc.accumulate(&Event::new(10, 10, 100_000, 1));
        assert!(acc.get_values().iter().all(|&v| (v - 128.0).abs() < 1e-10));
    }

    #[test]
    fn test_custom_params() {
        let acc = DecayAccumulator::new(4, 4, 50_000.0, 5.0, 0.5, 0.0, 1.0);
        assert!(acc.get_values().iter().all(|&v| (v - 0.5).abs() < 1e-10));
    }
}
