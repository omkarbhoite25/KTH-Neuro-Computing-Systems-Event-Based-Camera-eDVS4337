use crate::event::Event;

/// Polarity mode for the polarity filter.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PolarityMode {
    /// Pass only ON events (polarity > 0).
    OnOnly = 0,
    /// Pass only OFF events (polarity < 0).
    OffOnly = 1,
    /// Pass both ON and OFF events (no filtering).
    Both = 2,
}

impl PolarityMode {
    /// Create from a C-compatible integer. Returns `Both` for invalid values.
    pub fn from_i32(value: i32) -> Self {
        match value {
            0 => PolarityMode::OnOnly,
            1 => PolarityMode::OffOnly,
            _ => PolarityMode::Both,
        }
    }
}

/// Polarity filter — passes only events matching the selected polarity mode.
///
/// Useful for isolating ON (brightness increase) or OFF (brightness decrease)
/// events for separate analysis or visualization.
pub struct PolarityFilter {
    mode: PolarityMode,
}

impl PolarityFilter {
    pub fn new(mode: PolarityMode) -> Self {
        Self { mode }
    }

    /// Returns true if the event's polarity matches the filter mode.
    pub fn filter(&self, event: &Event) -> bool {
        let pol = event.polarity;
        match self.mode {
            PolarityMode::OnOnly => pol > 0,
            PolarityMode::OffOnly => pol < 0,
            PolarityMode::Both => true,
        }
    }

    pub fn mode(&self) -> PolarityMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: PolarityMode) {
        self.mode = mode;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_on_only_passes_on() {
        let filter = PolarityFilter::new(PolarityMode::OnOnly);
        let ev = Event::new(10, 10, 100_000, 1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_on_only_rejects_off() {
        let filter = PolarityFilter::new(PolarityMode::OnOnly);
        let ev = Event::new(10, 10, 100_000, -1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_off_only_passes_off() {
        let filter = PolarityFilter::new(PolarityMode::OffOnly);
        let ev = Event::new(10, 10, 100_000, -1);
        assert!(filter.filter(&ev));
    }

    #[test]
    fn test_off_only_rejects_on() {
        let filter = PolarityFilter::new(PolarityMode::OffOnly);
        let ev = Event::new(10, 10, 100_000, 1);
        assert!(!filter.filter(&ev));
    }

    #[test]
    fn test_both_passes_all() {
        let filter = PolarityFilter::new(PolarityMode::Both);
        assert!(filter.filter(&Event::new(10, 10, 100_000, 1)));
        assert!(filter.filter(&Event::new(10, 10, 100_000, -1)));
    }

    #[test]
    fn test_set_mode() {
        let mut filter = PolarityFilter::new(PolarityMode::OnOnly);
        assert_eq!(filter.mode(), PolarityMode::OnOnly);

        filter.set_mode(PolarityMode::OffOnly);
        assert_eq!(filter.mode(), PolarityMode::OffOnly);
        assert!(!filter.filter(&Event::new(10, 10, 100_000, 1)));
    }

    #[test]
    fn test_from_i32() {
        assert_eq!(PolarityMode::from_i32(0), PolarityMode::OnOnly);
        assert_eq!(PolarityMode::from_i32(1), PolarityMode::OffOnly);
        assert_eq!(PolarityMode::from_i32(2), PolarityMode::Both);
        assert_eq!(PolarityMode::from_i32(99), PolarityMode::Both);
    }
}
