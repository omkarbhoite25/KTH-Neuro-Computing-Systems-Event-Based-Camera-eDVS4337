/// Single polarity event from the eDVS camera.
/// Layout matches the C++ `edvs::Event` struct exactly.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub x: u16,
    pub y: u16,
    pub timestamp: i64,
    pub polarity: i8, // -1 or +1
}

impl Event {
    pub fn new(x: u16, y: u16, timestamp: i64, polarity: i8) -> Self {
        Self {
            x,
            y,
            timestamp,
            polarity,
        }
    }
}

impl std::fmt::Display for Event {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Copy fields to locals to avoid unaligned references from packed struct
        let x = self.x;
        let y = self.y;
        let ts = self.timestamp;
        let pol = self.polarity;
        write!(f, "{}\t{}\t{}\t{}", x, y, ts, pol)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_event_size() {
        // Verify FFI-compatible layout: 2+2+8+1 = 13 bytes packed
        assert_eq!(std::mem::size_of::<Event>(), 13);
    }

    #[test]
    fn test_event_display() {
        let ev = Event::new(10, 20, 123456, 1);
        assert_eq!(format!("{}", ev), "10\t20\t123456\t1");
    }
}
