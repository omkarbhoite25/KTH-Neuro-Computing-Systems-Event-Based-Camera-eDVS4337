use crate::event::Event;

/// Spatial transformation types for event coordinates.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransformType {
    /// Flip horizontally (mirror about vertical axis).
    FlipHorizontal = 0,
    /// Flip vertically (mirror about horizontal axis).
    FlipVertical = 1,
    /// Transpose (swap x and y). Note: changes effective width/height.
    Transpose = 2,
    /// Rotate 90 degrees clockwise.
    Rotate90 = 3,
    /// Rotate 180 degrees.
    Rotate180 = 4,
    /// Rotate 270 degrees clockwise (= 90 counter-clockwise).
    Rotate270 = 5,
}

impl TransformType {
    /// Create from a C-compatible integer. Returns None for invalid values.
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(TransformType::FlipHorizontal),
            1 => Some(TransformType::FlipVertical),
            2 => Some(TransformType::Transpose),
            3 => Some(TransformType::Rotate90),
            4 => Some(TransformType::Rotate180),
            5 => Some(TransformType::Rotate270),
            _ => None,
        }
    }
}

/// Spatial transform filter — applies geometric transforms to event coordinates.
///
/// The transform modifies the event's (x, y) coordinates in-place.
/// Sensor dimensions must be provided so the transform can compute
/// the correct mirrored/rotated coordinates.
pub struct SpatialTransform {
    transform: TransformType,
    width: u16,
    height: u16,
}

impl SpatialTransform {
    pub fn new(transform: TransformType, width: u16, height: u16) -> Self {
        assert!(width > 0 && height > 0, "dimensions must be positive");
        match transform {
            TransformType::Transpose | TransformType::Rotate90 | TransformType::Rotate270 => {
                assert!(
                    width == height,
                    "Transpose/Rotate90/Rotate270 require square dimensions (width == height)"
                );
            }
            _ => {}
        }
        Self {
            transform,
            width,
            height,
        }
    }

    /// Apply the transform to the event, modifying its coordinates in-place.
    /// Returns true if the event is valid after transformation.
    pub fn apply(&self, event: &mut Event) -> bool {
        let x = event.x;
        let y = event.y;
        let w = self.width;
        let h = self.height;

        if x >= w || y >= h {
            return false;
        }

        match self.transform {
            TransformType::FlipHorizontal => {
                event.x = w - 1 - x;
            }
            TransformType::FlipVertical => {
                event.y = h - 1 - y;
            }
            TransformType::Transpose => {
                event.x = y;
                event.y = x;
            }
            TransformType::Rotate90 => {
                // 90 CW: (x, y) -> (h-1-y, x)
                event.x = h - 1 - y;
                event.y = x;
            }
            TransformType::Rotate180 => {
                event.x = w - 1 - x;
                event.y = h - 1 - y;
            }
            TransformType::Rotate270 => {
                // 270 CW: (x, y) -> (y, w-1-x)
                event.x = y;
                event.y = w - 1 - x;
            }
        }
        true
    }

    pub fn transform_type(&self) -> TransformType {
        self.transform
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_event(x: u16, y: u16) -> Event {
        Event::new(x, y, 100_000, 1)
    }

    /// Helper to read x/y from packed struct safely.
    fn xy(ev: &Event) -> (u16, u16) {
        let x = ev.x;
        let y = ev.y;
        (x, y)
    }

    #[test]
    fn test_flip_horizontal() {
        let t = SpatialTransform::new(TransformType::FlipHorizontal, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (117, 20));
    }

    #[test]
    fn test_flip_vertical() {
        let t = SpatialTransform::new(TransformType::FlipVertical, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (10, 107));
    }

    #[test]
    fn test_transpose() {
        let t = SpatialTransform::new(TransformType::Transpose, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (20, 10));
    }

    #[test]
    fn test_rotate_90() {
        let t = SpatialTransform::new(TransformType::Rotate90, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (107, 10));
    }

    #[test]
    fn test_rotate_180() {
        let t = SpatialTransform::new(TransformType::Rotate180, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (117, 107));
    }

    #[test]
    fn test_rotate_270() {
        let t = SpatialTransform::new(TransformType::Rotate270, 128, 128);
        let mut ev = make_event(10, 20);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (20, 117));
    }

    #[test]
    fn test_rotate_360_identity() {
        let t90 = SpatialTransform::new(TransformType::Rotate90, 128, 128);
        let mut ev = make_event(10, 20);
        t90.apply(&mut ev);
        t90.apply(&mut ev);
        t90.apply(&mut ev);
        t90.apply(&mut ev);
        assert_eq!(xy(&ev), (10, 20));
    }

    #[test]
    fn test_double_flip_h_identity() {
        let t = SpatialTransform::new(TransformType::FlipHorizontal, 128, 128);
        let mut ev = make_event(42, 73);
        t.apply(&mut ev);
        t.apply(&mut ev);
        assert_eq!(xy(&ev), (42, 73));
    }

    #[test]
    fn test_out_of_bounds() {
        let t = SpatialTransform::new(TransformType::FlipHorizontal, 128, 128);
        let mut ev = make_event(200, 200);
        assert!(!t.apply(&mut ev));
    }

    #[test]
    fn test_corner_origin() {
        let t = SpatialTransform::new(TransformType::Rotate180, 128, 128);
        let mut ev = make_event(0, 0);
        assert!(t.apply(&mut ev));
        assert_eq!(xy(&ev), (127, 127));
    }

    #[test]
    fn test_from_i32() {
        assert_eq!(
            TransformType::from_i32(0),
            Some(TransformType::FlipHorizontal)
        );
        assert_eq!(TransformType::from_i32(5), Some(TransformType::Rotate270));
        assert_eq!(TransformType::from_i32(99), None);
    }
}
