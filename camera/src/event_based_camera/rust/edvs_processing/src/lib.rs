pub mod accumulator;
pub mod denoise;
pub mod event;
pub mod ffi;
pub mod hot_pixel;

/// Maximum allowed sensor dimension to prevent excessive memory allocation.
/// 32768 x 32768 = ~1 GB per filter, far beyond any real eDVS sensor.
pub const MAX_SENSOR_DIM: u32 = 32768;
