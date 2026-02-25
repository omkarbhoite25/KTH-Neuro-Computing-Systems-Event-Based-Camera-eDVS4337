pub mod accumulator;
pub mod anti_flicker;
pub mod decay_accumulator;
pub mod decimation;
pub mod denoise;
pub mod event;
pub mod ffi;
pub mod hot_pixel;
pub mod mask;
pub mod polarity;
pub mod rate_stats;
pub mod refractory;
pub mod roi;
pub mod slicer;
pub mod stc;
pub mod time_surface;
pub mod transform;

/// Maximum allowed sensor dimension to prevent excessive memory allocation.
/// 32768 x 32768 = ~1 GB per filter, far beyond any real eDVS sensor.
pub const MAX_SENSOR_DIM: u32 = 32768;
