pub mod accumulator;
pub mod anti_flicker;
pub mod corner;
pub mod decay_accumulator;
pub mod decimation;
pub mod denoise;
pub mod event;
pub mod ffi;
pub mod frequency;
pub mod hot_pixel;
pub mod mask;
pub mod optical_flow;
pub mod polarity;
pub mod rate_stats;
pub mod refractory;
pub mod roi;
pub mod sits;
pub mod slicer;
pub mod stc;
pub mod time_surface;
pub mod transform;
pub mod voxel_grid;

/// Maximum allowed sensor dimension to prevent excessive memory allocation.
/// 32768 x 32768 = ~1 GB per filter, far beyond any real eDVS sensor.
pub const MAX_SENSOR_DIM: u32 = 32768;
