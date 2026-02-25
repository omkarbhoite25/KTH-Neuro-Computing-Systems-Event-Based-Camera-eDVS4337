use std::panic;

use crate::accumulator::Accumulator;
use crate::anti_flicker::AntiFlickerFilter;
use crate::decay_accumulator::DecayAccumulator;
use crate::decimation::DecimationFilter;
use crate::denoise::TemporalFilter;
use crate::event::Event;
use crate::hot_pixel::HotPixelFilter;
use crate::mask::MaskFilter;
use crate::polarity::{PolarityFilter, PolarityMode};
use crate::rate_stats::EventRateStats;
use crate::refractory::RefractoryFilter;
use crate::roi::RoiFilter;
use crate::slicer::{EventSlicer, SliceMode};
use crate::stc::StcFilter;
use crate::time_surface::TimeSurface;
use crate::transform::{SpatialTransform, TransformType};

use crate::corner::HarrisCornerDetector;
use crate::frequency::FrequencyEstimator;
use crate::optical_flow::OpticalFlowEstimator;
use crate::sits::SpeedInvariantTimeSurface;
use crate::voxel_grid::VoxelGrid;

// --- TemporalFilter FFI ---

/// Creates a new temporal denoising filter.
///
/// Returns a null pointer if `width`, `height`, or `threshold_us` are invalid (zero or negative).
#[no_mangle]
pub extern "C" fn edvs_temporal_filter_create(
    width: u32,
    height: u32,
    threshold_us: i64,
) -> *mut TemporalFilter {
    let result = panic::catch_unwind(|| {
        Box::new(TemporalFilter::new(width, height, threshold_us))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_temporal_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `filter` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_temporal_filter_destroy` on `filter`.
#[no_mangle]
pub unsafe extern "C" fn edvs_temporal_filter_process(
    filter: *mut TemporalFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &mut *filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_temporal_filter_create`,
///   or null (in which case this is a no-op).
/// - After this call, `filter` is invalid and must not be used again.
/// - Must not be called concurrently with `edvs_temporal_filter_process` on the same pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_temporal_filter_destroy(filter: *mut TemporalFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- HotPixelFilter FFI ---

/// Creates a new hot pixel filter.
///
/// Returns a null pointer if `width`, `height`, `window_us`, or `max_rate` are invalid
/// (zero or negative).
#[no_mangle]
pub extern "C" fn edvs_hot_pixel_filter_create(
    width: u32,
    height: u32,
    window_us: i64,
    max_rate: u32,
) -> *mut HotPixelFilter {
    let result = panic::catch_unwind(|| {
        Box::new(HotPixelFilter::new(width, height, window_us, max_rate))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_hot_pixel_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `filter` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_hot_pixel_filter_destroy` on `filter`.
#[no_mangle]
pub unsafe extern "C" fn edvs_hot_pixel_filter_process(
    filter: *mut HotPixelFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &mut *filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_hot_pixel_filter_create`,
///   or null (in which case this is a no-op).
/// - After this call, `filter` is invalid and must not be used again.
/// - Must not be called concurrently with `edvs_hot_pixel_filter_process` on the same pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_hot_pixel_filter_destroy(filter: *mut HotPixelFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- Accumulator FFI ---

/// Creates a new event accumulator.
///
/// Returns a null pointer if `width` or `height` are zero.
#[no_mangle]
pub extern "C" fn edvs_accumulator_create(width: u32, height: u32) -> *mut Accumulator {
    let result = panic::catch_unwind(|| {
        Box::new(Accumulator::new(width, height))
    });
    match result {
        Ok(acc) => Box::into_raw(acc),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `acc` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_accumulator_destroy` on `acc`.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_accumulate(
    acc: *mut Accumulator,
    event: *const Event,
) {
    if acc.is_null() || event.is_null() {
        return;
    }
    let acc_ref = &mut *acc;
    let event_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| acc_ref.accumulate(event_ref)));
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`.
/// - The returned pointer borrows internal memory of the `Accumulator`. It is valid
///   only until the next call to `edvs_accumulator_accumulate`, `edvs_accumulator_reset`,
///   or `edvs_accumulator_destroy` on the same `acc`.
/// - `out_len`, if non-null, will be written with the frame length in bytes.
/// - The caller must copy the frame data if it needs to persist beyond the next mutation.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_get_frame(
    acc: *const Accumulator,
    out_len: *mut usize,
) -> *const u8 {
    if acc.is_null() {
        return std::ptr::null();
    }
    let acc = &*acc;
    let frame = acc.get_frame();
    if !out_len.is_null() {
        *out_len = frame.len();
    }
    frame.as_ptr()
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`,
///   or null (in which case this is a no-op).
/// - Any frame pointers previously obtained via `edvs_accumulator_get_frame` are
///   invalidated after this call.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_reset(acc: *mut Accumulator) {
    if !acc.is_null() {
        (*acc).reset();
    }
}

/// # Safety
///
/// - `acc` must be a valid pointer returned by `edvs_accumulator_create`,
///   or null (in which case this is a no-op).
/// - After this call, `acc` is invalid and must not be used again.
/// - Any frame pointers previously obtained via `edvs_accumulator_get_frame` are
///   invalidated after this call.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_destroy(acc: *mut Accumulator) {
    if !acc.is_null() {
        drop(Box::from_raw(acc));
    }
}

// --- RefractoryFilter FFI ---

/// Creates a new refractory period filter.
///
/// Returns null if parameters are invalid (zero dimensions or non-positive period).
#[no_mangle]
pub extern "C" fn edvs_refractory_filter_create(
    width: u32,
    height: u32,
    refractory_period_us: i64,
) -> *mut RefractoryFilter {
    let result = panic::catch_unwind(|| {
        Box::new(RefractoryFilter::new(width, height, refractory_period_us))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_refractory_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_refractory_filter_process(
    filter: *mut RefractoryFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &mut *filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_refractory_filter_create`,
///   or null (no-op).
#[no_mangle]
pub unsafe extern "C" fn edvs_refractory_filter_destroy(filter: *mut RefractoryFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- PolarityFilter FFI ---

/// Creates a new polarity filter.
///
/// `mode`: 0 = OnOnly, 1 = OffOnly, 2 = Both (default for invalid values).
#[no_mangle]
pub extern "C" fn edvs_polarity_filter_create(mode: i32) -> *mut PolarityFilter {
    let result = panic::catch_unwind(|| {
        Box::new(PolarityFilter::new(PolarityMode::from_i32(mode)))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_polarity_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_polarity_filter_process(
    filter: *const PolarityFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &*filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_polarity_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_polarity_filter_destroy(filter: *mut PolarityFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- RoiFilter FFI ---

/// Creates a new ROI filter. Bounds are inclusive.
///
/// Returns null if `x_min > x_max` or `y_min > y_max`.
#[no_mangle]
pub extern "C" fn edvs_roi_filter_create(
    x_min: u16,
    y_min: u16,
    x_max: u16,
    y_max: u16,
) -> *mut RoiFilter {
    let result = panic::catch_unwind(|| {
        Box::new(RoiFilter::new(x_min, y_min, x_max, y_max))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_roi_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_roi_filter_process(
    filter: *const RoiFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &*filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_roi_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_roi_filter_destroy(filter: *mut RoiFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- DecimationFilter FFI ---

/// Creates a new decimation filter that passes every `factor`-th event.
///
/// Returns null if `factor` is 0.
#[no_mangle]
pub extern "C" fn edvs_decimation_filter_create(factor: u32) -> *mut DecimationFilter {
    let result = panic::catch_unwind(|| {
        Box::new(DecimationFilter::new(factor))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_decimation_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decimation_filter_process(
    filter: *mut DecimationFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter_ref = &mut *filter;
    let event_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| filter_ref.filter(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_decimation_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decimation_filter_destroy(filter: *mut DecimationFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- SpatialTransform FFI ---

/// Creates a new spatial transform.
///
/// `transform_type`: 0=FlipH, 1=FlipV, 2=Transpose, 3=Rot90, 4=Rot180, 5=Rot270.
/// Returns null for invalid transform_type or zero dimensions.
#[no_mangle]
pub extern "C" fn edvs_spatial_transform_create(
    transform_type: i32,
    width: u16,
    height: u16,
) -> *mut SpatialTransform {
    let result = panic::catch_unwind(|| {
        let tt = TransformType::from_i32(transform_type)
            .expect("invalid transform type");
        Box::new(SpatialTransform::new(tt, width, height))
    });
    match result {
        Ok(t) => Box::into_raw(t),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `transform` must be a valid, non-null pointer returned by `edvs_spatial_transform_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
///   The event's coordinates are modified in-place.
#[no_mangle]
pub unsafe extern "C" fn edvs_spatial_transform_apply(
    transform: *const SpatialTransform,
    event: *mut Event,
) -> bool {
    if transform.is_null() || event.is_null() {
        return false;
    }
    let transform_ref = &*transform;
    let event_ref = &mut *event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| transform_ref.apply(event_ref)))
        .unwrap_or(false)
}

/// # Safety
///
/// - `transform` must be a valid pointer returned by `edvs_spatial_transform_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_spatial_transform_destroy(transform: *mut SpatialTransform) {
    if !transform.is_null() {
        drop(Box::from_raw(transform));
    }
}

// --- DecayAccumulator FFI ---

/// Creates a new decay accumulator.
///
/// Returns null for invalid parameters (zero dimensions, non-positive decay/contribution,
/// neutral outside [min_val, max_val]).
#[no_mangle]
pub extern "C" fn edvs_decay_accumulator_create(
    width: u32,
    height: u32,
    decay_tau_us: f64,
    contribution: f64,
    neutral: f64,
    min_val: f64,
    max_val: f64,
) -> *mut DecayAccumulator {
    let result = panic::catch_unwind(|| {
        Box::new(DecayAccumulator::new(
            width, height, decay_tau_us, contribution, neutral, min_val, max_val,
        ))
    });
    match result {
        Ok(acc) => Box::into_raw(acc),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_accumulate(
    acc: *mut DecayAccumulator,
    event: *const Event,
) {
    if acc.is_null() || event.is_null() {
        return;
    }
    let acc_ref = &mut *acc;
    let event_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| acc_ref.accumulate(event_ref)));
}

/// Returns a pointer to a heap-allocated u8 frame buffer and its length.
/// The caller must free the returned buffer with `edvs_decay_accumulator_free_frame`.
///
/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`.
/// - `out_len` must be a valid, non-null pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_get_frame_u8(
    acc: *const DecayAccumulator,
    out_len: *mut usize,
) -> *mut u8 {
    if acc.is_null() || out_len.is_null() {
        return std::ptr::null_mut();
    }
    let acc = &*acc;
    let frame = acc.get_frame_u8().into_boxed_slice();
    *out_len = frame.len();
    Box::into_raw(frame) as *mut u8
}

/// Free a frame buffer allocated by `edvs_decay_accumulator_get_frame_u8`.
///
/// # Safety
///
/// - `ptr` must be a pointer returned by `edvs_decay_accumulator_get_frame_u8`, or null.
/// - `len` must be the length returned via `out_len`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_free_frame(ptr: *mut u8, len: usize) {
    if !ptr.is_null() && len > 0 {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)));
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_reset(acc: *mut DecayAccumulator) {
    if !acc.is_null() {
        (*acc).reset();
    }
}

/// # Safety
///
/// - `acc` must be a valid pointer returned by `edvs_decay_accumulator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_destroy(acc: *mut DecayAccumulator) {
    if !acc.is_null() {
        drop(Box::from_raw(acc));
    }
}

// =========================================================================
// Phase 2 FFI exports
// =========================================================================

// --- TimeSurface FFI ---

#[no_mangle]
pub extern "C" fn edvs_time_surface_create(
    width: u32,
    height: u32,
    decay_tau_us: f64,
) -> *mut TimeSurface {
    let result = panic::catch_unwind(|| Box::new(TimeSurface::new(width, height, decay_tau_us)));
    match result {
        Ok(ts) => Box::into_raw(ts),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `ts` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_time_surface_update(ts: *mut TimeSurface, event: *const Event) {
    if ts.is_null() || event.is_null() {
        return;
    }
    let ts_ref = &mut *ts;
    let ev_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| ts_ref.update(ev_ref)));
}

/// # Safety
/// - `ts` must be valid. Caller must free returned buffer with `edvs_time_surface_free_frame`.
#[no_mangle]
pub unsafe extern "C" fn edvs_time_surface_get_frame(
    ts: *const TimeSurface,
    t_ref: i64,
    out_len: *mut usize,
) -> *mut u8 {
    if ts.is_null() || out_len.is_null() {
        return std::ptr::null_mut();
    }
    let ts_ref = &*ts;
    let frame = ts_ref.get_frame_at(t_ref).into_boxed_slice();
    *out_len = frame.len();
    Box::into_raw(frame) as *mut u8
}

/// # Safety
/// - `ptr` must be a pointer returned by `edvs_time_surface_get_frame`, or null.
/// - `len` must be the length returned via `out_len`.
#[no_mangle]
pub unsafe extern "C" fn edvs_time_surface_free_frame(ptr: *mut u8, len: usize) {
    if !ptr.is_null() && len > 0 {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)));
    }
}

/// # Safety
/// - `ts` must be a valid pointer returned by `edvs_time_surface_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_time_surface_reset(ts: *mut TimeSurface) {
    if !ts.is_null() {
        let ts_ref = &mut *ts;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| ts_ref.reset()));
    }
}

/// # Safety
/// - `ts` must be a valid pointer returned by `edvs_time_surface_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_time_surface_destroy(ts: *mut TimeSurface) {
    if !ts.is_null() {
        drop(Box::from_raw(ts));
    }
}

// --- EventSlicer FFI ---

/// `mode`: 0 = ByCount, 1 = ByTime.
#[no_mangle]
pub extern "C" fn edvs_slicer_create(mode: i32, threshold: i64) -> *mut EventSlicer {
    let result = panic::catch_unwind(|| {
        let m = SliceMode::from_i32(mode).expect("invalid slice mode");
        Box::new(EventSlicer::new(m, threshold))
    });
    match result {
        Ok(s) => Box::into_raw(s),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `slicer` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_slicer_process(
    slicer: *mut EventSlicer,
    event: *const Event,
) -> bool {
    if slicer.is_null() || event.is_null() {
        return false;
    }
    let s_ref = &mut *slicer;
    let ev_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.process(ev_ref))).unwrap_or(false)
}

/// # Safety
/// - `slicer` must be a valid pointer returned by `edvs_slicer_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_slicer_reset(slicer: *mut EventSlicer) {
    if !slicer.is_null() {
        let s_ref = &mut *slicer;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.reset()));
    }
}

/// # Safety
/// - `slicer` must be a valid pointer returned by `edvs_slicer_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_slicer_destroy(slicer: *mut EventSlicer) {
    if !slicer.is_null() {
        drop(Box::from_raw(slicer));
    }
}

// --- AntiFlickerFilter FFI ---

#[no_mangle]
pub extern "C" fn edvs_anti_flicker_filter_create(
    width: u32,
    height: u32,
    period_us: i64,
    tolerance_us: i64,
) -> *mut AntiFlickerFilter {
    let result = panic::catch_unwind(|| {
        Box::new(AntiFlickerFilter::new(width, height, period_us, tolerance_us))
    });
    match result {
        Ok(f) => Box::into_raw(f),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `filter` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_anti_flicker_filter_process(
    filter: *mut AntiFlickerFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let f_ref = &mut *filter;
    let ev_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| f_ref.filter(ev_ref))).unwrap_or(false)
}

/// # Safety
/// - `filter` must be a valid pointer returned by `edvs_anti_flicker_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_anti_flicker_filter_destroy(filter: *mut AntiFlickerFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- StcFilter FFI ---

#[no_mangle]
pub extern "C" fn edvs_stc_filter_create(
    width: u32,
    height: u32,
    threshold_us: i64,
) -> *mut StcFilter {
    let result =
        panic::catch_unwind(|| Box::new(StcFilter::new(width, height, threshold_us)));
    match result {
        Ok(f) => Box::into_raw(f),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `filter` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_stc_filter_process(
    filter: *mut StcFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let f_ref = &mut *filter;
    let ev_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| f_ref.filter(ev_ref))).unwrap_or(false)
}

/// # Safety
/// - `filter` must be a valid pointer returned by `edvs_stc_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_stc_filter_destroy(filter: *mut StcFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- EventRateStats FFI ---

#[no_mangle]
pub extern "C" fn edvs_rate_stats_create(window_us: i64) -> *mut EventRateStats {
    let result = panic::catch_unwind(|| Box::new(EventRateStats::new(window_us)));
    match result {
        Ok(s) => Box::into_raw(s),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `stats` and `event` must be valid, non-null pointers.
///
/// Returns the rate (events/sec) via `out_rate` if a window boundary was crossed.
/// Returns true if a rate was computed.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_record(
    stats: *mut EventRateStats,
    event: *const Event,
    out_rate: *mut f64,
) -> bool {
    if stats.is_null() || event.is_null() {
        return false;
    }
    let s_ref = &mut *stats;
    let ev_ref = &*event;
    let result =
        panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.record(ev_ref))).unwrap_or(None);
    match result {
        Some(rate) => {
            if !out_rate.is_null() {
                *out_rate = rate;
            }
            true
        }
        None => false,
    }
}

/// # Safety
/// - `stats` must be a valid pointer returned by `edvs_rate_stats_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_last_rate(stats: *const EventRateStats) -> f64 {
    if stats.is_null() {
        return 0.0;
    }
    let s_ref = &*stats;
    panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.last_rate())).unwrap_or(0.0)
}

/// # Safety
/// - `stats` must be a valid pointer returned by `edvs_rate_stats_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_peak_rate(stats: *const EventRateStats) -> f64 {
    if stats.is_null() {
        return 0.0;
    }
    let s_ref = &*stats;
    panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.peak_rate())).unwrap_or(0.0)
}

/// # Safety
/// - `stats` must be a valid pointer returned by `edvs_rate_stats_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_total_count(stats: *const EventRateStats) -> u64 {
    if stats.is_null() {
        return 0;
    }
    let s_ref = &*stats;
    panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.total_count())).unwrap_or(0)
}

/// # Safety
/// - `stats` must be a valid pointer returned by `edvs_rate_stats_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_reset(stats: *mut EventRateStats) {
    if !stats.is_null() {
        let s_ref = &mut *stats;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.reset()));
    }
}

/// # Safety
/// - `stats` must be a valid pointer returned by `edvs_rate_stats_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_rate_stats_destroy(stats: *mut EventRateStats) {
    if !stats.is_null() {
        drop(Box::from_raw(stats));
    }
}

// --- MaskFilter FFI ---

#[no_mangle]
pub extern "C" fn edvs_mask_filter_create(width: u32, height: u32) -> *mut MaskFilter {
    let result = panic::catch_unwind(|| Box::new(MaskFilter::new(width, height)));
    match result {
        Ok(m) => Box::into_raw(m),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `filter` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_mask_filter_process(
    filter: *const MaskFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let f_ref = &*filter;
    let ev_ref = &*event;
    panic::catch_unwind(panic::AssertUnwindSafe(|| f_ref.filter(ev_ref))).unwrap_or(false)
}

/// # Safety
/// - `filter` must be valid and non-null.
#[no_mangle]
pub unsafe extern "C" fn edvs_mask_filter_set_pixel(
    filter: *mut MaskFilter,
    x: u16,
    y: u16,
    enabled: bool,
) {
    if !filter.is_null() {
        let f_ref = &mut *filter;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| f_ref.set_pixel(x, y, enabled)));
    }
}

/// # Safety
/// - `filter` must be valid and non-null.
#[no_mangle]
pub unsafe extern "C" fn edvs_mask_filter_set_rect(
    filter: *mut MaskFilter,
    x_min: u16,
    y_min: u16,
    x_max: u16,
    y_max: u16,
    enabled: bool,
) {
    if !filter.is_null() {
        let f_ref = &mut *filter;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| {
            f_ref.set_rect(x_min, y_min, x_max, y_max, enabled)
        }));
    }
}

/// # Safety
/// - `filter` must be a valid pointer returned by `edvs_mask_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_mask_filter_destroy(filter: *mut MaskFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// =========================================================================
// Phase 3 FFI exports
// =========================================================================

// --- SpeedInvariantTimeSurface FFI ---

/// Creates a speed-invariant time surface.
///
/// Returns null if parameters are invalid.
#[no_mangle]
pub extern "C" fn edvs_sits_create(
    width: u32,
    height: u32,
    radius: u32,
) -> *mut SpeedInvariantTimeSurface {
    let result =
        panic::catch_unwind(|| Box::new(SpeedInvariantTimeSurface::new(width, height, radius)));
    match result {
        Ok(s) => Box::into_raw(s),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `sits` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_sits_update(
    sits: *mut SpeedInvariantTimeSurface,
    event: *const Event,
) {
    if sits.is_null() || event.is_null() {
        return;
    }
    let s_ref = &mut *sits;
    let ev_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.update(ev_ref)));
}

/// # Safety
/// - `sits` must be a valid pointer returned by `edvs_sits_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_sits_reset(sits: *mut SpeedInvariantTimeSurface) {
    if !sits.is_null() {
        let s_ref = &mut *sits;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| s_ref.reset()));
    }
}

/// # Safety
/// - `sits` must be a valid pointer returned by `edvs_sits_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_sits_destroy(sits: *mut SpeedInvariantTimeSurface) {
    if !sits.is_null() {
        drop(Box::from_raw(sits));
    }
}

// --- VoxelGrid FFI ---

/// Creates a voxel grid.
///
/// Returns null if parameters are invalid.
#[no_mangle]
pub extern "C" fn edvs_voxel_grid_create(
    width: u32,
    height: u32,
    num_bins: u32,
) -> *mut VoxelGrid {
    let result = panic::catch_unwind(|| Box::new(VoxelGrid::new(width, height, num_bins)));
    match result {
        Ok(vg) => Box::into_raw(vg),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `vg` must be a valid, non-null pointer returned by `edvs_voxel_grid_create`.
#[no_mangle]
pub unsafe extern "C" fn edvs_voxel_grid_set_time_window(
    vg: *mut VoxelGrid,
    t_start: i64,
    t_end: i64,
) {
    if vg.is_null() {
        return;
    }
    let vg_ref = &mut *vg;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| {
        vg_ref.set_time_window(t_start, t_end)
    }));
}

/// # Safety
/// - `vg` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_voxel_grid_add_event(vg: *mut VoxelGrid, event: *const Event) {
    if vg.is_null() || event.is_null() {
        return;
    }
    let vg_ref = &mut *vg;
    let ev_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| vg_ref.add_event(ev_ref)));
}

/// # Safety
/// - `vg` must be valid. Returns pointer to internal data valid until next mutation.
/// - `out_len` must be valid if non-null.
#[no_mangle]
pub unsafe extern "C" fn edvs_voxel_grid_get_grid(
    vg: *const VoxelGrid,
    out_len: *mut usize,
) -> *const f32 {
    if vg.is_null() {
        return std::ptr::null();
    }
    let result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
        let vg_ref = &*vg;
        let grid = vg_ref.get_grid();
        if !out_len.is_null() {
            *out_len = grid.len();
        }
        grid.as_ptr()
    }));
    match result {
        Ok(ptr) => ptr,
        Err(_) => std::ptr::null(),
    }
}

/// # Safety
/// - `vg` must be a valid pointer returned by `edvs_voxel_grid_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_voxel_grid_reset(vg: *mut VoxelGrid) {
    if !vg.is_null() {
        let vg_ref = &mut *vg;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| vg_ref.reset()));
    }
}

/// # Safety
/// - `vg` must be a valid pointer returned by `edvs_voxel_grid_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_voxel_grid_destroy(vg: *mut VoxelGrid) {
    if !vg.is_null() {
        drop(Box::from_raw(vg));
    }
}

// --- HarrisCornerDetector FFI ---

/// Creates a Harris corner detector.
///
/// Returns null if parameters are invalid.
#[no_mangle]
pub extern "C" fn edvs_corner_detector_create(
    width: u32,
    height: u32,
    harris_k: f64,
    threshold: f64,
) -> *mut HarrisCornerDetector {
    let result = panic::catch_unwind(|| {
        Box::new(HarrisCornerDetector::new(width, height, harris_k, threshold))
    });
    match result {
        Ok(d) => Box::into_raw(d),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `det` and `event` must be valid, non-null pointers.
///
/// Returns true if the event is a corner. If `out_response` is non-null,
/// writes the Harris response value.
#[no_mangle]
pub unsafe extern "C" fn edvs_corner_detector_process(
    det: *mut HarrisCornerDetector,
    event: *const Event,
    out_response: *mut f64,
) -> bool {
    if det.is_null() || event.is_null() {
        return false;
    }
    let d_ref = &mut *det;
    let ev_ref = &*event;
    let result =
        panic::catch_unwind(panic::AssertUnwindSafe(|| d_ref.process(ev_ref))).unwrap_or(None);
    match result {
        Some(response) => {
            if !out_response.is_null() {
                *out_response = response;
            }
            true
        }
        None => false,
    }
}

/// # Safety
/// - `det` must be a valid pointer returned by `edvs_corner_detector_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_corner_detector_reset(det: *mut HarrisCornerDetector) {
    if !det.is_null() {
        let d_ref = &mut *det;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| d_ref.reset()));
    }
}

/// # Safety
/// - `det` must be a valid pointer returned by `edvs_corner_detector_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_corner_detector_destroy(det: *mut HarrisCornerDetector) {
    if !det.is_null() {
        drop(Box::from_raw(det));
    }
}

// --- FrequencyEstimator FFI ---

/// Creates a per-pixel frequency estimator.
///
/// Returns null if parameters are invalid.
#[no_mangle]
pub extern "C" fn edvs_frequency_estimator_create(
    width: u32,
    height: u32,
    history_len: u32,
) -> *mut FrequencyEstimator {
    let result =
        panic::catch_unwind(|| Box::new(FrequencyEstimator::new(width, height, history_len)));
    match result {
        Ok(fe) => Box::into_raw(fe),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `fe` and `event` must be valid, non-null pointers.
#[no_mangle]
pub unsafe extern "C" fn edvs_frequency_estimator_record(
    fe: *mut FrequencyEstimator,
    event: *const Event,
) {
    if fe.is_null() || event.is_null() {
        return;
    }
    let fe_ref = &mut *fe;
    let ev_ref = &*event;
    let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| fe_ref.record(ev_ref)));
}

/// # Safety
/// - `fe` must be a valid, non-null pointer.
///
/// Returns the estimated frequency in Hz at pixel (x, y), or -1.0 if
/// not enough data is available.
#[no_mangle]
pub unsafe extern "C" fn edvs_frequency_estimator_estimate(
    fe: *const FrequencyEstimator,
    x: u16,
    y: u16,
) -> f64 {
    if fe.is_null() {
        return -1.0;
    }
    let fe_ref = &*fe;
    panic::catch_unwind(panic::AssertUnwindSafe(|| {
        fe_ref.estimate_frequency(x, y).unwrap_or(-1.0)
    }))
    .unwrap_or(-1.0)
}

/// # Safety
/// - `fe` must be a valid pointer returned by `edvs_frequency_estimator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_frequency_estimator_reset(fe: *mut FrequencyEstimator) {
    if !fe.is_null() {
        let fe_ref = &mut *fe;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| fe_ref.reset()));
    }
}

/// # Safety
/// - `fe` must be a valid pointer returned by `edvs_frequency_estimator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_frequency_estimator_destroy(fe: *mut FrequencyEstimator) {
    if !fe.is_null() {
        drop(Box::from_raw(fe));
    }
}

// --- OpticalFlowEstimator FFI ---

/// Creates an optical flow estimator.
///
/// Returns null if parameters are invalid.
#[no_mangle]
pub extern "C" fn edvs_optical_flow_create(
    width: u32,
    height: u32,
    radius: u32,
) -> *mut OpticalFlowEstimator {
    let result =
        panic::catch_unwind(|| Box::new(OpticalFlowEstimator::new(width, height, radius)));
    match result {
        Ok(of) => Box::into_raw(of),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// - `of` and `event` must be valid, non-null pointers.
///
/// Returns true if flow was estimated. Writes vx, vy (pixels/second) to
/// `out_vx` and `out_vy` if non-null.
#[no_mangle]
pub unsafe extern "C" fn edvs_optical_flow_process(
    of: *mut OpticalFlowEstimator,
    event: *const Event,
    out_vx: *mut f64,
    out_vy: *mut f64,
) -> bool {
    if of.is_null() || event.is_null() {
        return false;
    }
    let of_ref = &mut *of;
    let ev_ref = &*event;
    let result =
        panic::catch_unwind(panic::AssertUnwindSafe(|| of_ref.process(ev_ref))).unwrap_or(None);
    match result {
        Some(flow) => {
            if !out_vx.is_null() {
                *out_vx = flow.vx;
            }
            if !out_vy.is_null() {
                *out_vy = flow.vy;
            }
            true
        }
        None => false,
    }
}

/// # Safety
/// - `of` must be a valid pointer returned by `edvs_optical_flow_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_optical_flow_reset(of: *mut OpticalFlowEstimator) {
    if !of.is_null() {
        let of_ref = &mut *of;
        let _ = panic::catch_unwind(panic::AssertUnwindSafe(|| of_ref.reset()));
    }
}

/// # Safety
/// - `of` must be a valid pointer returned by `edvs_optical_flow_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_optical_flow_destroy(of: *mut OpticalFlowEstimator) {
    if !of.is_null() {
        drop(Box::from_raw(of));
    }
}
