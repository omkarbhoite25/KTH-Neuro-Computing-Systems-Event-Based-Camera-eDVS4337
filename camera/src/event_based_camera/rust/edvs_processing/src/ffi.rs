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
